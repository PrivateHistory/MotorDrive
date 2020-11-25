
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
//#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
////-------
#include <asm/io.h>
#include <mach/platform.h>

//definition for fast use

//function for pin select
#define GPIO_INPUT     0b000
#define GPIO_OUTPUT    0b001
#define GPIO_ALT_FUNC0 0b100
#define GPIO_ALT_FUNC1 0b101
#define GPIO_ALT_FUNC2 0b110
#define GPIO_ALT_FUNC3 0b111
#define GPIO_ALT_FUNC4 0b011
#define GPIO_ALT_FUNC5 0b010

//periferals
#define PERIPH_BASE 0x3F000000
#define GPIO_BASE (PERIPH_BASE + 0x200000)
#define PWM_BASE (PERIPH_BASE + 0x20C000)
#define PWM_CLK_BASE (PERIPH_BASE + 0x101000)
//pulse width modulation clock control
#define PWMCLK_CNTL	40
//pulse width modulation clock division
#define PWMCLK_DIV	41

#define INA 11; //set GPIO 17 as Input A for rotation direction of the motor
#define INB 13; //SET GPIO 27 as Input B for rotation direction of the motor


//structure for the register
struct gpio_register_base{
uint32_t GPIOSEL[6];  //first 6 select register with 32 bits
uint32_t RESERVED;   //reserved register
uint32_t GPIOSET[2]; //2 next set registered
uint32_t RESERVED_1; //reserved register
uint32_t GPIOCLR[2]; //register to remove/clear gpio
};


//structure for PWM register

struct PWM_Register
{
	uint32_t CTL;
	uint32_t STA;
	uint32_t DMAC;
	uint32_t reserved0;
	uint32_t RNG1;
	uint32_t DAT1;
	uint32_t FIF1;
	uint32_t reserved1;
	uint32_t RNG2;
	uint32_t DAT2;
} *s_pPwmRegisters;

//structure for control register
struct S_PWM_CTL{
	unsigned PWEN1 : 1;
	unsigned MODE1 : 1;
	unsigned RPTL1 : 1;
	unsigned SBIT1 : 1;
	unsigned POLA1 : 1;
	unsigned USEF1 : 1;
	unsigned CLRF1 : 1;
	unsigned MSEN1 : 1;
	unsigned PWEN2 : 1;
	unsigned MODE2 : 1;
	unsigned RPTL2 : 1;
	unsigned SBIT2 : 1;
	unsigned POLA2 : 1;
	unsigned USEF2 : 1;
	unsigned Reserved1 : 1;
	unsigned MSEN2 : 1;
	unsigned Reserved2 : 16;
} *pwm_ctl;



//structure for status of PWM
struct S_PWM_STA {
	unsigned FULL1 : 1;
	unsigned EMPT1 : 1;
	unsigned WERR1 : 1;
	unsigned RERR1 : 1;
	unsigned GAPO1 : 1;
	unsigned GAPO2 : 1;
	unsigned GAPO3 : 1;
	unsigned GAPO4 : 1;
	unsigned BERR : 1;
	unsigned STA1 : 1;
	unsigned STA2 : 1;
	unsigned STA3 : 1;
	unsigned STA4 : 1;
	unsigned Reserved : 19;
} *pwm_sta;
//pointer to PWM clocks
volatile unsigned int *s_pPwmClkRegisters;

//duty cycle
int duty_cycle=50;
//create a point to the structure of register
struct gpio_register_base* gpio_struct;

struct boilerplate_dev  {
	struct cdev cdev;
};

// declaration of entry points
static int boilerplate_open(struct inode *inode, struct file *filep);
static ssize_t boilerplate_read(struct file *filep, char *buf, size_t count, loff_t *f_pos);
static ssize_t boilerplate_write(struct file *filep, const char *bur, size_t count, loff_t *f_pos);
static int boilerplate_release(struct inode *inode, struct file *filep);

// file operations structure
static struct file_operations boilerplate_fops = {
	.owner		= THIS_MODULE,
	.open 		= boilerplate_open,
	.release 	= boilerplate_release,
	.read 		= boilerplate_read,
	.write 		= boilerplate_write,
};

// prototypes
static int boilerplate_init(void);
static void boilerplate_exit(void);

// global variables
struct boilerplate_dev *boilerplate_devp;
static dev_t first;
static struct class *boilerplate_class;
//message received from userspace
static char message_received_from_userspace[256] = {0};
//size of received message
static int received_message_length=0;

//function for direction
static void rotation_direction(int direction){
//direction=1 clockwise and direction=0 anticlockwise
//the values are taken from truth table
if(direction){
gpio_struct->GPIOSET[INA]=1;
gpio_struct->GPIOSET[INB]=0;
}
else
{
gpio_struct->GPIOSET[INA]=0;
gpio_struct->GPIOSET[INB]=1;

}

}




//function to set the GPIO

static void SetGPIO_with_function(int GPIO, int functionCode)
{
	//ten GPIO per register
    int registerIndex = GPIO / 10;
		//3 bits for GPIO per register
    int Gpio_bits = (GPIO % 10) * 3;
    //old value of the register
    unsigned oldValue = gpio_struct->GPFSEL[registerIndex];
		//bit for 111 which in negation correspond with 000 so alt 0
    unsigned  bits_for_alt0= 0b111 << Gpio_bits;
		//for explication of this code look to the second function setGPIO
    gpio_struct->GPFSEL[registerIndex] = (oldValue & ~bits_for_alt0) | ((functionCode <<Gpio_bits) & bits_for_alt0);
}





//function to set the frequency of the PWM
static void PWM_clock_frequency(uint32_t div) {
	//get the pointer to the controll register of PWM
	*(s_pPwmClkRegisters+PWMCLK_CNTL) = 0x5A000020; /* set bit 5 to 1 ,kill the clock*/

  //stop the PWM
	pwm_ctl->PWEN1 = 0;
	//wait for clock to stop
	udelay(10);

	//get the register of division for clock
	*(s_pPwmClkRegisters+PWMCLK_DIV) = 0x5A000000 | ( div << 12 ); //set the division for clocks starting with pin 12
//get the register for time control
	*(s_pPwmClkRegisters+PWMCLK_CNTL) = 0x5A000011; //choose the oscillator as source of frequency 19.2Mhz thsi corresponds with setting pin 1 with 1
	//also strat the clock generator

	//wait for changes to take place
	udelay(10);
	/*
	* Set GPIO 19 as PWM,set the alt function to 5
	*/
	SetGPIOFunction(19, GPIO_INPUT); /* Set ALT = 0 */
	SetGPIOFunction(19, GPIO_ALT_FUNC5); /* Or in '5 ' */
  //set the other bits in PWM control register
	pwm_ctl->MODE1 = 0;
	pwm_ctl->RPTL1 = 0;
	pwm_ctl->SBIT1 = 0;
	pwm_ctl->POLA1 = 0;
	pwm_ctl->USEF1 = 0;
	pwm_ctl->MSEN1 = 0;
	pwm_ctl->CLRF1 = 1;
}





//function to set the PWM duty cycle as ratio  of n and m
static void PWM_duty_cycle(unsigned n,unsigned m) {
   //stop the PWM
	pwm_ctl->PWEN1 = 0; /* Disable */

	//Set the range for PWM as m bits
	s_pPwmRegisters->RNG1 = m;
	//Set the n bits as n
	s_pPwmRegisters->DAT1 = n;

// if channel 1 is not transimited data ,some error happen(reset everything )
	if ( !pwm_sta->STA1 ) {
		//bit empty error occured
		if ( pwm_sta->RERR1 ) pwm_sta->RERR1 = 1; //reset the error bit
		//a full error occured
		if ( pwm_sta->WERR1 ) pwm_sta->WERR1 = 1; //reset the error bit

		//error while writing to register vis PM
		if ( pwm_sta->BERR ) pwm_sta->BERR = 1;  //reset the error bit
	}
	//wait for 10ms
	udelay(10);
	//start the PWM
	pwm_ctl->PWEN1 = 1;
}





// OPEN function
static int boilerplate_open(struct inode *inode, struct file *filep){

	return 0;
}

// RELEASE function
static int boilerplate_release(struct inode *inode, struct file *filep){

	return 0;
}

// READ function(read from kernelspace to user spacef)
static ssize_t boilerplate_read(struct file *filep, char *buf, size_t count, loff_t *f_pos){
	ssize_t retval =received_message_length; //message sent to userspace length
   int error_count = 0;
   // copy_to_user has the format ( * to, *from, size) and returns 0 on success
   error= copy_to_user(buf, message_received_from_userspace,received_message_length);

   if (error==0){            // if true then there are no errors
      printk(KERN_INFO "Sent %d characters to the user\n",received_message_length); //for debuging
      return (retval=0);  // clear the position to the start and return 0
   }
   else {
      printk(KERN_INFO "Failed to send %d characters to the user\n", error);
      return -EFAULT;
   }
   //return the message length
	return retval;
}

// WRITE function

//Assignmnet3
static ssize_t boilerplate_write(struct file *filep, const char *buf, size_t count, loff_t *f_pos){
   sprintf(message_received_from_userspace, "%s(%zu letters)", buf, count);   //append the received message from buffer to global varibale message_received_from_userspace

   duty_cycle=atoi(buf);
   //store the size of message received from userspace
   received_message_length=strlen(message_received_from_userspace);

   printk(KERN_INFO "Received %zu characters from the user\n and the data \i", count,duty_cycle); //debuging check that the string is received
   return count;
}

// INIT function
static int __init boilerplate_init(void)
{
	//------------code for led---------/
	//get the maping of real and virtual addressing for GPIO
	gpio_struct = (struct gpio_register_base*)ioremap(GPIO_BASE, sizeof(struct clone*));
	//get the maping of real and virtual addressing for PWM register
	s_pPwmRegisters = (struct PWM_Register *)ioremap(PWM_BASE, sizeof(struct pwmRegisters));
  //get the control register of PWM
	pwm_ctl = (struct S_PWM_CTL *) &s_pPwmRegisters -> CTL;
  //get the status register of PWM
	pwm_sta = (struct S_PWM_STA *) &s_pPwmRegisters -> STA;
  //get the mapping for clock register
	s_pPwmClkRegisters = ioremap(PWM_CLK_BASE, 4096);


  //set the clock to 10KHz
	PWM_clock_frequency(1920);
	//set teh rotation clockwise
	rotation_direction(1);
	//set the duty cycle to 50%
  int duty_value=1024*duty_cycle/100;
	PWM_duty_cycle(duty_value,1024);
	int ret = 0;

	if(alloc_chrdev_region(&first, 0, 1, DEVICE_NAME) < 0){
		pr_info("Cannot register device\n");
		return -1;
	}

	if((boilerplate_class = class_create(THIS_MODULE, DEVICE_NAME)) == NULL){
		pr_info("Cannot create class %s\n", DEVICE_NAME);
		unregister_chrdev_region(first, 1);
		return -EINVAL;
	}

	boilerplate_devp = kmalloc(sizeof(struct boilerplate_dev), GFP_KERNEL);

	if(!boilerplate_devp){
		pr_info("Bad kmalloc\n");
		return -ENOMEM;
	}

	boilerplate_devp->cdev.owner = THIS_MODULE;

	cdev_init(&boilerplate_devp->cdev, &boilerplate_fops);

	if((ret = cdev_add(&boilerplate_devp->cdev, first, 1))){
		pr_alert("Error %d adding cdev\n", ret);
		device_destroy(boilerplate_class, first);
		class_destroy(boilerplate_class);
		unregister_chrdev_region(first, 1);

		return ret;
	}

	if(device_create(boilerplate_class, NULL, first, NULL, DEVICE_NAME) == NULL){
		class_destroy(boilerplate_class);
		unregister_chrdev_region(first, 1);

		return -1;
	}

	pr_info("Boilerplate driver initialized\n");

	return 0;
}






static void SetGPIOOutputValue(int GPIO, bool outputValue)
{
    if (outputValue)
        s_pGpioRegisters->GPSET[GPIO / 32] = (1 << (GPIO % 32));
    else
        s_pGpioRegisters->GPCLR[GPIO / 32] = (1 << (GPIO % 32));
}








///set the gpio as output or input



static void setGPIO(int GPIO, int selectfunction){
//select function determines if the GPIO is output or input

	int rIndex = GPIO / 10;  //every select register has 10 GPIO on it so GPIO number div 10 will give the index of select register

	int bit_value = (GPIO % 10) * 3;   //every select register assigns 3 bits for every GPIO ,this will give the start index of the 3 bits


    ///-------------///
    //3 bits mentioned above can get different values(8 to be exact 2^3)
    //now to select gpio as input the 3 bits have to be 000
    // for output they have to be 001 ,and the rest are for alternative function
    ////----------////


    //select function 0 means input and 1 means output
    if(selectfunction==0){
  //set the GPIO to input
  gpio_struct->GPIOSEL[rIndex] &=~(7<<(bit_value));
  //this equations works like this.It was stated that for input you have to set the 3 bits belonging to GPIO to 000
  //moreover this bits have the starting index at bit_value ,so in order to achive this it is only needed
  //to get 7(which in power two is 111) to left shift by bit_value and to negate it so the three bits are 000
  //Because only the 3 bits have to be changed and the rest to remain 0 the and is used

}
else
{
  gpio_struct->GPIOSEL[rIndex]|=(1<<(bit_value));
   //this equations works like this.It was stated that for input you have to set the 3 bits belonging to GPIO to 001
  //moreover this bits have the starting index at bit_value ,so in order to achive this it is only needed
  //to get 1(which in power two is 001) to left shift by bit_value
  //Because only the third bit matter (1 has to be the third bit to be output) only or has to be used

}


}


// EXIT function
static void __exit boilerplate_exit(void)
{
	unregister_chrdev_region(first, 1);
	iounmap(gpio_struct);
	iounmap(s_pPwmRegisters);
	iounmap(s_pPwmClkRegisters);
	device_destroy(boilerplate_class, first);

	class_destroy(boilerplate_class);

	pr_info("Boilerplate driver removed\n");
}

module_init(boilerplate_init);
module_exit(boilerplate_exit);

MODULE_LICENSE("GPL");
