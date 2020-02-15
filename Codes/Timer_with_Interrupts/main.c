#include <stdbool.h>
#include <stdint.h>
#include "lm4f120h5qr.h"
//#include "TM4C123GH6PM.h"
//#include "core_cm4.h"
#include "delay.h"

#define Source_Clock_on_Port_A		SYSCTL_RCGCGPIO_R |= (1U << 0)
#define Source_Clock_on_Port_B		SYSCTL_RCGCGPIO_R |= (1U << 1)
#define Source_Clock_on_Port_C		SYSCTL_RCGCGPIO_R |= (1U << 2)
#define Source_Clock_on_Port_D		SYSCTL_RCGCGPIO_R |= (1U << 3)
#define Source_Clock_on_Port_E		SYSCTL_RCGCGPIO_R |= (1U << 4)
#define Source_Clock_on_Port_F		SYSCTL_RCGCGPIO_R |= (1U << 5)

#define Activate_AHB_for_GPIO_A		SYSCTL_GPIOHBCTL_R |= (1U << 0)
#define Activate_AHB_for_GPIO_B		SYSCTL_GPIOHBCTL_R |= (1U << 1)
#define Activate_AHB_for_GPIO_C		SYSCTL_GPIOHBCTL_R |= (1U << 2)
#define Activate_AHB_for_GPIO_D		SYSCTL_GPIOHBCTL_R |= (1U << 3)
#define Activate_AHB_for_GPIO_E		SYSCTL_GPIOHBCTL_R |= (1U << 4)
#define Activate_AHB_for_GPIO_F		SYSCTL_GPIOHBCTL_R |= (1U << 5)

#define SW1		(1U << 4)
#define SW2		(1U << 0)
#define LED_RED		(1U << 1)
#define LED_BLUE	(1U << 2)
#define LED_GREEN	(1U << 3)

#define LCD_enable_pin	(1U << 2)	// Pin d2	Pin 6 on LCD
#define LCD_RS_pin	(1U << 3)	// Pin d3	Pin 4 on LCD
#define LCD_RW_pin	(1U << 6)	// Pin d6	Pin 5 on LCD

#define	LCD_Enable_Port				GPIO_PORTD_AHB_DATA_BITS_R
#define LCD_RS_Port				GPIO_PORTD_AHB_DATA_BITS_R
#define	LCD_RW_Port				GPIO_PORTD_AHB_DATA_BITS_R
#define LCD_DataBus				GPIO_PORTE_AHB_DATA_R

#define DataDir_LCD_Enable			GPIO_PORTD_AHB_DIR_R 
#define DataDir_LCD_RS				GPIO_PORTD_AHB_DIR_R
#define DataDir_LCD_RW				GPIO_PORTD_AHB_DIR_R	
#define LCD_DataBus_AHB_Data_Dir		GPIO_PORTE_AHB_DIR_R

#define GPIO_LCD_enable_pin_AHB_DEN		GPIO_PORTD_AHB_DEN_R 
#define GPIO_LCD_RS_pin_AHB_DEN			GPIO_PORTD_AHB_DEN_R
#define GPIO_LCD_RW_pin_AHB_DEN 		GPIO_PORTD_AHB_DEN_R
#define GPIO_LCD_DataBus_AHB_DEN		GPIO_PORTE_AHB_DEN_R

#define LCD_DataBus_AHB_AFSEL			GPIO_PORTE_AHB_AFSEL_R
#define LCD_DataBus_AHB_2MilAmp			GPIO_PORTE_AHB_DR2R_R
#define LCD_DataBus_AHB_4MilAmp			GPIO_PORTE_AHB_DR4R_R
#define LCD_DataBus_AHB_8MilAmp			GPIO_PORTE_AHB_DR8R_R
#define LCD_DataBus_AHB_OD_Config		GPIO_PORTE_AHB_ODR_R
#define LCD_DataBus_AHB_PU_Config		GPIO_PORTE_AHB_PUR_R
#define LCD_DataBus_AHB_PD_Config		GPIO_PORTE_AHB_PDR_R

  //DataDir_LCD_DataBus &= ~(0xFF);
  //GPIO_PORTD_AHB_AFSEL_R &= ~(0xFF);
  //GPIO_PORTD_AHB_DR2R_R |= (0xFF);
  //GPIO_PORTD_AHB_ODR_R &= ~(0xFF);
  //GPIO_PORTD_AHB_PUR_R |= (0xFF);

unsigned fact(unsigned n);
bool p = false;

void TIMER0A_Handler (void);
  ///////////////////////////////////////////////////////////////
  /////////LCD///////////////////////////////////////////////////
  void Check_if_LCD_Busy(void);
  void Flash_LCD(void);
  void Command_LCD(char command);
  void Send_Character_to_LCD(char character);
  void Send_Integer_to_LCD(int integer);
  void Send_String_to_LCD(char *stringOfCharacters);
  void Send_String_Instantly_to_LCD(char *stringOfCharacters);
  
  void Initialize_Data_and_control_Ports_for_LCDs(void);
  void Run_LCD_initialization_Routine(void);
  void Clear_Screen(void);
  
  void Test_LCD(void);
  ///////////////////////////////////////////////////////////////
  /////////LCD///////////////////////////////////////////////////
volatile uint8_t tempTimerAParam = 0x02; //define a temperaoary variable to blink the LEDs

  void main(){
    // initialization and configuration of timer0
    SYSCTL_RCGCTIMER_R |= (1U<<0);			// set the respective bit in rcgctimer to use GPTM by enabling clock
    
    TIMER0_CTL_R &- ~(TIMER_CTL_TAEN);			// ensure that the timer is disabled before making any changes
    // alternatively TIMER0_CTL_R &- ~(1U<<0);		// ensure that the timer is disabled before making any changes    
      
    TIMER0_CFG_R &= TIMER_CFG_32_BIT_TIMER;		// GPTMCFG set to 0x00000000
    // alternatively TIMER0_CFG_R = 0x00000000;		// GPTMCFG set to 0x00000000
    
    TIMER0_TAMR_R |= TIMER_TAMR_TAMR_PERIOD;		// value of 0x2 Configured in the TnMR field in GPTMTnMR for Periodic mode
    // alternatively TIMER0_TAMR_R |= (0x02<<0);	// value of 0x2 Configured in the TnMR field in GPTMTnMR for Periodic mode
    
    TIMER0_TAMR_R |= TIMER_TAMR_TACDIR;			//  counter counts up instead of down as per the default setting		
    // alternatively     TIMER0_TAMR_R |= (1U << 4);	//  counter counts up instead of down as per the default setting
    
    TIMER0_TAILR_R = 0x00F424F0;			// the counter will count upto 16000000
    
    TIMER0_IMR_R |= TIMER_IMR_TATOIM;			// GPTM Timer A Time-Out Interrupt enabled
    NVIC_EN0_R |= NVIC_EN0_INT19;
    TIMER0_CTL_R |= (TIMER_CTL_TAEN);			// turn on the timer 
    // alternatively TIMER0_CTL_R |= (1U>>0);		// turn on the timer 	    

/*    
1. Ensure the timer is disabled (the TnEN bit in the GPTMCTL register is cleared) before making
any changes.
2. Write the GPTM Configuration Register (GPTMCFG) with a value of 0x0000.0000.
3. Configure the TnMR field in the GPTM Timer n Mode Register (GPTMTnMR):
a. Write a value of 0x1 for One-Shot mode.
b. Write a value of 0x2 for Periodic mode.
4. Optionally configure the TnSNAPS, TnWOT, TnMTE, and TnCDIR bits in the GPTMTnMR register
to select whether to capture the value of the free-running timer at time-out, use an external
trigger to start counting, configure an additional trigger or interrupt, and count up or down.
5. Load the start value into the GPTM Timer n Interval Load Register (GPTMTnILR).
6. If interrupts are required, set the appropriate bits in the GPTM Interrupt Mask Register
(GPTMIMR).
7. Set the TnEN bit in the GPTMCTL register to enable the timer and start counting.
8. Poll the GPTMRIS register or wait for the interrupt to be generated (if enabled). In both cases,
the status flags are cleared by writing a 1 to the appropriate bit of the GPTM Interrupt Clear
Register (GPTMICR). 
*/
  
    ////////////////////////////////
    ////////////////////////////////
    ////////////////////////////////
    ////////////////////////////////
  //GPIO_PORTD_AHB_LOCK_R = (0x4C4F434B);
  //GPIO_PORTD_AHB_CR_R |= (1U << 7);	// UNLOCK NMI PIN ON LCD DATABUS CR
  
  SYSCTL_RCGCGPIO_R |= (1U << 5);	// Enable clock for GPIO_F
  SYSCTL_GPIOHBCTL_R |= (1U << 5);	// Enable AHB for GPIO_F
  
  GPIO_PORTF_AHB_LOCK_R = (0x4C4F434B);
  GPIO_PORTF_AHB_CR_R |= (SW2); 	// UNLOCK SPECIAL CONSIDERATION PIN CR
  
  GPIO_PORTF_AHB_DIR_R |= ( LED_RED | LED_BLUE | LED_GREEN);
  GPIO_PORTF_AHB_DEN_R |= ( LED_RED | LED_BLUE | LED_GREEN | SW1 | SW2 );
  			
  GPIO_PORTF_AHB_DIR_R 	&= ~(SW1);
  GPIO_PORTF_AHB_AFSEL_R &= ~(SW1);
  GPIO_PORTF_AHB_DR2R_R |= (SW1);
  GPIO_PORTF_AHB_ODR_R &= ~(SW1);
  GPIO_PORTF_AHB_PUR_R |= (SW1);
  
  GPIO_PORTF_AHB_DIR_R &= ~(SW2);
  GPIO_PORTF_AHB_AFSEL_R &= ~(SW2);
  GPIO_PORTF_AHB_DR2R_R |= (SW2);
  GPIO_PORTF_AHB_ODR_R &= ~(SW2);
  GPIO_PORTF_AHB_PUR_R |= (SW2);
  //Test_LCD();
  
 // Initialize_Data_and_control_Ports_for_LCDs();
 // delay_ms(20);
 // Run_LCD_initialization_Routine();
 // delay(3000);
 // Send_String_to_LCD("hello Rhitvik !!");

/*  
    while(3){
      // in case we don't want to use the interrupt handler!!
      // polling the raw interrupt resister and read the raw interrupt status flag  
      
      if (TIMER0_RIS_R & TIMER_RIS_TATORIS){		// GPTM Timer A Time-Out Raw interrupt
      // alternatively if((TIMER0_RIS_R & 0x00000001) == 1 ){ }
	
	TIMER0_ICR_R |= TIMER_ICR_TATOCINT;		//reset int flag
	// alternatively TIMER0_ICR_R |= (0x00000001);	//reset int flag	
      }
    }
*/
  while(40){
    delay_s(2);
  }
  while(1){
    GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = LED_RED;
    
    delay_ms(1000);

    GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = LED_BLUE;

    delay_s(1);
    
    GPIO_PORTF_AHB_DATA_R = (0x08);
    //GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = 0;
   // GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = 0;   
    //GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = LED_GREEN;

    delay_ms(1000);
    
    if(GPIO_PORTF_AHB_DATA_BITS_R[SW2] == 0){
      
      delay_ms(10);
      GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = LED_BLUE;
      GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = LED_RED;    
      GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = LED_GREEN; 
   
      if(p == false){
      	p = true;
        GPIO_PORTF_AHB_LOCK_R = (0x4C4F434B);
        GPIO_PORTF_AHB_CR_R &= ~(SW2);
      }
      GPIO_PORTF_AHB_CR_R |= (SW2);
      GPIO_PORTF_AHB_PDR_R |= (SW2);
      
      delay_ms(1000);
      
    }	// if(GPIO_PORTF_AHB_DATA_BITS_R[SW2] == 0)      
  }	// while(1)

} //void main

  void TIMER0A_Handler (void){
    TIMER0_ICR_R |= TIMER_ICR_TATOCINT;		//reset int flag
	// alternatively TIMER0_ICR_R |= (0x00000001);	//reset int flag
    GPIO_PORTF_AHB_DATA_R = (tempTimerAParam);
	
	tempTimerAParam = tempTimerAParam*2;
	if(tempTimerAParam > 0x08){
	  tempTimerAParam = 0x02;
	}
  }

void Check_if_LCD_Busy(void){
  	GPIO_LCD_DataBus_AHB_DEN &= ~(0xFF);	//disable Digital pins on PORT_D
	
	LCD_DataBus_AHB_AFSEL &= ~(0xFF);
	LCD_DataBus_AHB_2MilAmp |= (0xFF);
	LCD_DataBus_AHB_OD_Config &= ~(0xFF);
	LCD_DataBus_AHB_PU_Config |= (0xFF);
   	LCD_DataBus_AHB_Data_Dir &= ~(0xFF);
	
	GPIO_LCD_DataBus_AHB_DEN |= (0xFF);	//enable Digital pins on PORT_D

  	LCD_RW_Port[LCD_RW_pin] = LCD_RW_pin;
  	LCD_RS_Port[LCD_RS_pin] = 0;
    	
	uint8_t Busy_Flag = (LCD_DataBus & 0x0F);
	
    while (Busy_Flag >= 0x08){
      	Busy_Flag = (LCD_DataBus & 0x0F); 
	Flash_LCD();
    }
    
    GPIO_LCD_DataBus_AHB_DEN &= ~(0xFF);	//disable Digital pins on PORT_D
    LCD_DataBus_AHB_Data_Dir |= 0xFF;
    GPIO_LCD_DataBus_AHB_DEN |= (0xFF);		//enable Digital pins on PORT_D

}

void Flash_LCD(void){
  	LCD_Enable_Port[LCD_enable_pin] = LCD_enable_pin;
	delay_us(50);
	LCD_Enable_Port[LCD_enable_pin] = 0;
}
void Command_LCD(char command){
	Check_if_LCD_Busy();
	LCD_DataBus &= 0xF0;
	LCD_DataBus = (( command & 0xF0) >> 4);
	LCD_RW_Port[LCD_RW_pin] = 0;
  	LCD_RS_Port[LCD_RS_pin] = 0;
	Flash_LCD();
	delay_us(200);
	LCD_DataBus |= (command & 0x0F);
	Flash_LCD();
	delay_ms(2);
	LCD_DataBus &= (0xF0);
}
void Send_Character_to_LCD(char character){
	Check_if_LCD_Busy();
	LCD_DataBus &= 0xF0;
	LCD_DataBus |= ((character & 0xF0) >> 4);
	LCD_RW_Port[LCD_RW_pin] = 0;
  	LCD_RS_Port[LCD_RS_pin] = LCD_RS_pin;
	Flash_LCD();
	delay_us(200);
	LCD_DataBus |= (character & 0x0F);
	Flash_LCD();
	delay_ms(2);
	LCD_DataBus &= (0xF0);
}
void Send_Integer_to_LCD(int integer){
	//Check_if_LCD_Busy();
	LCD_DataBus &= 0xF0;
	LCD_DataBus |= ((integer & 0xF0) >> 4);
	LCD_RW_Port[LCD_RW_pin] = 0;
  	LCD_RS_Port[LCD_RS_pin] = LCD_RS_pin;
	Flash_LCD();
	LCD_DataBus |= (integer & 0x0F);
	Flash_LCD();
	LCD_DataBus &= (0xF0);
}
void Send_String_to_LCD(char *String){
	while(*String > 0){
	  Send_Character_to_LCD(*String++);
	  delay(3000);
	}
}
void Send_String_Instantly_to_LCD(char *String){
	while(*String > 0){
	  Send_Character_to_LCD(*String++);
	  delay(300);
	}
}

void Initialize_Data_and_control_Ports_for_LCDs(void){
  	Source_Clock_on_Port_E;
	Source_Clock_on_Port_D; 
	Activate_AHB_for_GPIO_E;
	Activate_AHB_for_GPIO_D;
	
	LCD_DataBus_AHB_Data_Dir |= (0xFF);
		
	DataDir_LCD_Enable |= LCD_enable_pin;
	DataDir_LCD_RW |= LCD_RW_pin;
	DataDir_LCD_RS |= LCD_RS_pin;
	  
	GPIO_LCD_enable_pin_AHB_DEN |= LCD_enable_pin;
	GPIO_LCD_RS_pin_AHB_DEN |= LCD_RS_pin;		
	GPIO_LCD_RW_pin_AHB_DEN |= LCD_RW_pin;
	GPIO_LCD_DataBus_AHB_DEN |= (0xFF);	//enable Digital pins on PORT_E
	
	//GPIO_PORTB_AHB_DATA_R &= ~LCD_enable_pin; works !!	  
	//GPIO_PORTB_AHB_DATA_R &= 0xFF;
	
	LCD_Enable_Port[LCD_enable_pin] = 0;
	LCD_RW_Port[LCD_RW_pin] = 0;
	LCD_RS_Port[LCD_RS_pin] = 0;
}
void Run_LCD_initialization_Routine(void){
  delay_ms(10);
  Command_LCD(0x02);		/* send for 4 bit initialization of LCD  */
  delay_ms(2);
  Command_LCD(0x33);
  delay_ms(2);
  Command_LCD(0x32);
  delay_ms(2);
  Command_LCD(0x28);              /* 2 line, 5*7 matrix in 4-bit mode */
  delay_ms(2);
  Command_LCD(0x0c);              /* Display on cursor off*/
  delay_ms(2);
  Command_LCD(0x06);              /* Increment cursor (shift cursor to right)*/
  delay_ms(2);
  Clear_Screen();
  //Command_LCD(0x38);
  //delay(50);
  //Command_LCD(0x0C);
  //delay(50);
}
void Clear_Screen(void){
  delay_ms(2);
  Command_LCD(0x01); //clear screen
  delay_ms(2);
}

void Test_LCD(void){
  Initialize_Data_and_control_Ports_for_LCDs();
  while(3){
  	LCD_Enable_Port[LCD_enable_pin] = LCD_enable_pin;
	LCD_RW_Port[LCD_RW_pin] = LCD_RW_pin;
	LCD_RS_Port[LCD_RS_pin] = LCD_RS_pin;
	LCD_DataBus &= ~(0xFF);
	
	delay(100000);
	
	LCD_Enable_Port[LCD_enable_pin] = 0;
	LCD_RW_Port[LCD_RW_pin] = 0;
	LCD_RS_Port[LCD_RS_pin] = 0;
	LCD_DataBus |= (0xFF);

	delay(100000);

  }
}