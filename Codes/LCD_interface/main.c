#include <stdbool.h>
#include "lm4f120h5qr.h"
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


//	PORTD as LCD data bus				IN/OUT
//	PinB4	LCD pin6	Enable			OUTPUT
//	pinE4	LCD pin5	RW			OUTPUT
//	pinE5	LCD pin4	RS/Register_Select	OUTPUT

#define LCD_enable_pin	(1U << 2)	// Pin d2	Pin 6 on LCD
#define LCD_RS_pin	(1U << 3)	// Pin d3	Pin 4 on LCD
#define LCD_RW_pin	(1U << 6)	// Pin d6	Pin 5 on LCD

#define	LCD_Enable_Port				GPIO_PORTD_AHB_DATA_BITS_R
#define LCD_RS_Port				GPIO_PORTD_AHB_DATA_BITS_R
#define	LCD_RW_Port				GPIO_PORTD_AHB_DATA_BITS_R
#define LCD_DataBus				GPIO_PORTB_AHB_DATA_R

#define DataDir_LCD_Enable			GPIO_PORTD_AHB_DIR_R 
#define DataDir_LCD_RS				GPIO_PORTD_AHB_DIR_R
#define DataDir_LCD_RW				GPIO_PORTD_AHB_DIR_R	
#define LCD_DataBus_AHB_Data_Dir		GPIO_PORTB_AHB_DIR_R

#define GPIO_LCD_enable_pin_AHB_DEN		GPIO_PORTD_AHB_DEN_R 
#define GPIO_LCD_RS_pin_AHB_DEN			GPIO_PORTD_AHB_DEN_R
#define GPIO_LCD_RW_pin_AHB_DEN 		GPIO_PORTD_AHB_DEN_R
#define GPIO_LCD_DataBus_AHB_DEN		GPIO_PORTB_AHB_DEN_R

#define LCD_DataBus_AHB_AFSEL			GPIO_PORTB_AHB_AFSEL_R
#define LCD_DataBus_AHB_2MilAmp			GPIO_PORTB_AHB_DR2R_R
#define LCD_DataBus_AHB_4MilAmp			GPIO_PORTB_AHB_DR4R_R
#define LCD_DataBus_AHB_8MilAmp			GPIO_PORTB_AHB_DR8R_R
#define LCD_DataBus_AHB_OD_Config		GPIO_PORTB_AHB_ODR_R
#define LCD_DataBus_AHB_PU_Config		GPIO_PORTB_AHB_PUR_R
#define LCD_DataBus_AHB_PD_Config		GPIO_PORTB_AHB_PDR_R

  //DataDir_LCD_DataBus &= ~(0xFF);
  //GPIO_PORTD_AHB_AFSEL_R &= ~(0xFF);
  //GPIO_PORTD_AHB_DR2R_R |= (0xFF);
  //GPIO_PORTD_AHB_ODR_R &= ~(0xFF);
  //GPIO_PORTD_AHB_PUR_R |= (0xFF);

unsigned fact(unsigned n);
bool p = false;


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



  void main(){
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
  Initialize_Data_and_control_Ports_for_LCDs();
  delay(100);
  Run_LCD_initialization_Routine();
  delay(3000);
  Send_String_to_LCD("Prof. Campisi is    Awesome");
     
  while(1){
    GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = LED_RED;
    
    delay(1000000);

    GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = LED_BLUE;

    delay(1000000);
    GPIO_PORTF_AHB_DATA_R = (0x08);
    //GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = 0;
   // GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = 0;   
    //GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = LED_GREEN;

    delay(1000000);
    
    if(GPIO_PORTF_AHB_DATA_BITS_R[SW2] == 0){
      
      delay(10000);
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
      
      delay(1000000);
      
    }	// if(GPIO_PORTF_AHB_DATA_BITS_R[SW2] == 0)      
  }	// while(1)

} //void main

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
    
    while (LCD_DataBus >= 0x80){
	    Flash_LCD();
	    //delay(1000000);

    }
    GPIO_LCD_DataBus_AHB_DEN &= ~(0xFF);	//disable Digital pins on PORT_D
    LCD_DataBus_AHB_Data_Dir |= 0xFF;
    GPIO_LCD_DataBus_AHB_DEN |= (0xFF);		//enable Digital pins on PORT_D

}

void Flash_LCD(void){
  	LCD_Enable_Port[LCD_enable_pin] = LCD_enable_pin;
	delay(5);
	LCD_Enable_Port[LCD_enable_pin] = 0;
}
void Command_LCD(char command){
	Check_if_LCD_Busy();
	LCD_DataBus |= command;
	LCD_RW_Port[LCD_RW_pin] = 0;
  	LCD_RS_Port[LCD_RS_pin] = 0;
	Flash_LCD();
	LCD_DataBus &= ~(0xFF);
}
void Send_Character_to_LCD(char character){
	Check_if_LCD_Busy();
	LCD_DataBus |= character;
	LCD_RW_Port[LCD_RW_pin] = 0;
  	LCD_RS_Port[LCD_RS_pin] = LCD_RS_pin;
	Flash_LCD();
	LCD_DataBus &= ~(0xFF);
}
void Send_Integer_to_LCD(int integer){
	Check_if_LCD_Busy();
	LCD_DataBus |= integer;
	LCD_RW_Port[LCD_RW_pin] = 0;
  	LCD_RS_Port[LCD_RS_pin] = LCD_RS_pin;
	Flash_LCD();
	LCD_DataBus &= ~(0xFF);
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
  	Source_Clock_on_Port_B;
	Source_Clock_on_Port_D; 
	Activate_AHB_for_GPIO_B;
	Activate_AHB_for_GPIO_D;
	
	LCD_DataBus_AHB_Data_Dir |= (0xFF);
		
	DataDir_LCD_Enable |= LCD_enable_pin;
	DataDir_LCD_RW |= LCD_RW_pin;
	DataDir_LCD_RS |= LCD_RS_pin;
	  
	GPIO_LCD_enable_pin_AHB_DEN |= LCD_enable_pin;
	GPIO_LCD_RS_pin_AHB_DEN |= LCD_RS_pin;		
	GPIO_LCD_RW_pin_AHB_DEN |= LCD_RW_pin;
	GPIO_LCD_DataBus_AHB_DEN |= (0xFF);	//enable Digital pins on PORT_D
	
	//GPIO_PORTB_AHB_DATA_R &= ~LCD_enable_pin; works !!	  
	//GPIO_PORTB_AHB_DATA_R &= 0xFF;
	
	LCD_Enable_Port[LCD_enable_pin] = 0;
	LCD_RW_Port[LCD_RW_pin] = 0;
	LCD_RS_Port[LCD_RS_pin] = 0;
}
void Run_LCD_initialization_Routine(void){
  delay(5000);
  Clear_Screen();
  Command_LCD(0x38);
  delay(50);
  Command_LCD(0x0C);
  delay(50);
}
void Clear_Screen(void){
  delay(5000);
  Command_LCD(0x01); //clear screen
  delay(2000);
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