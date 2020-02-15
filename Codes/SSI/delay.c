#include "delay.h"

void delay_s(int iter_s){
  int volatile counter = 0;
  
  while(counter < iter_s*1367000){
    ++counter;
  }
}

void delay_ms(int n){
   int i,j;
   for(i=0;i<n;i++)
   for(j=0;j<3180;j++)
   {}
}
void delay_us(int n){
   int i,j;
   for(i=0;i<n;i++)
   for(j=0;j<3;j++)
   {}
}

void delay(int iter){
  int volatile counter = 0;
  
  while(counter < iter){
    ++counter;
  }
}