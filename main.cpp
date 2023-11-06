
#include <Arduino.h> 
#include"ldr.hpp"
void setup(){
  const int jlopin=PA10 ,backpin= PA11 ,leftpin= PA12 ,rightpin=PB15 ;
    pinMode(leftpin,OUTPUT);
    pinMode(rightpin,OUTPUT);
    pinMode(jlopin,OUTPUT);
    pinMode(backpin,OUTPUT);
     digitalWrite(rightpin,LOW);
     digitalWrite(leftpin,LOW);
     digitalWrite(jlopin,LOW);
     digitalWrite(backpin,LOW);
LDR*ldr_1=new LDR(PA0);
if (ldr_1->ldrread()==0) {
  digitalWrite(leftpin,HIGH);
  delay(10000);
  }

}
void loop (){

}
/*
int dmpR=0;
DMP_DATA gyro;
SR* sr_1 = new SR(PA4, PA5);
void setup () {
dmpR =setupMPU6050DMP(35);
}

void loop () {
  GetDMPdata(&gyro);
  
}*/