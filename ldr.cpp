#include<Arduino.h>
#include"ldr.hpp"
LDR::LDR(int ldrpin){
   this->ldrcalib=ldrcalib;
   this->ldrpin=ldrpin;
   ldrcalib=0;
   for(int i =0 ; i<30;i++)      ldrcalib+=analogRead(ldrpin);
   ldrcalib=ldrcalib/30;
}
int LDR::ldrread(){
       int line,i;
       line=analogRead(ldrpin);
       if (line<ldrcalib) i = 0 ;
       else i = 1 ;
       return(int)i;
}