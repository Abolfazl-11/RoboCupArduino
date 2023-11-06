#include<Arduino.h>

class LDR{
    public:
        LDR(int ldrpin);
        int ldrread();
        int ldrpin,ldrcalib;
};