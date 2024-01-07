#include <Arduino.h>

#define FIELD_WIDTH 175
#define FIELD_LENGHT 235

typedef struct Position {
    double x;
    double y;
} position;

class SR {
    public:
        SR(int trigpin, int echopin);
        double readsr();
    private :
        int trigpin;
        int echopin;
};

class SR_Controller{
    public:
        SR_Controller(SR* sr1 , SR* sr2 ,SR* sr3, SR* sr4, Position* position);

        void ReadAllSRs();

        Position* position;
    private:
        SR* sr1;
        SR* sr2;
        SR* sr3;
        SR* sr4;
};
