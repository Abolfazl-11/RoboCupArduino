// includes
#include <Arduino.h>
#include <math.h>
#include "./MPU6050/MPU6050_DMP6.h"
#include "./Movement/Motors.hpp"
#include "./Pixy/Pixy2.h"
#include <LiquidCrystal.h>
#include "./Movement/Moves.hpp"
#include "./HCSR04/HCSR04.hpp"

// Defines
#define MPU_READ_TH 5
#define DEBUG_TIM 200
#define PIXY_READ_TH 50
#define SR_READ_TH 300

#define PIXY_X_MIN 60
#define PIXY_X_MAX 250
#define PIXY_Y_MIN 4
#define PIXY_Y_MAX 154
#define PIXY_X_MID 156
#define PIXY_Y_MID 98

#define SPEED 45 

#define DEBUG

// Variables
uint64_t timerCounter = 0;
int blink = 1;

int dmpR = 0;
DMP_DATA gyro;

// pixy
Pixy2 pixy_t;

typedef struct BallTransform {
    int x;
    int y;
    int r;
    int theta;
    bool detected;
} BallTransform;

BallTransform ballTransform = {0, 0, 0, 0, false};

bool pixy_init = false;

// HCSR04
SR* sr1 = new SR(PA0, PC15);
SR* sr2 = new SR(PB14, PB15);
SR* sr3 = new SR(PB13, PB12);
SR* sr4 = new SR(PA1, PA2);

Position pos = {0, 0};

SR_Controller* srController = new SR_Controller(sr1, sr2, sr3, sr4, &pos);

double s1, s2, s3, s4;
// Debugger
int d4 = PC10, d5 = PC11, d6 = PC12, d7 = PD2, en = PB7, rs = PB6;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int current = 0;
int len = 0;

bool setuped = false;

// Timers
HardwareTimer* tim1 = new HardwareTimer(TIM1);

HardwareTimer* tim2 = new HardwareTimer(TIM2);

HardwareTimer* tim3 = new HardwareTimer(TIM3);

// Motors Declaration
Motor* motor1 = new Motor(tim1, 2, PA8, 0, 1);
Motor* motor2 = new Motor(tim1, 3, PA4, 0, 1);
Motor* motor3 = new Motor(tim2, 4, PA12, 0, 1);
Motor* motor4 = new Motor(tim1, 4, PA6, 0, 1);

Driver* driver = new Driver(motor1, motor2, motor3, motor4);

Moves* moves = new Moves(driver);

Zones zone = NA;

int nd = 0;

typedef enum State {IN, OUT, HALTED} State;

State state = IN;

int outDir = 0;

long long first = -1;

uint16_t interruptCounter = 0;

// Function Declaration
void setupTimers();

void ReadOutDir();

void init_pixy();
uint8_t GetBallPos();

int lcdCounter = 0;

bool left = false;
// Timer interrupt callback function
void Timer_IT_Callback() {
    timerCounter++;

    if (!(timerCounter%500) && setuped) {
        blink = !blink;
        digitalWrite(PC13, (blink ? HIGH : LOW));
    }

    // Reading Pixy every 150ms
    if (!(timerCounter%PIXY_READ_TH) && setuped) {
        GetBallPos();
    }

    // Reading MPU6050 every 5ms
    if (!(timerCounter%MPU_READ_TH) && setuped) {
        getDMPData(&gyro);
        moves->RotateToZero(gyro.yaw);
    }

    // Reading SRs every 200ms
    if (!(timerCounter%SR_READ_TH) && setuped) {
        //lcd.print("SRs");
        //s1 = sr1->readsr();
        s2 = sr2->readsr();
        //s3 = sr3->readsr();
        s4 = sr4->readsr();
        
        moves->sr1 = s4;
        moves->sr2 = s2;
    }
}

bool outinterrupt_flag = false;

// callback of the bottom board interrupt
void outEXTI_Callback() {
	interruptCounter++;
}

void setup(){
    // pin setup
    pinMode(PC13, OUTPUT);

    pinMode(PB2, INPUT_PULLDOWN);

    stm32_interrupt_enable(GPIOB, GPIO_PIN_2, outEXTI_Callback, GPIO_MODE_IT_RISING);

    pinMode(PA7, INPUT);
    pinMode(PB1, INPUT);

    digitalWrite(PC13, HIGH);
    delay(500);
    digitalWrite(PC13, LOW);
    delay(500);

    lcd.begin(16, 2);

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Hello, World! :)");
    delay(1000);
    lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Ahhh SHIT!");
	lcd.setCursor(0, 1);
	lcd.print("here we go again");
    delay(1500);
    lcd.clear();

    setupTimers();

    lcd.print("Pixy ");
    init_pixy();
    delay(500);
    lcd.clear();

    Wire.setSDA(PB11);
    Wire.setSCL(PB10);
    Wire.begin();
    Wire.setClock(400000);
    dmpR = setupMPU6050DMP(35);

    if (dmpR) {
        digitalWrite(PC13, HIGH);
        lcd.setCursor(0, 0);
        lcd.print("gyro init");
        delay(1000);
        lcd.clear();
    }
    digitalWrite(PC13, LOW);
    delay(500);

    setuped = pixy_init && dmpR;
}


void loop() {
    if (interruptCounter > 1) {
        if (state == IN) {
            state = OUT;
        }
        outinterrupt_flag = false;
		interruptCounter = 0;
    }
    switch(state) {
        case IN:
            if (ballTransform.detected) {
                moves->GetBall(ballTransform.r, ballTransform.theta, SPEED, &zone);
            }
            else {
                if (driver->isMoving) driver->Brake();
            }
            moves->RotateToZero(gyro.yaw);

            break;
        case OUT:
            lcd.clear();
            if (driver->isMoving) driver->Brake();
            ReadOutDir();
            lcd.print("OUT ");
			lcd.print(outDir);
            state = HALTED;

            break;
        case HALTED:
            // Front
            if (outDir == 0) {
                driver->gotoPoint(180, SPEED - 15);
            }
            // Back
            else if (outDir == 1) {
                driver->gotoPoint(0, SPEED - 15);
            }
            // Right
            else if (outDir == 2) {
                driver->gotoPoint(90, SPEED - 15);
            }
            // Left
            else if (outDir == 3) {
                driver->gotoPoint(-90, SPEED - 15);
            }
			lcd.setCursor(0, 1);
            lcd.print("GETTING OUT");
            if (first == -1) {
                first = timerCounter;
            }
            if ((timerCounter - first >= 500) || (timerCounter < first)) {
                driver->Brake();
                state = IN;
                lcd.clear();
                first = -1;
            }
    
            break;
    }
}

// Setting up system clock on 48MHz
extern "C" void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

}

void setupTimers() {
    // TIM3 PWM
    tim2->setOverflow(100, MICROSEC_FORMAT);

    tim2->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PA3);

    // Motor 3
    tim2->setCaptureCompare(4, 0, PERCENT_COMPARE_FORMAT);

    tim2->resume();

    // TIM1 PWM
    tim1->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA9);
    tim1->setMode(3, TIMER_OUTPUT_COMPARE_PWM1, PA10);
    tim1->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PA11);

    tim1->setOverflow(100, MICROSEC_FORMAT);
    // Motor 1
    tim1->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
    // Motor 2
    tim1->setCaptureCompare(3, 0, PERCENT_COMPARE_FORMAT);
    // Motor 4
    tim1->setCaptureCompare(4, 0, PERCENT_COMPARE_FORMAT);

    tim1->resume();

    // IT Timer
    tim3->setOverflow(1000, MICROSEC_FORMAT);
    tim3->attachInterrupt(Timer_IT_Callback);
    tim3->resume();
}

void ReadOutDir() {
    outDir = 0;
    outDir += 2 * !digitalRead(PA7);
    outDir += !digitalRead(PB1);
}

void init_pixy() {
    pixy_t.init();
    
    lcd.print(pixy_t.getVersion());
    if (pixy_t.getVersion() == 16) pixy_init = true;
}

uint8_t GetBallPos() {
    //lcd.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("PixyStart");
    int8_t r = pixy_t.ccc.getBlocks(false, CCC_SIG_ALL, 1);

    ballTransform.detected = false;
    if (pixy_t.ccc.numBlocks > 0) {
        for (int i = 0; i < pixy_t.ccc.numBlocks; i++) {
            Block b = pixy_t.ccc.blocks[i];

            if (b.m_x < PIXY_X_MIN || b.m_x > PIXY_X_MAX || b.m_y < PIXY_Y_MIN || b.m_y > PIXY_Y_MAX) {
                continue;
            }

            ballTransform.detected = true;

            ballTransform.x = b.m_x - PIXY_X_MID;
            ballTransform.y = -(b.m_y - PIXY_Y_MID);
            ballTransform.r = sqrt(ballTransform.x * ballTransform.x + ballTransform.y * ballTransform.y);
            if (ballTransform.x >= 0) {
                ballTransform.theta = -(atan((double)ballTransform.y / ballTransform.x) * RAD_TO_DEG - 90);
            }
            else {
                ballTransform.theta = -((atan((double)ballTransform.y / ballTransform.x) + PI) * RAD_TO_DEG - 90);
            }
        }
    }
    else {
        nd++;
    }

    if (r == PIXY_RESULT_ERROR) {
        lcd.setCursor(0, 0);
        lcd.print("Error");
        ballTransform.detected = false;
		return -1;
    }
    else if (r == PIXY_RESULT_BUSY) {
        lcd.setCursor(0, 0);
        lcd.print("Busy");
        ballTransform.detected = false;
    }
    else if (r > 0 && ballTransform.detected) {
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print("r: ");
		lcd.print(ballTransform.r);
        lcd.print(" t: ");
        lcd.print(ballTransform.theta);
        lcd.setCursor(0, 1);
        lcd.print(zone);
        nd = 0;
    }
    else {
        // Ball finding
        if (nd > 200) {
            ballTransform.detected = false;
        }
        nd++;
        //lcd.clear();
    }
    
    //lcd.setCursor(0, 1);
    //lcd.print("PixyEnded");

    return r;
}
