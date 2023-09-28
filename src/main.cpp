// includes
#include <Arduino.h>
#include "./MPU6050/MPU6050_DMP6.h"
#include "./Movement/Motors.hpp"

// Defines
#define MPU_READ_TH 200

// Variables
long long timerCounter = 0;
int blink = 1;

int dmpR = 0;
DMP_DATA gyro;

// Timers
HardwareTimer* tim1 = new HardwareTimer(TIM1);

HardwareTimer* tim2 = new HardwareTimer(TIM2);

HardwareTimer* tim3 = new HardwareTimer(TIM3);

// Motors Declaration
Motor* motor1 = new Motor(tim3, 1, PA3);
Motor* motor2 = new Motor(tim1, 4, PA12);
Motor* motor3 = new Motor(tim1, 2, PA4);
Motor* motor4 = new Motor(tim1, 1, PA10);

Driver* driver = new Driver(motor1, motor2, motor3, motor4);

// Function Declaration
void setupTimers();

// Timer interrupt callback function
void Timer_IT_Callback() {
    timerCounter++;

    if (!(timerCounter%500) && dmpR) {
        blink = !blink;
        digitalWrite(PC13, (blink ? HIGH : LOW));
    }

    if (!(timerCounter%MPU_READ_TH) && dmpR) {
        getDMPData(&gyro);
    }
}

void setup(){
    // pin setup
    pinMode(PC13, OUTPUT);

    digitalWrite(PC13, HIGH);
    delay(500);
    digitalWrite(PC13, LOW);
    delay(500);

    setupTimers();

    dmpR = setupMPU6050DMP(25);

    if (dmpR) {
        digitalWrite(PC13, HIGH);
        delay(1000);
    }
    digitalWrite(PC13, LOW);
    delay(500);
}

void loop() {

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
    // IT Timer
    tim2->setOverflow(1000, MICROSEC_FORMAT);
    tim2->attachInterrupt(Timer_IT_Callback);
    tim2->resume();

    // TIM1 PWM
    tim1->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA8);
    tim1->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA9);
    tim1->setMode(4, TIMER_OUTPUT_COMPARE_PWM1, PA11);

    tim1->setOverflow(10000, MICROSEC_FORMAT);
    // Motor 4
    tim1->setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
    // Motor 3
    tim1->setCaptureCompare(2, 0, PERCENT_COMPARE_FORMAT);
    // Motor 2
    tim1->setCaptureCompare(4, 0, PERCENT_COMPARE_FORMAT);

    tim1->resume();

    // TIM3 PWM
    tim3->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA6);

    tim3->setOverflow(10000, MICROSEC_FORMAT);
    // Motor 1
    tim3->setCaptureCompare(1, 0, PERCENT_COMPARE_FORMAT);
    
    tim3->resume();
}
