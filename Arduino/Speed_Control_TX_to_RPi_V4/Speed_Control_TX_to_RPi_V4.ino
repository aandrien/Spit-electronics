#include <movingAvg.h>
#include "MovingAverageFloat.h"
#include "SerialTransfer.h"

SerialTransfer myTransfer;

// Frequency modes for TIMER4
#define PWM187k 1  // 187500 Hz
#define PWM94k 2   //  93750 Hz
#define PWM47k 3   //  46875 Hz
#define PWM23k 4   //  23437 Hz
#define PWM12k 5   //  11719 Hz
#define PWM6k 6    //   5859 Hz
#define PWM3k 7    //   2930 Hz

// Direct PWM change variables
#define PWM6 OCR4D

// Remote Pins
#define controlPin A0
#define stopPin 2
#define startPin 4
#define directionPin 0

// Configure the PWM clock
// The argument is one of the 7 previously defined modes
void pwm613configure(int mode) {
  // TCCR4A configuration
  TCCR4A = 0;

  // TCCR4B configuration
  TCCR4B = mode;

  // TCCR4C configuration
  TCCR4C = 0;

  // TCCR4D configuration
  TCCR4D = 0;

  // TCCR4D configuration
  TCCR4D = 0;

  // PLL Configuration
  // Use 96MHz / 2 = 48MHz
  PLLFRQ = (PLLFRQ & 0xCF) | 0x30;
  // PLLFRQ=(PLLFRQ&0xCF)|0x10; // Will double all frequencies

  // Terminal count for Timer 4 PWM
  OCR4C = 255;
}

// Set PWM to D6 (Timer4 D)
// Argument is PWM between 0 and 255
void pwmSet6(int value) {
  OCR4D = value;   // Set PWM value
  DDRD |= 1 << 7;  // Set Output Mode D7
  TCCR4C |= 0x09;  // Activate channel D
}

void config1kHzLoop(float interrupt_freq) {
  //--------------------------timer setup
  cli();  // disable all interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   //initialize counter value to 0
  // set timer count for 100 Hz increments
  //  OCR1A = 624;// = (16*10^6) / (100*256) - 1 // 100 Hz
  //  OCR1A = 62499;// = (16*10^6) / (1*256) - 1 // 1 Hz
  OCR1A = (16000000) / (interrupt_freq * 1024) - 1;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();  // enable all interrupts
  //--------------------------timer setup
}

/*************** ADDITIONAL DEFINITIONS ******************/

// Macro to converts from duty (0..100) to PWM (0..255)
#define DUTY2PWM(x) ((255 * (x)) / 100)

/**********************************************************/

// Pin definitions
const int encoderPin = 7;

// Variables
float vel_ref = 0.0;  // PWM pulse to motor, must be in range 0 to 255

float vel_ref_pi = 0.0;  // PWM pulse to motor, must be in range 0 to 255
float vel_ref_pot = 0.0;  // PWM pulse to motor, must be in range 0 to 255

float u = 0.0;
float ctrl = 0.0;
float error = 0.0;
float control_vel = 0.0;
float error_integral = 0.0;

float P = 2.0;
float I_action = 10.0;
// float I_action = 0.0;

int direction = 1;
float max_pwm = 100;
boolean motor_start = false;

volatile unsigned long encoderValue = 0;
volatile float rpm = 0.0;  // rpm of motor
volatile float vel = 0.0;
volatile unsigned long prevT = 0;
volatile unsigned long prevTmain = 0;

const float interrupt_freq = 50.0;  // Interrupt frequency in Hz

#define GEAR_RATIO 200.0  //Gear ratio
#define MOTOR_COUNTS 6    // Counts per motor revolution

const int enc_counts = GEAR_RATIO * MOTOR_COUNTS;  //Encoder counts per rotation at outputshaft = gear_ratio*counts_per_motor_rev

movingAvg mySensor(10);
MovingAverageFloat<5> velMovingAvg;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 500;  // interval at which to blink (milliseconds)

// Struct for sending Data
struct __attribute__((packed)) STRUCT_send {
  float vel_ref;
} testStruct;

uint8_t start_stop_RPi  = 0;
uint8_t start_stop_RPi_prev = 0;
bool started_from_RPi = false;


float vel_ref_receive = 0.0;

void setup() {
  // put your setup code here, to run once:

  //--------------------------timer setup
  //  config1kHzLoop(interrupt_freq);
  pwm613configure(PWM23k);
  pwmSet6(0);

  //--------------------------timer setup

  //-------------------------- Define pin modes
  //   Attach interrupt for the encoder Pin on each rising signal
  attachInterrupt(digitalPinToInterrupt(encoderPin), updateEncoder, RISING);

  //  // Attach interrupt for the start Pin on each rising signal
  // attachInterrupt(digitalPinToInterrupt(startPin), updateStart, RISING);
  //
  //  // Attach interrupt for the stop Pin on each rising signal
  //  attachInterrupt(digitalPinToInterrupt(stopPin), updateStop, RISING);
  //-------------------------- Define pin modes

  pinMode(directionPin, OUTPUT);    // sets the digital pin 13 as output
  digitalWrite(directionPin, LOW);  // sets the digital pin 13 off

  // Start Moving Average
  mySensor.begin();

  // Open serial port
  Serial.begin(115200);
  myTransfer.begin(Serial);

  testStruct.vel_ref = 0.0;

  // Setup Complete
}

void loop() {
  // put your main code here, to run repeatedly:

  if (myTransfer.available()) {
    // use this variable to keep track of how many
    // bytes we've processed from the receive buffer
    uint16_t recSize = 0;
    recSize = myTransfer.rxObj(vel_ref_receive, recSize);
    recSize = myTransfer.rxObj(start_stop_RPi, recSize);

    vel_ref_pi = vel_ref_receive;
  }

  bool start_from_RPi = false;
  if(start_stop_RPi && !start_stop_RPi_prev )
  {
    // Start requested from RPi
    start_from_RPi = true;
  }

  bool stop_from_RPi = false;
  if(!start_stop_RPi && start_stop_RPi_prev )
  {
    // Stop requested from RPi
    stop_from_RPi = true;
  }

  start_stop_RPi_prev = start_stop_RPi;

  float potValue = analogRead(controlPin);
  float potMovingAvg = mySensor.reading(potValue);
  int stopCom = digitalRead(stopPin);
  int startCom = digitalRead(startPin);

  if ((startCom || start_from_RPi) && !motor_start) {
    if (!stopCom) {
      motor_start = true;
      error_integral = 0.0;
      if(start_from_RPi)
      {
        started_from_RPi = true;
      }
      else
      {
        started_from_RPi = false;
      }
    }
    startCom = LOW;
  }
  if (stopCom || stop_from_RPi) {
    motor_start = false;
    stopCom = LOW;
    started_from_RPi = false;
  }

  vel_ref_pot = max_pwm - ((potMovingAvg * 100.0) / 1023.0);

  if (started_from_RPi)
  {
    vel_ref = vel_ref_pi;
  }
  else
  {
    vel_ref = vel_ref_pot;
  }


  // dt calculation
  unsigned long currTmain = millis();
  float deltaTmain = ((float)(currTmain - prevTmain) / 1000.0);


  if (vel_ref < 0.1) {
    error_integral = 0.0;
    control_vel = 0.0;
    velMovingAvg.reset();
    vel = 0.0;
    u = 0.0;
  }

  if (motor_start && vel_ref > 0.1) {
    error = vel_ref - control_vel;
    error_integral += error * deltaTmain;
    error_integral = max(min(error_integral, 10), -10);
    ctrl = P * error + I_action * error_integral + 10.0;
    u = max(min(100, ctrl), 0);
    PWM6 = DUTY2PWM(u);
    control_vel = velMovingAvg.add(vel / 3.0);
  } else {
    u = 0.0;
    PWM6 = DUTY2PWM(0.0);
    error_integral = 0.0;
    control_vel = 0.0;
    velMovingAvg.reset();
    vel = 0.0;
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // use this variable to keep track of how many
    // bytes we're stuffing in the transmit buffer
    uint16_t sendSize = 0;

    unsigned long encoderSend = 0;
    encoderSend = encoderValue;

    ///////////////////////////////////////// Stuff buffer with struct
    sendSize = myTransfer.txObj(control_vel, sendSize);
    sendSize = myTransfer.txObj(encoderSend, sendSize);

    ///////////////////////////////////////// Send buffer
    myTransfer.sendData(sendSize);
  }



  delay(1 / 500);
  prevTmain = currTmain;
}

// UPDATE_ENCODER FUNCTION
void updateEncoder() {
  // Add encoderValue by 1, each time it detects rising signal from light detector
  encoderValue++;

  // Velocity computation
  unsigned long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  if ((1.0 / deltaT) < 5000.0) {
    vel = 1.0 / deltaT;
  }
  prevT = currT;
}

ISR(TIMER1_COMPA_vect) {  //Interrupt at freq of 1kHz to measure reed switch

  // Revolutions per minute (RPM) =
  // (total encoder pulse in 1s / motor encoder output) x 60s
  rpm = float((interrupt_freq) / enc_counts);
  //  rpm = float((60.0 * interrupt_freq * encoderValue) / enc_counts);
  // encoderValue = 0;
}
