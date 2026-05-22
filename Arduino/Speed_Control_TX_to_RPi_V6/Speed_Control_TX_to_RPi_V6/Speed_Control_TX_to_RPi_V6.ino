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

int direction = 1;
float max_pwm = 100;
boolean motor_start = false;

volatile long encoderValue = 0;          // signed: increments on forward pulses, decrements on reverse pulses
volatile uint8_t current_direction = 0;  // mirror of the pin that's actually driving the H-bridge; ISR reads this to decide +/-
volatile float vel = 0.0;                // pulse-rate magnitude (always >= 0) — direction is tracked separately
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
const long serial_tx_interval_ms = 50;  // interval at which to blink (milliseconds)

// Struct for sending Data
struct __attribute__((packed)) STRUCT_send {
  float vel_ref;
} testStruct;

uint8_t start_stop_RPi  = 0;
uint8_t start_stop_RPi_prev = 0;
bool started_from_RPi = false;

uint8_t direction_RPi = 0;     // requested direction from Pi (velocity-mode source of truth)
uint8_t direction_target = 0;  // desired direction the latch is trying to apply
                               //   velocity mode: tracks direction_RPi
                               //   position mode: sign of (pos_ref - encoderValue)
                               // current_direction (already declared) is what's actually driving the H-bridge.

// Cascaded position-control state. control_mode_RPi switches between:
//   0 = velocity mode (existing behavior: vel_ref_pi follows the Pi's vel_ref_receive)
//   1 = position mode: outer P controller computes a signed velocity setpoint from
//       pos_ref_counts - encoderValue; magnitude feeds the existing velocity PID
//       via vel_ref_pi, sign drives direction_target through the existing latch.
uint8_t control_mode_RPi = 0;
long pos_ref_counts = 0;
float pos_P_gain = 0.05;
float pos_I_action = 0.0;    // integral gain on the outer position loop
float pos_D_gain = 0.0;      // derivative gain — applied to measured signed velocity (not error derivative)
float pos_max_vel = 30.0;
float pos_max_accel = 25.0;  // vel-units / second slew limit on the position-loop output

// Slew-rate state for position mode. vel_mag_smooth is the rate-limited magnitude
// of the position-loop's velocity output that actually drives the inner velocity
// PID. pos_error_integral accumulates pos_error * dt for the I term, with
// anti-windup applied to bound its contribution to ±pos_max_vel. pos_vel_filt is
// an EWMA-filtered signed velocity used for the D term (we filter because the
// raw 1/dt from each encoder pulse is noisy). All three reset to 0 the moment
// we enter position mode so it starts cleanly.
float vel_mag_smooth = 0.0;
float pos_error_integral = 0.0;
float pos_vel_filt = 0.0;
const float POS_VEL_FILT_ALPHA = 0.2;  // EWMA: new = alpha * latest + (1-alpha) * old
unsigned long lastPosLoopTime = 0;
uint8_t control_mode_prev = 0;

// A direction flip is only applied to the H-bridge when BOTH of the following
// are true:
//   - PWM duty is effectively zero (no power is being delivered)
//   - No encoder pulse for DIRECTION_FLIP_QUIET_MS (the spit has actually coasted
//     to rest — control_vel can lie because vel is stuck at its last value when
//     pulses stop arriving)
// At 305 pulses/rev, 200 ms of silence bounds motion below ~1/(305*0.2) rev/sec
// ≈ 6 deg/sec, which is effectively stationary.
const unsigned long DIRECTION_FLIP_QUIET_MS = 200;
volatile unsigned long lastPulseTime = 0;

float vel_ref_receive = 0.0;
float P = 2.0;
float I_action = 3.5;
float feed_forward = 0.0;

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
    recSize = myTransfer.rxObj(P, recSize);
    recSize = myTransfer.rxObj(I_action, recSize);
    recSize = myTransfer.rxObj(feed_forward, recSize);
    recSize = myTransfer.rxObj(start_stop_RPi, recSize);
    recSize = myTransfer.rxObj(direction_RPi, recSize);
    recSize = myTransfer.rxObj(control_mode_RPi, recSize);
    recSize = myTransfer.rxObj(pos_ref_counts, recSize);
    recSize = myTransfer.rxObj(pos_P_gain, recSize);
    recSize = myTransfer.rxObj(pos_max_vel, recSize);
    recSize = myTransfer.rxObj(pos_max_accel, recSize);
    recSize = myTransfer.rxObj(pos_I_action, recSize);
    recSize = myTransfer.rxObj(pos_D_gain, recSize);
  }

  // Mode-dependent computation of direction_target and vel_ref_pi.
  // The existing velocity PID and direction-flip latch downstream are
  // identical for both modes — they just consume different inputs.
  if (control_mode_RPi == 1) {
    // Position mode: outer PID controller (P + I with anti-windup + D on
    // measured velocity), fed through a slew-rate-limited magnitude to the
    // inner velocity PID.
    unsigned long now_pos_ms = millis();
    if (control_mode_prev == 0) {
      vel_mag_smooth = 0.0;
      pos_error_integral = 0.0;
      pos_vel_filt = 0.0;
      lastPosLoopTime = now_pos_ms;
    }
    float dt_pos = (now_pos_ms - lastPosLoopTime) / 1000.0;
    lastPosLoopTime = now_pos_ms;

    long pos_error = pos_ref_counts - encoderValue;
    pos_error_integral += (float)pos_error * dt_pos;

    // Anti-windup: bound the integral so its contribution is at most
    // ±pos_max_vel. With I disabled (gain ~0), just zero it — no point
    // accumulating something we won't use.
    if (pos_I_action > 1e-6) {
      float max_int = pos_max_vel / pos_I_action;
      if (pos_error_integral >  max_int) pos_error_integral =  max_int;
      if (pos_error_integral < -max_int) pos_error_integral = -max_int;
    } else {
      pos_error_integral = 0.0;
    }

    // D term uses the *measured* signed velocity (velocity-feedback form of D)
    // rather than d(error)/dt. Two reasons:
    //   1. d(error)/dt amplifies single-pulse encoder timing jitter.
    //   2. We already have a good velocity estimate from the ISR's vel.
    // We can't use control_vel directly: it gets force-zeroed when vel_ref<0.1
    // (exactly the regime where the loop tends to oscillate around the setpoint).
    // So we build our own: raw ISR vel/3 (matching control_vel scaling), zeroed
    // honestly when no pulses have fired recently, signed by current_direction,
    // EWMA-smoothed to take the edge off pulse jitter.
    unsigned long since_pulse_d = now_pos_ms - lastPulseTime;
    float vel_mag_for_d = (since_pulse_d > DIRECTION_FLIP_QUIET_MS) ? 0.0 : (vel / 3.0);
    float vel_signed_meas = vel_mag_for_d * (current_direction == 0 ? 1.0 : -1.0);
    pos_vel_filt = POS_VEL_FILT_ALPHA * vel_signed_meas
                   + (1.0 - POS_VEL_FILT_ALPHA) * pos_vel_filt;

    float vel_signed = (float)pos_error * pos_P_gain
                       + pos_error_integral * pos_I_action
                       - pos_D_gain * pos_vel_filt;
    if (vel_signed >  pos_max_vel) vel_signed =  pos_max_vel;
    if (vel_signed < -pos_max_vel) vel_signed = -pos_max_vel;

    direction_target = (vel_signed >= 0) ? 0 : 1;

    // Slew-rate-limit the magnitude fed to the inner velocity PID. Smooth
    // acceleration on big setpoint moves, smooth deceleration into the target,
    // and no rapid switching when vel_signed sign-flips on encoder jitter near
    // the setpoint. The slew target is 0 whenever the direction pin doesn't
    // match yet, so the spit decelerates cleanly into a turnaround.
    float slew_target = (current_direction == direction_target) ? fabs(vel_signed) : 0.0;
    float max_step = pos_max_accel * dt_pos;
    float diff = slew_target - vel_mag_smooth;
    if (fabs(diff) <= max_step) {
      vel_mag_smooth = slew_target;
    } else if (diff > 0) {
      vel_mag_smooth += max_step;
    } else {
      vel_mag_smooth -= max_step;
    }
    vel_ref_pi = vel_mag_smooth;
  } else {
    // Velocity mode: pass through the Pi's vel_ref and direction unchanged.
    direction_target = direction_RPi;
    vel_ref_pi = vel_ref_receive;
    vel_mag_smooth = 0.0;      // reset for clean next entry to position mode
    pos_error_integral = 0.0;
    pos_vel_filt = 0.0;
  }
  control_mode_prev = control_mode_RPi;

  // Apply latched direction only when the spit is actually coasted to rest
  // AND we are not delivering PWM. Both checks together guarantee no hard
  // reversal under load: PWM=0 means the H-bridge isn't driving anything,
  // and the encoder-silence check confirms the motor is physically still
  // (control_vel cannot be trusted because vel is frozen at its last value
  // when pulses stop, and it is force-zeroed when vel_ref < 0.1).
  unsigned long sinceLastPulse = millis() - lastPulseTime;
  if (u < 0.5 && sinceLastPulse > DIRECTION_FLIP_QUIET_MS) {
    // Update current_direction BEFORE the pin write so the ISR can't briefly
    // count a stray pulse in the old direction. (No pulses should fire during
    // the latch wait anyway, but cheap defense.)
    // direction_target is the mode-aware desired direction: from Pi in
    // velocity mode, from sign of position error in position mode.
    current_direction = direction_target;
    digitalWrite(directionPin, direction_target ? HIGH : LOW);
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

  // If motor started and velocity reference is higher than 0.1
  if (motor_start && vel_ref > 0.1) {
    error = vel_ref - control_vel; // Calculate error
    error_integral += error * deltaTmain; // Calculate error integral
    error_integral = max(min(error_integral, 10), -10); // Saturate integral to -10, 10
    ctrl = P * error + I_action * error_integral + feed_forward; 
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

  if (currentMillis - previousMillis >= serial_tx_interval_ms) {
    // save the last time data was sent
    previousMillis = currentMillis;

    // use this variable to keep track of how many
    // bytes we're stuffing in the transmit buffer
    uint16_t sendSize = 0;

    long encoderSend = encoderValue;  // signed: now reflects net direction-aware position
    // encoderSend = encoderValue;

    ///////////////////////////////////////// Stuff buffer with struct
    sendSize = myTransfer.txObj(control_vel, sendSize);
    sendSize = myTransfer.txObj(encoderSend, sendSize);

    ///////////////////////////////////////// Send buffer
    myTransfer.sendData(sendSize);
  }

  delay(1 / 500); // Delay for main script 'sample frequency'
  prevTmain = currTmain;
}

// UPDATE_ENCODER FUNCTION
void updateEncoder() {
  // Single-channel encoder — can't tell us direction from the hardware itself.
  // Use the latched current_direction (set by the main loop only when the
  // motor is at rest, so it always matches physical motion) to count signed.
  if (current_direction == 0) {
    encoderValue++;
  } else {
    encoderValue--;
  }

  // Stamp the time of the last pulse for the direction-flip safety gate.
  lastPulseTime = millis();

  // Velocity computation — pulse-rate magnitude, independent of direction.
  unsigned long currT = micros();
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  if ((1.0 / deltaT) < 5000.0) { // update velocity only if value is reasonable
    vel = 1.0 / deltaT;
  }
  prevT = currT;
}