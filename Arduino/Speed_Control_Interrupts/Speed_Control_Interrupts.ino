#define GEAR_RATIO 200.0 //Gear ratio
#define MOTOR_COUNTS 6 // Counts per motor revolution

const int enc_counts = GEAR_RATIO * MOTOR_COUNTS; //Encoder counts per rotation at outputshaft = gear_ratio*counts_per_motor_rev
const int motorPin = 11; // Pin 11 is Timer0A, Single slope FAST PWM at 976 Hz
const int encoderPin = 2;
const int directionPin = 3;
const int rpm_max = 50; // Maximum rpm of output shaft
const float interrupt_freq = 10.0; // Interrupt frequency in Hz
const float kp_speed = 2; // P-gain for speed control loop
const float kd_speed = 0.05; // P-gain for speed control loop
const float ki_speed = 1; // P-gain for speed control loop

volatile long encoderValue = 0;
volatile byte state = LOW;
volatile float rpm = 0.0;

float e_speed = 0; // Speed error = speed_set-rpm
float e_speed_pre = 0; // Previous speed error
float diff_e_speed = 0; // Difference between current and previous error
float e_speed_sum = 0; // Integral of speed error
float pwm_pulse = 0; // PWM pulse to motor, must be in range 0 to 255

int speed_set = 0;
int start_bias = 20; // Starting PWM bias for motor, motor will not turn below this

boolean motor_start = false;

void setup() {
  // put your setup code here, to run once:

  //-------------------------- Define pin modes
  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);
  //-------------------------- Define pin modes

  //--------------------------timer setup
  cli();           // disable all interrupts
  //set timer1 interrupt at 1kHz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set timer count for 100 Hz increments
  //  OCR1A = 624;// = (16*10^6) / (100*256) - 1 // 100 Hz
  //  OCR1A = 62499;// = (16*10^6) / (1*256) - 1 // 1 Hz
  OCR1A = (16000000) / (interrupt_freq * 256) - 1;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 bit for 256 prescaler
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();             // enable all interrupts
  //--------------------------timer setup

  // Attach interrupt for the encoder Pin on each rising signal
  attachInterrupt(digitalPinToInterrupt(encoderPin), updateEncoder, RISING);

  // Open serial port
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  //  while (! Serial);
  //Serial.print("Speed 0 to 255");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    speed_set = Serial.parseInt();
    if (speed_set <= 0) speed_set = 0; // Limit speeds setpoint above 0
    if (speed_set >= rpm_max) speed_set = rpm_max; // Limit speeds setpoint below rpm_max
  }
  if (speed_set >= 1 && speed_set <= rpm_max) {
    motor_start = true;
  }
  else {
    motor_start = false;
  }
}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from encoder
  encoderValue++;
}

ISR(TIMER1_COMPA_vect) {//Interrupt at freq of 1kHz to measure reed switch
  // Revolutions per minute (RPM) =
  // (total encoder pulse in 1s / motor encoder output) x 60s
  rpm = float((60.0 * interrupt_freq * encoderValue) / enc_counts);
  encoderValue = 0;

  if (motor_start) {
    e_speed = speed_set - rpm;
    diff_e_speed = (e_speed - e_speed_pre);
    pwm_pulse = e_speed * kp_speed + diff_e_speed * kd_speed + e_speed_sum * ki_speed + start_bias;
    e_speed_pre = e_speed;  //save last (previous) error
    e_speed_sum += e_speed; //sum of error
    if (e_speed_sum > 4000) e_speed_sum = 200;
    if (e_speed_sum < -4000) e_speed_sum = -200;
    if (pwm_pulse >= 255) pwm_pulse = 254;
    if (pwm_pulse <= 0) pwm_pulse = 0;
  }
  else {
    e_speed = 0;
    e_speed_pre = 0;
    e_speed_sum = 0;
    pwm_pulse = 0;
  }

  // Write PWM to motor
  if (pwm_pulse <255 & pwm_pulse >0) {
    analogWrite(motorPin, pwm_pulse); //set motor speed
  }
  else {
    if (pwm_pulse > 255) {
      analogWrite(motorPin, 255);
    }
    else {
      analogWrite(motorPin, 0);
    }
  }

  //print out speed
  if (Serial.available() <= 0) {
    Serial.print("RPM: ");
    Serial.println(rpm);         //Print speed (rpm) value to Visual Studio
    Serial.print("\n");

    Serial.print("PWM pulse: ");
    Serial.println(pwm_pulse);
    Serial.print("\n");

    //    Serial.print("Speed error: ");
    //    Serial.println(e_speed);
    //    Serial.print("\n");

    //    Serial.print("Diff speed error: ");
    //    Serial.println(diff_e_speed);
    //    Serial.print("\n");
  }

}
