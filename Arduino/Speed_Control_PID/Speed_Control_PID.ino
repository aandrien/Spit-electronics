#define ENC_COUNTS 1200 //Encoder counts per rotation = gear_ratio*counts_per_motor_rev

const int motorPin = 11; // Pin 11 is Timer0A, Single slope FAST PWM at 976 Hz
const int encoderPin = 2;
const int directionPin = 3;

volatile long encoderValue = 0;
volatile byte state = LOW;

int interval = 500; //Update RPM time in ms
long previousMillis = 0;
long currentMillis = 0;
int speed = 0;

int rpm = 0;
int timer1_counter; //for timer

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
  OCR1A = 624;// = (16*10^6) / (100*256) - 1 // 100 Hz
//  OCR1A = 62499;// = (16*10^6) / (1*256) - 1 // 1 Hz
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 bit for 256 prescaler
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();             // enable all interrupts
  //--------------------------timer setup

  previousMillis = millis();

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
  //  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    speed = Serial.parseInt();
  }
  if (speed >= 1 && speed <= 255) {
    analogWrite(motorPin, speed);
  }

  // Update RPM value on every second
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

    // Revolutions per minute (RPM) =
    // (total encoder pulse in 1s / motor encoder output) x 60s
    rpm = (float)(encoderValue * 60 * 1000 / interval / ENC_COUNTS);

    // Only update display when there have readings
    if ( rpm > 0) {


      //      Serial.print(encoderValue);
      //      Serial.print(" pulse / ");
      //      Serial.print(ENC_COUNTS);
      //      Serial.print(" pulse per rotation x 60 seconds = ");
      //      Serial.println(rpm);
      //      Serial.print(" RPM");
      Serial.println(rpm);
      //      Serial.print(" ");
    }

    encoderValue = 0;

  }

}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
}

ISR(TIMER1_COMPA_vect) {//Interrupt at freq of 1kHz to measure reed switch
  
}
