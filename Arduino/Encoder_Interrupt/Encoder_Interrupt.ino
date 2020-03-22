#define ENC_COUNTS 1200 //Encoder counts per rotation = gear_ratio*counts_per_motor_rev

const int motorPin = 9;
const int encoderPin = 2;
const int directionPin = 3;

volatile long encoderValue = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
int speed = 0;

int rpm = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; // set pwm to 3921.16 Hz
  pinMode(motorPin, OUTPUT);
  EncoderInit();//Initialize the module
  analogWrite(motorPin, 0);
  Serial.begin(9600);
  previousMillis = millis();
  //  while (! Serial);
  //Serial.print("Speed 0 to 255");
}

void loop() {
  // put your main code here, to run repeatedly:
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
    rpm = (float)(encoderValue * 60 / ENC_COUNTS);

    // Only update display when there have readings
    if ( rpm > 0) {


      Serial.print(encoderValue);
      Serial.print(" pulse / ");
      Serial.print(ENC_COUNTS);
      Serial.print(" pulse per rotation x 60 seconds = ");
      Serial.print(rpm);
      Serial.println(" RPM");
    }

    encoderValue = 0;
  }
}

void EncoderInit()
{
  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(encoderPin), updateEncoder, RISING);
}

void updateEncoder()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue++;
}
