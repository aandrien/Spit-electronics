int motorPin = 9;
int sensorPinAnalog = A0;
int sensorPinDig = 2;
int sensorPinBrake = 3;
int sensorValue = 0;
int speed = 0;
int senPinLastState;
int counter = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(sensorPinBrake, OUTPUT);
  digitalWrite(sensorPinBrake, LOW);
  TCCR1B = (TCCR1B & 0b11111000) | 0x01; // set pwm to 3921.16 Hz
  pinMode(motorPin, OUTPUT);
  pinMode(sensorPinDig, INPUT);
  analogWrite(motorPin, 0);
  Serial.begin(9600);
  while (! Serial);
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
  //  sensorValue = analogRead(sensorPin);
  sensorValue = digitalRead(sensorPinDig);
  if (senPinLastState != sensorValue) {
    counter ++;
  }
  //  delay(1);
  //  Serial.print(speed);
  //  Serial.print("\n");
  //  Serial.print("SensorValue: ");
  //  Serial.print(sensorValue);
  //  Serial.print("\n");
//  Serial.println(sensorValue);
//  Serial.print("\n");
  Serial.println(counter);
  Serial.print("\n");
  senPinLastState = sensorValue;
}
