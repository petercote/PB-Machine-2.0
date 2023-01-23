#include <AccelStepper.h>

const byte distancePin = A2;
const byte speedPin = A1;
const byte stepPin = 3;
const byte dirPin = 2;
int oscSignal = 5;
int oscSTATUS;
//const byte enablePin = 8;  // for CNC shield

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

int status;

void setup() {
  //pinMode(enablePin, OUTPUT); // for CNC shield
  //digitalWrite(enablePin, LOW); // enable steppers
  // Change these to suit your stepper if you want
  stepper.setMaxSpeed(2200);
  stepper.setAcceleration(2500);
  Serial.begin(9600);
  pinMode(oscSignal, INPUT_PULLDOWN);
  pinMode(9, INPUT_PULLDOWN);
}

void loop() {
  status = digitalRead(9);
  Serial.println(status);
  if (status == HIGH) {

    static int sign = 1;

    int speed = analogRead(speedPin);
    int distance = analogRead(distancePin);
    int range = map(distance, 0, 1023, 100, 800);

    stepper.setMaxSpeed(speed);

    // If at the end of travel go to the other end
    if (stepper.run() == LOW)  // if the stepper has stopped
    {
      sign = sign * -1;  // reverse sign
      stepper.move(range * sign);
    }
  }
  if (status == LOW) {
    stepper.stop();  // Stop as fast as possible: sets new target
  }
}
