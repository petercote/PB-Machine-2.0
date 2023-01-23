    //Include AccelStepper Library
    #include <AccelStepper.h>
    #define feedPulse 3                            //pin 3 leads to pul+ terminal  on ball feed stepper driver
    #define feedDir 2                              //pin 2 leads to dir+ terminal on ball feed stepper driver
    AccelStepper stepper1(2, feedPulse, feedDir);  //declares feed motor as stepper1, sets pin sequence order

    //potentiometer pins
    int feedPin = A15;      //ball feed interval potentiometer
    int spinPin = A7;       //ball spin potentiometer
    int velocityPin = A2;   //ball velocity potentiometer
    int oscRangePin = A13;  //oscillation range potentiometer
    int oscSpeedPin = A9;   //oscillation speed potentiometer

    //button pins
    int StartStopPin = 7;  //start/stop button connects to arduino pin 7
    int loftupPin = 5;     //loft up button pin
    int loftdwnPin = 6;    //loft down button pin
    int remote = A8;       //RF Remote to A8 pin on Arduino

    //switch pins
    int oscillationPin = 20;  //Oscillation on/off switch to pin 20 on arduino
    int oscSTATUS;            //digital read of oscillation on/off
    int oscOUTPUT = 51;       // pin 14 on Arduino Mega connects to D5 on arduino Nano RP2040
    int rand2Line = 19;       //Random/2 Line Switch to pin 19 on arduino
    int remoteOnOff = 18;     //remote on/off switch to pin 18 on arduino

    //hardware pins
    int topMotorDir1 = 4;      //top motor controller pin
    int topMotorDir2 = 8;      //top motor controller pin
    int bottomMotorDir1 = 9;   //bottom motor controller pin
    int bottomMotorDir2 = 10;  //bottom motor controller pin
    int loftDir = 12;    // arduino pin 12 leads to dir+ terminal on loft stepper driver
    int loftPulse = 13;  // arduino pin 13 leads to pul+ terminal on loft stepper driver
    int beep = A4;       //warning beep to arduino pin A4

    //varaiables
    int ballSpeed;                   //read value of ball speed potientiometer
    int spinValue;                   //read value of ball spin potentiometer
    int feedRate;                    //read value of feed rate potentiometer
    unsigned long feedSpeed;         //delay value that determines speed of ball feed stepper motor
    unsigned long previousStep = 0;  //time of previous step in millis
    float topMotorSpeed;             //motor speed of top motor
    float bottomMotorSpeed;          //motor speed of bottom motor
    int topSpinMap;
    int backSpinMap;
    float topSpinPercent;
    float backSpinPercent;
    int loftupStatus;     //read value from loft up button
    int loftdwnStatus;    //read value from loft down button
    int StartStopStatus;  //read value from start/stop button
    int loftSpeed = 30;   //delay time between loft stepper pulses
    unsigned long currentMillis;
    int priorMillis;
    unsigned long elapsedMillis;
    int pulseStatus = LOW;
    int debounce = 100;   //delays to allow button to debounce
    int pgmState = 1;     //a read value of 1 means the program is not running
    int buttonNew;        //a read value of 0 means the program is running
    int buttonOld = 1;    //ensures that the program does not run on power up
    int remoteState;      // reads the state of the RF remote switch
    int threshold = 550;  //threshold for IF statement in While loop
    int oscVal;           //read value of oscillation on/off switch
    int loopCounter;    //helps run code after activating RF Remote

    void setup() {
      //potentiometer pin modes here
      pinMode(feedPin, INPUT);
      pinMode(spinPin, INPUT);
      pinMode(velocityPin, INPUT);

      //switch pin modes
      pinMode(oscillationPin, INPUT_PULLUP);

      //button pin modes here
      pinMode(StartStopPin, INPUT_PULLUP);  //start stop button set as input with pullup resistor
      pinMode(loftupPin, INPUT_PULLUP);     //loft up button set as input with pullup resistor
      pinMode(loftdwnPin, INPUT_PULLUP);    //loft down button set as input with pullup resistor
      pinMode(remote, INPUT_PULLUP);        //RF remote pin set as input with pullup resistor

      //hardware signal pin modes here
      pinMode(topMotorDir1, OUTPUT);
      pinMode(topMotorDir2, OUTPUT);
      pinMode(bottomMotorDir1, OUTPUT);
      pinMode(bottomMotorDir2, OUTPUT);
      pinMode(feedDir, OUTPUT);
      pinMode(feedPulse, OUTPUT);
      pinMode(loftDir, OUTPUT);
      pinMode(loftPulse, OUTPUT);
      pinMode(beep, OUTPUT);
      pinMode(oscOUTPUT, OUTPUT);

      stepper1.setMaxSpeed(4300);  //max feed rate speed

      //Start serial monitor
      Serial.begin(115200);
    }

    void loop() {
      Serial.println("MAIN LOOP");
      remoteState = analogRead(remote);  //reads the values from RF remote pin

      //MAIN LOOP BALL VELOCITY
      //sends pwm signal to ball speed dc motor controller
      analogWrite(topMotorDir1, 0); //stops top motor
      analogWrite(bottomMotorDir1, 0); //stops bottom motor
      analogWrite(beep, 0);           //warning beep off
      digitalWrite(oscSTATUS, HIGH);  //turns off oscillation

      //reads button values
      loftupStatus = digitalRead(loftupPin);        //reads the status of loft up button and defines the variable
      loftdwnStatus = digitalRead(loftdwnPin);      //reads the status of the loft down button and defines the variable

      //LOFT UP CONTROL
      //while loft up button is pushed it will execute the below commands in a continuous loop
      while (loftupStatus == LOW) {
        // stepper motor/driver commands to adjust loft up
        digitalWrite(loftDir, HIGH);            //High and Low sets direction of loft stepper motor
        digitalWrite(loftPulse, HIGH);          //moves motor one setp
        delayMicroseconds(loftSpeed);           //time between steps.  Changing this time will increase or decrease speed of motor
        digitalWrite(loftPulse, LOW);           //resets stepper pulse so that the next pulse will move the stepper motor another step
        delayMicroseconds(loftSpeed);           //time between steps.  Changing this time will increase or decrease speed of motor
        loftupStatus = digitalRead(loftupPin);  //reads the status of loft up button and defines the variable
      }
      //LOFT DOWN CONTROL
      //while loft down button is pushed it will execute the below commands in a continuous loop
      while (loftdwnStatus == LOW) {
        // stepper motor/driver commands to adjust loft down
        digitalWrite(loftDir, LOW);               //High and Low sets direction of loft stepper motor
        digitalWrite(loftPulse, HIGH);            //moves motor one step
        delayMicroseconds(loftSpeed);             //time between steps.  Changing this time will increase or decrease speed of motor
        digitalWrite(loftPulse, LOW);             //resets stepper pulse so that the next pulse will move the stepper motor another step
        delayMicroseconds(loftSpeed);             //time between steps.  Changing this time will increase or decrease speed of motor
        loftdwnStatus = digitalRead(loftdwnPin);  //reads the status of the loft down button and defines the variable
      }
      priorMillis = millis();
      digitalWrite(oscOUTPUT, LOW);  //turns off oscillator
      loopCounter=0; //resets loop counter to 0

      while (remoteState < 20) {
        //reads potiometer values
        ballSpeed = analogRead(velocityPin);  //reads the value of the ball velocity potentiometer 0-1023
        spinValue = analogRead(spinPin);      //reads the value of the ball spin potentiometer 0-1023
        feedRate = analogRead(feedPin);       //reads the value of the ball feed potentiometer 0-1023

        //reads button values
        loftupStatus = digitalRead(loftupPin);        //reads the status of loft up button and defines the variable
        loftdwnStatus = digitalRead(loftdwnPin);      //reads the status of the loft down button and defines the variable
        StartStopStatus = digitalRead(StartStopPin);  //reads the status of the start/stop button

        //FlAT SPIN CONTROL
        //No spin (flat ball) control, if spin potentiometer has no spin, then top and bottom motors should be the same speed
        if (spinValue <= 541 && spinValue >= 481) {
          topMotorSpeed = map(ballSpeed, 0, 1023, 15, 255);     //remaps top motor analog read range to analog write range with a speed minimum above zero
          bottomMotorSpeed = map(ballSpeed, 0, 1023, 15, 255);  //remaps bottom motor analog read range to analog write range with a speed minimum above zero
        }
        //BACKSPIN CONTROL
        //if spin potentiometer has backspin, then the top motor will run slower
        if (spinValue < 481) {
          backSpinMap = map(spinValue, 0, 480, 70, 95);
          backSpinPercent = backSpinMap / 100.;  ////remaps the potentiometer values so that the bottom motor will run at 70-95 percent of the speed of the top motor
          topMotorSpeed = (ballSpeed / 4.011) * backSpinPercent;
          bottomMotorSpeed = map(ballSpeed, 0, 1023, 15, 255);  //remaps bottom motor analog read range to analog write range with a speed minimum above zero
        }
        //TOPSPIN CONTROL
        //if spin potentiometer has topspin, then the bottom motor will run slower
        if (spinValue > 541) {
          topSpinMap = map(spinValue, 542, 1023, 95, 70);
          topSpinPercent = topSpinMap / 100.;  //remaps the potentiometer values so that the bottom motor will run at 70-95 percent of the speed of the top motor
          bottomMotorSpeed = (ballSpeed / 4.011) * (topSpinPercent);
          topMotorSpeed = map(ballSpeed, 0, 1023, 15, 255);  //remaps top motor analog read range to analog write range with a speed minimum above zero
        }

        //currentMillis = millis();
        //elapsedMillis = currentMillis - priorMillis;
        loopCounter++;
        if (loopCounter==1) {
          analogWrite(topMotorDir1, 90);     //jump starts the motors at low speed to ensure they don't stall
          analogWrite(bottomMotorDir1, 90);  //jump starts the motors at low speed to ensure they don't stall
          analogWrite(beep, 255);  // warning beep on
          delay(300);
          analogWrite(beep, 0);  //warning beep off
          delay(800);
          analogWrite(beep, 255);  // warning beep on
          analogWrite(topMotorDir1, 130);
          analogWrite(bottomMotorDir1, 130);
          delay(300);
          analogWrite(beep, 0);  //warning beep off
          delay(800);
          analogWrite(beep, 255);  // warning beep on
          delay(800);
          analogWrite(beep, 0);  //warning beep off
          delay(1500);
        }        

        //if (elapsedMillis < threshold) {
          
        //}
        oscSTATUS = digitalRead(oscillationPin);
        if (oscSTATUS == LOW) {
          digitalWrite(oscOUTPUT, HIGH);
        }
        if (oscSTATUS == HIGH) {
          digitalWrite(oscOUTPUT, LOW);
        }
        //BALL VELOCITY
        //sends pwm signal to ball speed dc motor controller
        analogWrite(topMotorDir1, topMotorSpeed);
        analogWrite(bottomMotorDir1, bottomMotorSpeed);

        //LOFT UP CONTROL
        //while loft up button is pushed it will execute the below commands in a continuous loop
        while (loftupStatus == LOW) {
          // stepper motor/driver commands to adjust loft up
          digitalWrite(loftDir, HIGH);            //High and Low sets direction of loft stepper motor
          digitalWrite(loftPulse, HIGH);          //moves motor one setp
          delayMicroseconds(loftSpeed);           //time between steps.  Changing this time will increase or decrease speed of motor
          digitalWrite(loftPulse, LOW);           //resets stepper pulse so that the next pulse will move the stepper motor another step
          delayMicroseconds(loftSpeed);           //time between steps.  Changing this time will increase or decrease speed of motor
          loftupStatus = digitalRead(loftupPin);  //reads the status of loft up button and defines the variable
        }

        //LOFT DOWN CONTROL
        //while loft down button is pushed it will execute the below commands in a continuous loop
        while (loftdwnStatus == LOW) {
          // stepper motor/driver commands to adjust loft down
          digitalWrite(loftDir, LOW);               //High and Low sets direction of loft stepper motor
          digitalWrite(loftPulse, HIGH);            //moves motor one step
          delayMicroseconds(loftSpeed);             //time between steps.  Changing this time will increase or decrease speed of motor
          digitalWrite(loftPulse, LOW);             //resets stepper pulse so that the next pulse will move the stepper motor another step
          delayMicroseconds(loftSpeed);             //time between steps.  Changing this time will increase or decrease speed of motor
          loftdwnStatus = digitalRead(loftdwnPin);  //reads the status of the loft down button and defines the variable
        }

        feedSpeed = map(feedRate, 0, 1023, 0, 3900);
        stepper1.setSpeed(feedSpeed);
        stepper1.runSpeed();
        unsigned long currentMillis = millis();


        remoteState = analogRead(remote);  //reads the values from RF remote pin
      }
    }
