/*This code is to implement a PI motor control that takes in positions passed from raspberry pi and rotates the motor to 
 * the correct position.
 * Hardware Setup: 
 * Connect motor sheild on top of arduino
 * Connect encoder pins to pins 2 and 6
 * Connect 5v encoder power to 5v pin
 * Connect motor/encoder ground to GND pin
 * Connect motor +/- to correct terminals on motor sheild.
 * Connect Data Line from PI to pin A4
 * Connect Clock Line from PI to pin A5
 * Connect ground from PI to arduino GND
*/

 #define wheelDiameter 15  //cm
 #include <Encoder.h>
 #include <Wire.h>
 #define SLAVE_ADDRESS 0x04   //i2c address for arduino
 int state = 0;
 #define MotorVoltageA 10
 #define MotorVoltageB   9
 #define VoltageSignA 8
 #define VoltageSignB 7
 #define Reset 4
 #define Fault 12
  #define pin1 2
  #define pin2 6
  #define pin3 3
  #define pin4 5
  #define i2c 13
  float KpLeft = 500;  //proportional control
  float KiLeft = 6;    //integral control
  float KpRight = 500;  //proportional control
  float KiRight = 6;    //integral control
  long double integralErrorLeft = 0;
  long double integralErrorRight = 0;
  long double positionErrorLeft;
  long double positionErrorRight;
  int passedPos = 0;
  float Ts = 0;
  float Tc = millis();
  long double encoderPositionLeft = 0.000;
  long double encoderPositionRight = 0.000;
  long double encoderRadiansLeft = 0.000;
  long double encoderRadiansRight = 0.000;
  long double neededPositionLeft = 0;
  long double neededPositionRight = 0;
  String receivedString;
  String resetEncoder = "reset";
  boolean newData = false;
  #define CountsPerRev  3200
  Encoder rightWheel(pin1, pin2); 
  Encoder leftWheel(pin3, pin4);
  long double motorSpeedLeft = 0;
  long double motorSpeedRight = 0; 
  int16_t motorSpeedLeftInt = 0;
  int16_t motorSpeedRightInt = 0;
  int16_t stringToInt;
  double distance = 3;
  float angle = PI/8;
  int circle = 0;
  bool useSecondary = 0;
  double secDistance = 5.939;
  int receivedDataCount = 0;
  double angleInt;
  double angleDec;
  double distanceInt;
  double distanceDec;
 


//////////////////////////////////////////////
void setup(){
  Serial.begin(115200);  // set up serial so we can output state
  pinMode(MotorVoltageA, OUTPUT);  //set motor speed and dir pins as output
  pinMode(VoltageSignA, OUTPUT);
  pinMode(MotorVoltageB, OUTPUT);
  pinMode(VoltageSignB, OUTPUT);
  pinMode(Reset,OUTPUT);  
  pinMode(Fault,INPUT);
  digitalWrite(Reset, HIGH);
  pinMode(i2c,OUTPUT);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  }

//////////////////////////////////////////////
  // The loop function reads encoder position and calculated needed speed and direction to get to desired position
void loop(){
    while(receivedDataCount <4){
      Serial.println("Waiting on you fucker...");
      delay(1000);
    }
   
    if(angle != 0){
      angleFunc();
    }
    else if(distance != 0){
      FowardFunc();
    }  
    else if(secDistance != 0){
      angle = PI/2;
      angleFunc();
      CircleFunc();
      
    }
 
 
}

//////////////////////////////////////////////
// The motor function outputs the pwm value passed to it to the correct motor pin by analogWriting the value
void motor(int motor, int16_t pwm){
    if(motor == 0){
      if(pwm > 0){    //this insures the positive pwms are sent as normal
      digitalWrite(VoltageSignA, LOW);
      analogWrite(MotorVoltageA, pwm);
      }
      else if(pwm < 0){  // this insures negative pwm values are inverted before passing to pwm and the direction pin is set low to spin the opposite direction
        digitalWrite(VoltageSignA, HIGH);
        analogWrite(MotorVoltageA, -pwm);
        }
      else{
        analogWrite(MotorVoltageA, 0);
      }
    }
    if(motor == 1){
      if(pwm > 0){    //this insures the positive pwms are sent as normal
      digitalWrite(VoltageSignB, HIGH);
      analogWrite(MotorVoltageB, pwm);
      }
      else if(pwm < 0){  // this insures negative pwm values are inverted before passing to pwm and the direction pin is set low to spin the opposite direction
        digitalWrite(VoltageSignB, LOW);
        analogWrite(MotorVoltageB, -pwm);
        }
      else{
        analogWrite(MotorVoltageB, 0);
      }
    }
}
///////////////////////////////////
void angleFunc(){
      Serial.println("Started Angle Func");
      delay(1000);
    while(angle != 0){
      neededPositionLeft = -1*(1.46*angle);
      neededPositionRight = (1.46*angle);
      encoderPositionLeft = -1*leftWheel.read();     //Reads current encoder position
      encoderPositionRight = rightWheel.read();
      // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
      encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
      encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
      positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
      positionErrorRight = neededPositionRight - encoderRadiansRight;
      integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
      integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
      motorSpeedLeft = ((KpLeft*positionErrorLeft) + (KiLeft*integralErrorLeft));   //calculating motor output in PWM output directly, no need to convert from voltage
      motorSpeedRight = ((KpRight*positionErrorRight) + (KiRight*integralErrorRight));
      if(motorSpeedLeft < -150){ //bounding motor speed to usable pwm values
        motorSpeedLeft = -150;
      }
      else if(motorSpeedLeft > 150){
        motorSpeedLeft = 150;
      }
      else if(motorSpeedLeft > -3 && motorSpeedLeft < 3){ //turns motor off too get rid of motor whine
        motorSpeedLeft = 0;
      }
      else if(motorSpeedLeft > -12 && motorSpeedLeft <= -3){  //boosting pwm to overcome friction
        motorSpeedLeft -= 15;
      }
      else if(motorSpeedLeft >= 3 && motorSpeedLeft < 12){
        motorSpeedLeft += 15;
      }
      if(motorSpeedRight < -150){ //bounding motor speed to usable pwm values
        motorSpeedRight = -150;
      } 
      else if(motorSpeedRight > 150){
        motorSpeedRight = 150;
      } 
      else if(motorSpeedRight > -3 && motorSpeedRight < 3){ //turns motor off too get rid of motor whine
        motorSpeedRight = 0;
      }
      else if(motorSpeedRight > -12 && motorSpeedRight <= -3){  //boosting pwm to overcome friction
        motorSpeedRight -= 15;
      }
      else if(motorSpeedRight >= 3 && motorSpeedRight < 12){
        motorSpeedRight += 15;
      }
      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      
      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
      
      motor(0,motorSpeedRightInt);
      motor(1,motorSpeedLeftInt);
      if(motorSpeedRightInt == 0 && motorSpeedLeftInt == 0){
        angle = 0;
        encoderPositionLeft = 0;
        encoderPositionRight = 0;
        encoderRadiansRight = 0;
        encoderRadiansLeft = 0;
        integralErrorRight = 0;
        integralErrorLeft = 0;
        positionErrorRight = 0;
        positionErrorLeft = 0;
        leftWheel.write(0);
        rightWheel.write(0);
        Serial.println("Finished Angle Func");
        delay(1000);
      }
    }
}

//////////////////////////////////////////////
   void FowardFunc(){   
     Serial.println("Started Forward Func");
     delay(1000);
    while(distance != 0){
      neededPositionLeft = ((distance*29.5)/(PI*wheelDiameter))*2*PI;
      neededPositionRight = neededPositionLeft;
      encoderPositionLeft = -1*leftWheel.read();     //Reads current encoder position
      encoderPositionRight = rightWheel.read();
      // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
      encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
      encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
      positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
      positionErrorRight = neededPositionRight - encoderRadiansRight;
      integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
      integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
      motorSpeedLeft = ((KpLeft*positionErrorLeft) + (KiLeft*integralErrorLeft));   //calculating motor output in PWM output directly, no need to convert from voltage
      motorSpeedRight = ((KpRight*positionErrorRight) + (KiRight*integralErrorRight));
      //Serial.println((double)(encoderPositionRight));
      if(motorSpeedLeft < -150){ //bounding motor speed to usable pwm values
        motorSpeedLeft = -150;
      }
      else if(motorSpeedLeft > 150){
        motorSpeedLeft = 150;
      }
      else if(motorSpeedLeft > -3 && motorSpeedLeft < 3){ //turns motor off too get rid of motor whine
        motorSpeedLeft = 0;
      }
      else if(motorSpeedLeft > -12 && motorSpeedLeft <= -3){  //boosting pwm to overcome friction
        motorSpeedLeft -= 15;
      }
      else if(motorSpeedLeft >= 3 && motorSpeedLeft < 12){
        motorSpeedLeft += 15;
      }
      if(motorSpeedRight < -150){ //bounding motor speed to usable pwm values
        motorSpeedRight = -150;
      } 
      else if(motorSpeedRight > 150){
        motorSpeedRight = 150;
      } 
      else if(motorSpeedRight > -3 && motorSpeedRight < 3){ //turns motor off too get rid of motor whine
        motorSpeedRight = 0;
      }
      else if(motorSpeedRight > -12 && motorSpeedRight <= -3){  //boosting pwm to overcome friction
        motorSpeedRight -= 15;
      }
      else if(motorSpeedRight >= 3 && motorSpeedRight < 12){
        motorSpeedRight += 15;
      }
      
      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      motorSpeedLeft = motorSpeedLeft*0.955 ;

      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
  
      motor(0,motorSpeedRightInt);
      motor(1,motorSpeedLeftInt);
      if(motorSpeedRightInt == 0 && motorSpeedLeftInt == 0){
        distance = 0;
        encoderPositionLeft = 0;
        encoderPositionRight = 0;
        encoderRadiansRight = 0;
        encoderRadiansLeft = 0;
        integralErrorRight = 0;
        integralErrorLeft = 0;
        positionErrorRight = 0;
        positionErrorLeft = 0;
        leftWheel.write(0);
        rightWheel.write(0);
        Serial.println("Finished Forward Func");
        delay(1000);
      }
    }
   }

//////////////////////////////////////////////
void CircleFunc(){   
     Serial.println("Circle Function Started");
     delay(1000); 
    while(secDistance != 0){
      neededPositionRight = ((secDistance*29.5)/(PI*wheelDiameter))*2*PI;
      neededPositionLeft = neededPositionRight/4.84;
      encoderPositionLeft = -1*leftWheel.read();     //Reads current encoder position
      encoderPositionRight = rightWheel.read();
      // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
      encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
      encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
      positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
      positionErrorRight = neededPositionRight - encoderRadiansRight;
      integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
      integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
      motorSpeedLeft = ((KpLeft*positionErrorLeft) + (KiLeft*integralErrorLeft));   //calculating motor output in PWM output directly, no need to convert from voltage
      motorSpeedRight = ((KpRight*positionErrorRight) + (KiRight*integralErrorRight));
      //Serial.println((double)(encoderPositionRight));
      if(motorSpeedLeft < -150){ //bounding motor speed to usable pwm values
        motorSpeedLeft = -150;
      }
      else if(motorSpeedLeft > 150){
        motorSpeedLeft = 150;
      }
      else if(motorSpeedLeft > -3 && motorSpeedLeft < 3){ //turns motor off too get rid of motor whine
        motorSpeedLeft = 0;
      }
      else if(motorSpeedLeft > -12 && motorSpeedLeft <= -3){  //boosting pwm to overcome friction
        motorSpeedLeft -= 15;
      }
      else if(motorSpeedLeft >= 3 && motorSpeedLeft < 12){
        motorSpeedLeft += 15;
      }
      if(motorSpeedRight < -150){ //bounding motor speed to usable pwm values
        motorSpeedRight = -150;
      } 
      else if(motorSpeedRight > 150){
        motorSpeedRight = 150;
      } 
      else if(motorSpeedRight > -3 && motorSpeedRight < 3){ //turns motor off too get rid of motor whine
        motorSpeedRight = 0;
      }
      else if(motorSpeedRight > -12 && motorSpeedRight <= -3){  //boosting pwm to overcome friction
        motorSpeedRight -= 15;
      }
      else if(motorSpeedRight >= 3 && motorSpeedRight < 12){
        motorSpeedRight += 15;
      }
      
      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      motorSpeedLeft = motorSpeedLeft*0.955 ;
      motorSpeedLeft = motorSpeedLeft/4.3;

      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
  
      motor(0,motorSpeedRightInt);
      motor(1,motorSpeedLeftInt);
      if(motorSpeedRightInt == 0 && motorSpeedLeftInt == 0){
        secDistance = 0;
        encoderPositionLeft = 0;
        encoderPositionRight = 0;
        encoderRadiansRight = 0;
        encoderRadiansLeft = 0;
        integralErrorRight = 0;
        integralErrorLeft = 0;
        positionErrorRight = 0;
        positionErrorLeft = 0;
        leftWheel.write(0);
        rightWheel.write(0);
        Serial.println("Finished Circle Func");
        delay(1000);
      }
    }
   }
  /////////////////////////////////////
//This function reads the data sent over i2c
void receiveData(int byteCount){
  (void) Wire.read();
  double inputVal[4];
  while(Wire.available()) {
    inputVal[receivedDataCount++] = Wire.read();
  }
  angleInt = inputVal[0];
  angleDec = inputVal[1];
  distanceInt = inputVal[2];
  distanceDec = inputVal[3];
  angle = angleInt + angleDec/100;
  distance = distanceInt + distanceDec/100;
  Serial.print("Angle is: ");
  Serial.println(angle);
  Serial.print("Distance is: ");
  Serial.println(distance);
  
}

//////////////////////////////////////////////
//This function sends back current position in radians to LCD
void sendData(){
  double temp = encoderRadiansRight;
  state = 1;
  String passString = (String)(temp/2);
  for(int i = 0; i<5; i++){
    Wire.write((byte)passString.charAt(i));
    //Serial.println((byte)passString.charAt(i));
  }
  state = 0;
}


//////////////////////////////////////////////
// This function waits for an input from the serial monitor and captures any strings, and trims/converts to ints
void recvString() {
 if (Serial.available() > 0) {
 receivedString = Serial.readString();
 receivedString.trim();
 stringToInt = receivedString.toFloat();
 char inputChar = receivedString.charAt(0);
 if(inputChar == '-'){
   receivedString.remove(1,0);
   stringToInt = receivedString.toFloat();
   stringToInt = stringToInt*(-1);
 }

 
 newData = true;  // sets data waiting flag
 }
}

//////////////////////////////////////////////
// This function checks if the serial input is the reset command or a pwm value to drive and resets the encoder or sets the motor speed accordingly
void showNewData() {
 if (newData == true) {
   if(receivedString.equals(resetEncoder)){
    leftWheel.write(0);
    rightWheel.write(0);
   }
   else if(stringToInt == 0){
    passedPos = 0;
   }
   else if(stringToInt == 1){
    passedPos = 1;
   }
   else if(stringToInt == 2){
    passedPos = 2;
   }
   else if(stringToInt == 3){
    passedPos = 3;
   }
  newData = false;  //resets data waiting flag
 }
}
