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
  #define ChangeDirect 11
  float KpLeft = 300;  //proportional control
  float KiLeft = 5;    //integral control
  float KpRight = 300;  //proportional control
  float KiRight = 5;    //integral control
  float KDLeft = 0;    //integral control
  float KDRight = 0;  //proportional control
  long double integralErrorLeft = 0;
  long double integralErrorRight = 0;
  long double positionErrorLeft;
  long double positionErrorRight;
  long double PrevpositionErrorLeft;
  long double PrevpositionErrorRight;
  long double DerivErrorLeft;
  long double DerivErrorRight;
  int passedPos = 0;
  float Ts = 0;
  float Tc = millis();
  long double encoderPositionLeft = 0.000;
  long double encoderPositionRight = 0.000;
  long double encoderPositionLeftLast = 0.000;
  long double encoderPositionRightLast = 0.000;
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
  int16_t motorSpeedLeftIntLast = 0;
  int16_t motorSpeedRightIntLast = 0;
  int16_t stringToInt;
  double distance = 0;
  double Pidistance = 0;
  float angle = 0;
  float angle2 = 0;
  float Piangle = 0;
  float angle2Prev = 0;
  int circle = 0;
  bool useSecondary = 0;
  double secDistance = 8.52;//8.62;//7.5;
  double secDistanceInner = 3.95;
  int receivedDataCount = 0;
  double angleInt;
  double angleDec;
  double distanceInt;
  double distanceDec;
  int firstSend = 0;
 int angle2Count = 1;
 int STOP = 0;
 int ErrorCount = 0;
 int firstLeg = 0;
 int secondLeg = 0;
 int thirdLeg = 0;
 int fourthLeg = 0;
 int fifthLeg = 0;
 int sixthLeg = 0;
 int ZeroLeg = 0;
 int SendData = 1;
 


//////////////////////////////////////////////
void setup(){
  Serial.begin(115200);  // set up serial so we can output state
  pinMode(MotorVoltageA, OUTPUT);  //set motor speed and dir pins as output
  pinMode(VoltageSignA, OUTPUT);
  pinMode(MotorVoltageB, OUTPUT);
  pinMode(VoltageSignB, OUTPUT);
  pinMode(Reset,OUTPUT);  
  pinMode(Fault,INPUT);
  pinMode(ChangeDirect,INPUT_PULLUP);
  digitalWrite(Reset, HIGH);
  pinMode(i2c,OUTPUT);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  }

//////////////////////////////////////////////
  // The loop function reads encoder position and calculated needed speed and direction to get to desired position
void loop(){
    while(firstSend == 0){
      delay(100);
    }

    Serial.println("Out of While");
    if(Piangle != 0 && ZeroLeg == 0){
      angle = Piangle-(0.2);
      Serial.println("FirstPiAngle");
      angleFunc(Piangle-0.2);
      delay(100);
    }
    if(Pidistance != 0 && ZeroLeg == 0){
      Serial.println("FirstPiDist");
      distance = Pidistance;
      ForwardFunc(Pidistance);
      firstSend = 0;
      delay(100);
      angle = -PI/2;
      angleFunc((-PI)/2);
      ZeroLeg = 1;
      STOP = 0;

    }  
    if(firstLeg == 0){
      Serial.println("FirstLeg");
      Piangle = 0;
      Pidistance = 0;
      
      SendData = 1;
      while(STOP == 0){
        Serial.println("FirstStopLoop");
        angle = PI/3;
        angleFunc((PI/3));
        delay(1500);
      }
      while(firstSend == 0){
        Serial.println("FirstSendLoop");
      }
      STOP = 0;
      angle = Piangle;
      distance = Pidistance;
      Serial.println(angle);
      Serial.println(distance);
      angle = angle - (1.6*atan2(0.5,distance));
      distance += 1.0;
      Serial.print("FirstArcTanAngle: ");
      Serial.println(angle);
      angleFunc(angle);
      delay(100);
      ForwardFunc(distance);
      firstLeg = 1;
      firstSend = 0;
    }
    if(secondLeg == 0){
      Piangle = 0;
      Pidistance = 0;
      Serial.println("SecLeg");
      SendData = 1;
      while(STOP == 0){
        angle = PI/3;
        angleFunc((PI/3));
        delay(1500);
      }
      STOP = 0;
      angle = Piangle;
      distance = Pidistance;
      Serial.println(angle);
      Serial.println(distance);
      angle = angle - (1.6*atan2(0.5,distance));
      distance += 1.0;
      Serial.print("SecArcTanAngle: ");
      Serial.println(angle);
      angleFunc(angle);
      delay(100);
      ForwardFunc(distance);
      secondLeg = 1;
      firstSend = 0;
    }
    if(thirdLeg == 0){
      Piangle = 0;
      Pidistance = 0;
      Serial.println("ThirLeg");
      SendData = 1;
      while(STOP == 0){
        angle = PI/3;
        angleFunc((PI/3));
        delay(1500);
      }
      STOP = 0;
      angle = Piangle;
      distance = Pidistance;
      Serial.println(angle);
      Serial.println(distance);
      angle = angle - (1.6*atan2(0.5,distance));
      distance += 1.0;
      Serial.print("ThirdArcTanAngle: ");
      Serial.println(angle);
      angleFunc(angle);
      delay(100);
      ForwardFunc(distance);
      thirdLeg = 1;
      firstSend = 0;
    }
    if(fourthLeg == 0){
      Piangle = 0;
      Pidistance = 0;
      Serial.println("FourthLeg");
      SendData = 1;
      while(STOP == 0){
        angle = PI/3;
        angleFunc((PI/3));
        delay(1500);
      }
      STOP = 0;
      angle = Piangle;
      distance = Pidistance;
      Serial.println(angle);
      Serial.println(distance);
      angle = angle - (1.6*atan2(0.5,distance));
      distance += 1.0;
      Serial.print("FourthArcTanAngle: ");
      Serial.println(angle);
      angleFunc(angle);
      delay(100);
      ForwardFunc(distance);
      fourthLeg = 1;
      firstSend = 0;
    }
    if(fifthLeg == 0){
      Piangle = 0;
      Pidistance = 0;
      Serial.println("FifthLeg");
      SendData = 1;
      while(STOP == 0){
        angle = PI/3;
        angleFunc((PI/3));
        delay(1500);
      }
      STOP = 0;
      angle = Piangle;
      distance = Pidistance;
      Serial.println(angle);
      Serial.println(distance);
      angle = angle - (1.6*atan2(0.5,distance));
      distance += 1.0;
      Serial.print("FifthArcTanAngle: ");
      Serial.println(angle);
      angleFunc(angle);
      delay(100);
      ForwardFunc(distance);
      fifthLeg = 1;
      firstSend = 0;
    }
    if(sixthLeg == 0){
      Piangle = 0;
      Pidistance = 0;
      Serial.println("SixthLeg");
      SendData = 1;
      while(STOP == 0){
        angle = PI/3;
        angleFunc((PI/3));
        delay(1500);
      }
      STOP = 0;
      angle = Piangle;
      distance = Pidistance;
      Serial.println(angle);
      Serial.println(distance);
      angle = angle - (1.6*atan2(0.5,distance));
      distance += 1.0;
      Serial.print("SixthArcTanAngle: ");
      Serial.println(angle);
      angleFunc(angle);
      delay(100);
      ForwardFunc(distance);
      sixthLeg = 1;
      firstSend = 0;
    }
}

//////////////////////////////////////////////
// The motor function outputs the pwm value passed to it to the correct motor pin by analogWriting the value
void motor(int16_t motorR, int16_t motorL){
      if(motorR > 0){    //this insures the positive pwms are sent as normal
      digitalWrite(VoltageSignA, LOW);
      analogWrite(MotorVoltageA, motorR);
      }
      else if(motorR < 0){  // this insures negative pwm values are inverted before passing to pwm and the direction pin is set low to spin the opposite direction
        digitalWrite(VoltageSignA, HIGH);
        analogWrite(MotorVoltageA, -motorR);
        }
      else{
        analogWrite(MotorVoltageA, 0);
      }
    
      if(motorL > 0){    //this insures the positive pwms are sent as normal
      digitalWrite(VoltageSignB, HIGH);
      analogWrite(MotorVoltageB, motorL);
      }
      else if(motorL < 0){  // this insures negative pwm values are inverted before passing to pwm and the direction pin is set low to spin the opposite direction
        digitalWrite(VoltageSignB, LOW);
        analogWrite(MotorVoltageB, -motorL);
        }
      else{
        analogWrite(MotorVoltageB, 0);
      }
}
///////////////////////////////////
void angleFunc(float inAngle){
 // Serial.println("StartedAngleFunc");
      while(angle != 0){
      //neededPositionLeft = -1*(1.433*inAngle);
      neededPositionRight = (1.433*inAngle);
      //encoderPositionLeft = -1*leftWheel.read();     //Reads current encoder position
      encoderPositionRight = rightWheel.read();
      // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
      //encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
      encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
      //positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
      positionErrorRight = neededPositionRight - encoderRadiansRight;
      //PrevpositionErrorLeft = positionErrorLeft;
      PrevpositionErrorRight = positionErrorRight;
     // integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
      integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
     // motorSpeedLeft = ((KpLeft*positionErrorLeft) + (KiLeft*integralErrorLeft) + (KDLeft*DerivErrorLeft));   //calculating motor output in PWM output directly, no need to convert from voltage
      motorSpeedRight = ((KpRight*positionErrorRight) + (KiRight*integralErrorRight) + (KDRight*DerivErrorRight));
      
//      if(motorSpeedLeft < -150){ //bounding motor speed to usable pwm values
//        motorSpeedLeft = -150;
//      }
//      else if(motorSpeedLeft > 150){
//        motorSpeedLeft = 150;
//      }
////      else if(motorSpeedLeft > -1 && motorSpeedLeft < 1){ //turns motor off too get rid of motor whine
////        motorSpeedLeft = 0;
////      }
//      else if(motorSpeedLeft > -12 && motorSpeedLeft <= -1){  //boosting pwm to overcome friction
//        motorSpeedLeft -= 25;
//      }
//      else if(motorSpeedLeft >= 1 && motorSpeedLeft < 12){
//        motorSpeedLeft += 25;
//      }
      if(motorSpeedRight < -150){ //bounding motor speed to usable pwm values
        motorSpeedRight = -150;
      } 
      else if(motorSpeedRight > 150){
        motorSpeedRight = 150;
      } 
//      else if(motorSpeedRight > -1 && motorSpeedRight < 1){ //turns motor off too get rid of motor whine
//        motorSpeedRight = 0;
//      }
      else if(motorSpeedRight > -12 && motorSpeedRight <= -1){  //boosting pwm to overcome friction
        motorSpeedRight -= 15;
      }
      else if(motorSpeedRight >= 1 && motorSpeedRight < 12){
        motorSpeedRight += 15;
      }
      if((motorSpeedRightInt - motorSpeedRightIntLast) > 5){
        motorSpeedRightInt = motorSpeedRightIntLast + 5;
      }

//      if((motorSpeedLeftInt - motorSpeedLeftIntLast) > 5){
//        motorSpeedLeftInt = motorSpeedLeftIntLast + 5;
//      }
      
      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      
      motorSpeedRightInt = (int)(motorSpeedRight);
    //  motorSpeedLeftInt = (int)(motorSpeedLeft);
      
      motor(motorSpeedRightInt,0);//motorSpeedLeftInt);
      
     // motorSpeedLeftIntLast = motorSpeedLeftInt;
      motorSpeedRightIntLast = motorSpeedRightInt;
     
      
     
      //Serial.println(ErrorCount);
    //  encoderPositionLeftLast = encoderPositionLeft;
      encoderPositionRightLast = encoderPositionRight;
      if(motorSpeedRightInt == 0 && motorSpeedLeftInt == 0){
       // Serial.println("FinsihedAngFunc");
        angle = 0;
        encoderPositionLeft = 0;
        encoderPositionRight = 0;
        encoderRadiansRight = 0;
        encoderRadiansLeft = 0;
        integralErrorRight = 0;
        integralErrorLeft = 0;
        positionErrorRight = 0;
        positionErrorLeft = 0;
        motorSpeedLeftIntLast = 0;
        motorSpeedRightIntLast = 0;
        leftWheel.write(0);
        rightWheel.write(0);
        ErrorCount = 0;
      }
    }
}

//////////////////////////////////////////////
   void ForwardFunc(double inDistance){   
    Serial.println("StartedForwardFunc");
     while(distance != 0){
      neededPositionLeft = ((inDistance*29.5)/(PI*wheelDiameter))*2*PI;
      neededPositionRight = neededPositionLeft;
      encoderPositionLeft = -1*leftWheel.read();     //Reads current encoder position
      encoderPositionRight = rightWheel.read();
      // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
      encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
      encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
      positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
      positionErrorRight = neededPositionRight - encoderRadiansRight;
      PrevpositionErrorLeft = positionErrorLeft;
      PrevpositionErrorRight = positionErrorRight;
      integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
      integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
      motorSpeedLeft = ((KpLeft*positionErrorLeft) + (KiLeft*integralErrorLeft) + (KDLeft*DerivErrorLeft));   //calculating motor output in PWM output directly, no need to convert from voltage
      motorSpeedRight = ((KpRight*positionErrorRight) + (KiRight*integralErrorRight) + (KDRight*DerivErrorRight));
    
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
      if((motorSpeedRightInt - motorSpeedRightIntLast) > 1){
        motorSpeedRightInt = motorSpeedRightIntLast + 1;
      }

      if((motorSpeedLeftInt - motorSpeedLeftIntLast) > 1){
        motorSpeedLeftInt = motorSpeedLeftIntLast + 1;
      }
      
      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      motorSpeedLeft = motorSpeedLeft*0.947 ;

  
      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
      
     //Serial.print(motorSpeedRightInt);
     //Serial.print("; ");
     //Serial.println(motorSpeedLeftInt);
      motor(motorSpeedRightInt,motorSpeedLeftInt);
      motorSpeedLeftIntLast = motorSpeedLeftInt;
      motorSpeedRightIntLast = motorSpeedRightInt;
      
      
      encoderPositionLeftLast = encoderPositionLeft;
      encoderPositionRightLast = encoderPositionRight;
     // Serial.println(ErrorCount);
      if(motorSpeedRightInt == 0 && motorSpeedLeftInt == 0){
        Serial.println("FinishedForFunc");
        distance = 0;
        encoderPositionLeft = 0;
        encoderPositionRight = 0;
        encoderRadiansRight = 0;
        encoderRadiansLeft = 0;
        integralErrorRight = 0;
        integralErrorLeft = 0;
        positionErrorRight = 0;
        positionErrorLeft = 0;
        motorSpeedLeftIntLast = 0;
        motorSpeedRightIntLast = 0;
        angle2Count = 1;
        leftWheel.write(0);
        rightWheel.write(0);
        ErrorCount = 0;
        
        
      }
    }
   }

  /////////////////////////////////////
//This function reads the data sent over i2c
void receiveData(int byteCount){
  (void) Wire.read();
  double inputVal[6];
  while(Wire.available()) {
    inputVal[receivedDataCount++] = Wire.read();
  }
  if(inputVal[0] == 3){
    STOP = 1;
    receivedDataCount = 0;
    Serial.println("StopReceived");
    delay(300);
  }
  else if(inputVal[0] == 0){
    receivedDataCount = 0;
  }
  else{
    angleInt = inputVal[1];
    angleDec = inputVal[2];
    double angleTemp = inputVal[3];
    distanceInt = inputVal[4];
    distanceDec = inputVal[5];
      Serial.println("First Send");
      
      Piangle = angleInt + angleDec/100 + angleTemp/10000;
      if(inputVal[0] == 2){
        Piangle = Piangle*-1;
      }
      Pidistance = distanceInt + distanceDec/100;
      firstSend = 1;
      Serial.print("Piangle: ");
      Serial.println(Piangle);
      Serial.print("Pidistance: ");
      Serial.println(Pidistance);
        SendData = 0; 
    receivedDataCount = 0;

  }
}

//////////////////////////////////////////////
//This function sends back current position in radians to LCD
void sendData(){
  
  state = 1;
    Wire.write(SendData);
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
