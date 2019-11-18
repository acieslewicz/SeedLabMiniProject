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
  float KpLeft = 900;  //proportional control
  float KiLeft = 6;    //integral control
  float KpRight = 900;  //proportional control
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
  int16_t motorSpeedLeftIntLast = 0;
  int16_t motorSpeedRightIntLast = 0;
  int16_t stringToInt;
  double distance = 0;
  float angle = 0;
  float angle2 = 0;
  float angle2Prev = 0;
  int circle = 0;
  bool useSecondary = 0;
  double secDistance = 8.57;//8.62;//7.5;
  double secDistanceInner = 3.926;
  int receivedDataCount = 0;
  double angleInt;
  double angleDec;
  double distanceInt;
  double distanceDec;
  int firstSend = 0;
 int angle2Count = 1;


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
    while(firstSend == 0){
      Serial.println("Waiting on you fucker...");
      //receivedDataCount += 1;
      angle = PI/7;
      angleFunc();
      angle = 0;
      delay(1700);
      //receivedDataCount += 1;
    }
   
    if(angle != 0){
      angleFunc();
      delay(100);
    }
    else if(distance != 0){
      FowardFunc();
      delay(100);
    }  
    else if(secDistance != 0){
      angle = -PI/2;
      angleFunc();
      delay(100);
      CircleFunc();
      
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
void angleFunc(){
      Serial.println("Started Angle Func");
    while(angle != 0){
      neededPositionLeft = -1*(1.433*angle);
      neededPositionRight = (1.433*angle);
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
      if(motorSpeedLeft < -130){ //bounding motor speed to usable pwm values
        motorSpeedLeft = -130;
      }
      else if(motorSpeedLeft > 130){
        motorSpeedLeft = 130;
      }
      else if(motorSpeedLeft > -1 && motorSpeedLeft < 1){ //turns motor off too get rid of motor whine
        motorSpeedLeft = 0;
      }
      else if(motorSpeedLeft > -12 && motorSpeedLeft <= -1){  //boosting pwm to overcome friction
        motorSpeedLeft -= 15;
      }
      else if(motorSpeedLeft >= 1 && motorSpeedLeft < 12){
        motorSpeedLeft += 15;
      }
      if(motorSpeedRight < -130){ //bounding motor speed to usable pwm values
        motorSpeedRight = -130;
      } 
      else if(motorSpeedRight > 130){
        motorSpeedRight = 130;
      } 
      else if(motorSpeedRight > -1 && motorSpeedRight < 1){ //turns motor off too get rid of motor whine
        motorSpeedRight = 0;
      }
      else if(motorSpeedRight > -12 && motorSpeedRight <= -1){  //boosting pwm to overcome friction
        motorSpeedRight -= 15;
      }
      else if(motorSpeedRight >= 1 && motorSpeedRight < 12){
        motorSpeedRight += 15;
      }
      if((motorSpeedRightInt - motorSpeedRightIntLast) > 5){
        motorSpeedRightInt = motorSpeedRightIntLast + 5;
      }

      if((motorSpeedLeftInt - motorSpeedLeftIntLast) > 5){
        motorSpeedLeftInt = motorSpeedLeftIntLast + 5;
      }

      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      
      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
      
      motor(motorSpeedRightInt,motorSpeedLeftInt);
      
      motorSpeedLeftIntLast = motorSpeedLeftInt;
      motorSpeedRightIntLast = motorSpeedRightInt;
      
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
        motorSpeedLeftIntLast = 0;
        motorSpeedRightIntLast = 0;
        leftWheel.write(0);
        rightWheel.write(0);
        Serial.println("Finished Angle Func");
      }
    }
}

//////////////////////////////////////////////
   void FowardFunc(){   
     Serial.println("Started Forward Func");
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
      if(motorSpeedLeft < -130){ //bounding motor speed to usable pwm values
        motorSpeedLeft = -130;
      }
      else if(motorSpeedLeft > 130){
        motorSpeedLeft = 130;
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
      if(motorSpeedRight < -130){ //bounding motor speed to usable pwm values
        motorSpeedRight = -130;
      } 
      else if(motorSpeedRight > 130){
        motorSpeedRight = 130;
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
      if(angle2Prev == angle2){
        angle2Count +=1;
      }
      else if(angle2Prev != angle2){
        angle2Count = 1;
      }
        if(angle2 < 0){
          motorSpeedRight = ((1-pow((abs(angle2)),angle2Count/2))*motorSpeedRight);
        }
        if(angle2 > 0){
          motorSpeedLeft = ((1-pow((abs(angle2)),angle2Count/2))*motorSpeedLeft);
        }
  
      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
      angle2Prev = angle2;
      
     //Serial.print(motorSpeedRightInt);
     //Serial.print("; ");
     //Serial.println(motorSpeedLeftInt);
      motor(motorSpeedRightInt,motorSpeedLeftInt);
      motorSpeedLeftIntLast = motorSpeedLeftInt;
      motorSpeedRightIntLast = motorSpeedRightInt;
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
        motorSpeedLeftIntLast = 0;
        motorSpeedRightIntLast = 0;
        angle2Count = 1;
        leftWheel.write(0);
        rightWheel.write(0);
        Serial.println("Finished Forward Func");
      }
    }
   }

//////////////////////////////////////////////
void CircleFunc(){   
     Serial.println("Circle Function Started");
    while(secDistance != 0){
      neededPositionRight = ((secDistance*29.5)/(PI*wheelDiameter))*2*PI;
      neededPositionLeft = ((secDistanceInner*29.5)/(PI*wheelDiameter))*2*PI;
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
      if((motorSpeedRightInt - motorSpeedRightIntLast) > 5){
        motorSpeedRightInt = motorSpeedRightIntLast + 15;
      }
  
      if((motorSpeedLeftInt - motorSpeedLeftIntLast) > 5){
        motorSpeedLeftInt = motorSpeedLeftIntLast + 5;
      }

      
      Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
      Tc = millis();
      motorSpeedLeft = motorSpeedLeft*0.955 ;
      motorSpeedLeft = motorSpeedLeft/1.98;

      motorSpeedRightInt = (int)(motorSpeedRight);
      motorSpeedLeftInt = (int)(motorSpeedLeft);
  
      motor(motorSpeedRightInt,motorSpeedLeftInt);
      motorSpeedLeftIntLast = motorSpeedLeftInt;
      motorSpeedRightIntLast = motorSpeedRightInt;
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
        motorSpeedLeftIntLast = 0;
        motorSpeedRightIntLast = 0;
        leftWheel.write(0);
        rightWheel.write(0);
        Serial.println("Finished Circle Func");
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
  angleInt = inputVal[1];
  angleDec = inputVal[2];
  double angleTemp = inputVal[3];
  distanceInt = inputVal[4];
  distanceDec = inputVal[5];
  if(firstSend == 0){
    firstSend = 1;
    angle = angleInt + angleDec/100 + angleTemp/10000;
    if(inputVal[0] == 1){
      angle = angle*-1;
    }
    Serial.println(angle);
    
    distance = distanceInt + distanceDec/100;
  }
  else if(firstSend == 1){
    angle2 = angleInt + angleDec/100 + angleTemp/10000;
    if(inputVal[0] == 1){
      angle2 = angle2*-1;
    }
    Serial.print("SecAngle: ");
    Serial.println(angle2);
  }
  
  receivedDataCount = 0;
  Serial.print("Angle is: ");
  Serial.println(angle,4);
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
