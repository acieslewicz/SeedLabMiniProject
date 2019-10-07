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
 #define MotorVoltageA 9
 #define MotorVoltageB   10
 #define VoltageSignA 7
 #define VoltageSignB 8
 #define Reset 4
 #define Fault 12
  #define pin1 2
  #define pin2 6
  #define pin3 3
  #define pin4 5
  #define i2c 13
  float Kp = 300;  //proportional control
  float Ki = 11;    //integral control
  float integralErrorLeft = 0;
  float integralErrorRight = 0;
  float positionErrorLeft;
  float positionErrorRight;
  int passedPos = 0;
  float Ts = 0;
  float Tc = millis();
  float encoderPositionLeft = 0.000;
  float encoderPositionRight = 0.000;
  float encoderRadiansLeft = 0.000;
  float encoderRadiansRight = 0.000;
  float neededPositionLeft = 0;
  float neededPositionRight = 0;
  String receivedString;
  String resetEncoder = "reset";
  boolean newData = false;
  #define CountsPerRev  3200
  Encoder rightWheel(pin1, pin2); 
  Encoder leftWheel(pin3, pin4);
  int16_t motorSpeedLeft = 0;
  int16_t motorSpeedRight = 0;
  int16_t stringToInt;
  int distance = 0;
  float angle = 0;


//////////////////////////////////////////////
void setup()
{
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
 if(angle != 0){
    neededPositionLeft = -1*(angle/2);
    neededPositionRight = (angle/2);
    encoderPositionLeft = leftWheel.read();     //Reads current encoder position
    encoderPositionRight = rightWheel.read();
    // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
    encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
    encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
    positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
    positionErrorRight = neededPositionRight - encoderRadiansRight;
    integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
    integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
    motorSpeedLeft = (int)(Kp*positionErrorLeft + Ki*integralErrorLeft);   //calculating motor output in PWM output directly, no need to convert from voltage
    motorSpeedRight = (int)(Kp*positionErrorRight + Ki*integralErrorRight);
    if(motorSpeedLeft < -255){ //bounding motor speed to usable pwm values
      motorSpeedLeft = -255;
    }
    if(motorSpeedRight < -255){ //bounding motor speed to usable pwm values
      motorSpeedRight = -255;
    }
    else if(motorSpeedLeft > 255){
      motorSpeedLeft = 255;
    }
    else if(motorSpeedRight > 255){
      motorSpeedRight = 255;
    }
    else if(motorSpeedLeft > -3 && motorSpeedLeft < 3){ //turns motor off too get rid of motor whine
      motorSpeedLeft = 0;
    }
    else if(motorSpeedRight > -3 && motorSpeedRight < 3){ //turns motor off too get rid of motor whine
      motorSpeedRight = 0;
    }
    else if(motorSpeedLeft > -12 && motorSpeedLeft <= -3){  //boosting pwm to overcome friction
      motorSpeedLeft -= 15;
    }
    else if(motorSpeedRight > -12 && motorSpeedRight <= -3){  //boosting pwm to overcome friction
      motorSpeedRight -= 15;
    }
    else if(motorSpeedLeft >= 3 && motorSpeedLeft < 12){
      motorSpeedLeft += 15;
    }
    else if(motorSpeedRight >= 3 && motorSpeedRight < 12){
      motorSpeedRight += 15;
    }
    Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
    Tc = millis();
  
    
    motor(0,motorSpeedRight);
    motor(1,motorSpeedLeft);
  }
  if(distance != 0){
    neededPositionLeft = ((distance*30.48)/(PI*wheelDiameter))*2*PI;
    encoderPositionLeft = leftWheel.read();     //Reads current encoder position
    encoderPositionRight = rightWheel.read();
    // do not use encoderPosition = fmod(encoderPosition,CountsPerRev);
    encoderRadiansLeft = (encoderPositionLeft/CountsPerRev)*2*PI;    //modulus and converting to radians
    encoderRadiansRight = (encoderPositionRight/CountsPerRev)*2*PI;  
    positionErrorLeft = neededPositionLeft - encoderRadiansLeft;
    positionErrorRight = neededPositionLeft - encoderRadiansRight;
    integralErrorLeft = integralErrorLeft + ((Ts*positionErrorLeft)/1000); //implementing the integral error accumulation
    integralErrorRight  = integralErrorRight + ((Ts*positionErrorRight)/1000);
    motorSpeedLeft = (int)(Kp*positionErrorLeft + Ki*integralErrorLeft);   //calculating motor output in PWM output directly, no need to convert from voltage
    motorSpeedRight = (int)(Kp*positionErrorRight + Ki*integralErrorRight);
    if(motorSpeedLeft < -255){ //bounding motor speed to usable pwm values
      motorSpeedLeft = -255;
    }
    if(motorSpeedRight < -255){ //bounding motor speed to usable pwm values
      motorSpeedRight = -255;
    }
    else if(motorSpeedLeft > 255){
      motorSpeedLeft = 255;
    }
    else if(motorSpeedRight > 255){
      motorSpeedRight = 255;
    }
    else if(motorSpeedLeft > -3 && motorSpeedLeft < 3){ //turns motor off too get rid of motor whine
      motorSpeedLeft = 0;
    }
    else if(motorSpeedRight > -3 && motorSpeedRight < 3){ //turns motor off too get rid of motor whine
      motorSpeedRight = 0;
    }
    else if(motorSpeedLeft > -12 && motorSpeedLeft <= -3){  //boosting pwm to overcome friction
      motorSpeedLeft -= 15;
    }
    else if(motorSpeedRight > -12 && motorSpeedRight <= -3){  //boosting pwm to overcome friction
      motorSpeedRight -= 15;
    }
    else if(motorSpeedLeft >= 3 && motorSpeedLeft < 12){
      motorSpeedLeft += 15;
    }
    else if(motorSpeedRight >= 3 && motorSpeedRight < 12){
      motorSpeedRight += 15;
    }
    Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
    Tc = millis();
  
    
    motor(0,motorSpeedRight);
    motor(1,motorSpeedLeft);
  }

 
}

//////////////////////////////////////////////
// The motor function outputs the pwm value passed to it to the correct motor pin by analogWriting the value
void motor(int motor, int16_t pwm){
    if(motor == 0){
      if(pwm > 0){    //this insures the positive pwms are sent as normal
      digitalWrite(VoltageSignA, HIGH);
      analogWrite(MotorVoltageA, pwm);
      }
      else if(pwm < 0){  // this insures negative pwm values are inverted before passing to pwm and the direction pin is set low to spin the opposite direction
        digitalWrite(VoltageSignA, LOW);
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



//////////////////////////////////////////////
//This function reads the data sent over i2c
void receiveData(int byteCount){
  int inputVal;
  while(Wire.available()) {
    if(state == 0){
      inputVal = Wire.read();
    }
    if(inputVal != 0){
      passedPos = inputVal-1;
    }
    
    
  }
  Serial.println(passedPos);
}

//////////////////////////////////////////////
//This function sends back current position in radians to LCD
void sendData(){
  state = 1;
  String passString = (String)(encoderRadiansRight/2);
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
 if(receivedString.charAt(0) == "-"){
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
