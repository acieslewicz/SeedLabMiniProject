
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
#include <Encoder.h>
 #define MotorVoltageA 9
 #define MotorVoltageB   10
 #define VoltageSignA 7
 #define VoltageSignB 8
 #define Reset 4
 #define Fault 12
  #define pin1 2
  #define pin2 6
  float Kp = 100;
  float Ki = 10;
  float integralError = 0;
  float positionError;
  int passedPos = 3;
  float Ts = 0;
  float Tc = millis();
  float encoderPosition = 0;
  float encoderRadians = 0;
  float neededPosition[] = {0, PI/2, PI, 3*PI/2};
  String receivedString;
  String resetEncoder = "reset";
  boolean newData = false;
  #define CountsPerRev  3200
  Encoder myEnc(pin1, pin2); 
  int16_t motorSpeed = 0;
  int16_t stringToInt;


void setup()
{
  Serial.begin(9600);  // set up serial so we can output state
  pinMode(MotorVoltageA, OUTPUT);
  pinMode(VoltageSignA, OUTPUT);
  pinMode(MotorVoltageB, OUTPUT);
  pinMode(VoltageSignB, OUTPUT);
  pinMode(Reset,OUTPUT);
  pinMode(Fault,INPUT);
  digitalWrite(Reset, HIGH);
  

  }

  // The loop function reads encoder position and calculated needed speed and direction to get to desired position
void loop()
{
  encoderPosition = myEnc.read();     //Reads current encoder position
  encoderPosition = fmod(encoderPosition,CountsPerRev);
  encoderRadians = (encoderPosition/CountsPerRev)*2*PI;    //modulus and converting to radians
  positionError = neededPosition[passedPos] - encoderRadians;
  if(positionError > PI){ 
    positionError = positionError-2*PI;    //This insures the direction chosen is always the shortest path
  }
  integralError = integralError + ((Ts*positionError)/1000); //implementing the integral error accumulation
  motorSpeed = (int)(Kp*positionError + Ki*integralError);   //calculating motor output
  if(motorSpeed < -255){ //bounding motor speed to usable pwm values
    motorSpeed = -255;
  }
  else if(motorSpeed > 255){
    motorSpeed = 255;
  }
  else if(motorSpeed > -5 && motorSpeed < 5){ //turns motor off too get rid of motor whine
    motorSpeed = 0;
  }
  else if(motorSpeed > -12 && motorSpeed <= -5){  //boosting pwm to overcome friction
    motorSpeed -= 8;
  }
  else if(motorSpeed >= 5 && motorSpeed < 12){
    motorSpeed += 8;
  }
  Ts = millis()-Tc;  //calculating sampling rate for discrete time integral
  Tc = millis();

  //recvString();
  //showNewData();
  motor(motorSpeed);

  
 
}

// The motor function outputs the pwm value passed to it to the correct motor pin by analogWriting the value
void motor(int16_t pwm){
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
// This function checks if the serial input is the reset command or a pwm value to drive and resets the encoder or sets the motor speed accordingly
void showNewData() {
 if (newData == true) {
   if(receivedString.equals(resetEncoder)){
    myEnc.write(0);
   }
   else if(stringToInt>= -255 && stringToInt <= 255){
    motorSpeed = stringToInt;
   }
  
  newData = false;  //resets data waiting flag
 }
}
