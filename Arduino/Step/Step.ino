//The purpose of this code is to perform a step response experiment to create a estimated transfer function for the motor. This is done by driving the motor 
//with a PWM of 255 for 1 second. During this, the encoder position and time elapsed are printed in serial. This will allow us to calculate the radians/sec 
//of the motor, from which we can estimate a transfer function.

#include <Encoder.h>
 #define MotorVoltageA 9
 #define MotorVoltageB   10
 #define VoltageSignA 7
 #define VoltageSignB 8
 #define Reset 4
 #define Fault 12
  #define pin1 2
  #define pin2 6
  float encoderPosition = 0;
  float encoderRadians = 0;
  String receivedString;
  String resetEncoder = "reset";
  boolean newData = false;
  #define CountsPerRev  3200
  Encoder myEnc(pin1, pin2); 
  int16_t speed = 0;
  int16_t stringToInt;
  unsigned long time_now = 0;
  unsigned long period = 5000;
  unsigned long totalPeriod = 0;
  bool hasRun = false;
  bool hasRun2 = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);  // set up serial so we can output state
  pinMode(MotorVoltageA, OUTPUT);
  pinMode(VoltageSignA, OUTPUT);
  pinMode(MotorVoltageB, OUTPUT);
  pinMode(VoltageSignB, OUTPUT);
  pinMode(Reset,OUTPUT);
  pinMode(Fault,INPUT);
  digitalWrite(Reset, HIGH);
  totalPeriod = millis();

}

void loop() {
  if(hasRun == false){
    totalPeriod = millis();
    hasRun = true;
  }
  while((millis() - totalPeriod) >= 3000 && (millis() - totalPeriod) <= 4000){ //insures the total running time is from 3 to 4 seconds after boot
      time_now = micros();
      encoderPosition = myEnc.read(); 
      //encoderPosition = fmod(encoderPosition,CountsPerRev);
      //encoderRadians = (encoderPosition/CountsPerRev)*2*PI;
      Serial.print(encoderPosition);
      Serial.print(";");
      Serial.print(micros());
      Serial.println();
      if(hasRun2 == false){
        motor(255);
        hasRun2 = true;
      }
      while((micros()-time_now) <= period){
        
      }
     
  }
  motor(0);

}
// The motor function outputs the pwm value passed to it to the correct motor pin by analogWriting the value
void motor(int16_t pwm){
    if(pwm > 0){//this insures the positive pwms are sent as normal
    digitalWrite(VoltageSignA, HIGH);
    analogWrite(MotorVoltageA, pwm);
    }
    else if(pwm < 0){// this insures negative pwm values are inverted before passing to pwm and the direction pin is set low to spin the opposite direction
      digitalWrite(VoltageSignA, LOW);
      analogWrite(MotorVoltageA, -pwm);
      Serial.println(pwm);
      }
    else{
      analogWrite(MotorVoltageA, 0);
    }
}
