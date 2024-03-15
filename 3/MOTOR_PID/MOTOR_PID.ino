/* Robot_SimpleMotor_Drive_V0 with encoders
 * 
 * Adafruit Feather M4 using Pololu TB6612FNG motor controller
 * Drives two motors at fixed speed with PI control
 * 
 * Motor Control Table
 * XIN1   XIN2    Effect
 * Low    Low     Brake
 * Low    High    Forward
 * High   Low     Reverse
 * 
 * Free to use for all
 * F Milburn, January 2020
 */
 
 #include <PID_v1.h>
// Output pins used to control motors
const uint16_t PWMA = 6;         // Motor A PWM control     Orange
const uint16_t AIN2 = 7;         // Motor A input 2         Brown
const uint16_t AIN1 = 5;         // Motor A input 1         Green
const uint16_t BIN1 = 8;        // Motor B input 1         Yellow
const uint16_t BIN2 = 9;        // Motor B input 2         Purple
const uint16_t PWMB = 10;        // Motor B PWM control     White
const uint16_t STBY = 13;        // Standby                 Brown
// Motor encoder external interrupt pins
const uint16_t ENCA = 20;        // Encoder A input         Yellow
const uint16_t ENCB = 21;        // Encoder B input         Green
// PWM
const uint16_t ANALOG_WRITE_BITS = 8;
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;
const uint16_t MIN_PWM = MAX_PWM / 4;    // Make sure motor turns
// Motor timing
unsigned long nowTime = 0;       // updated on every loop
unsigned long startTimeA = 0;    // start timing A interrupts
unsigned long startTimeB = 0;    // start timing B interrupts
unsigned long countIntA = 0;     // count the A interrupts
unsigned long countIntB = 0;     // count the B interrupts
double periodA = 0;              // motor A period
double periodB = 0;              // motor B period
// PID 
const unsigned long SAMPLE_TIME = 100;  // time between PID updates
const unsigned long INT_COUNT = 20;     // sufficient interrupts for accurate timing
double setpointA = 150;         // setpoint is rotational speed in Hz
double inputA = 0;              // input is PWM to motors
double outputA = 0;             // output is rotational speed in Hz
double setpointB = 150;         // setpoint is rotational speed in Hz
double inputB = 0;              // input is PWM to motors
double outputB = 0;             // output is rotational speed in Hz
double KpA = 0.20, KiA = 0.20, KdA = 0;
double KpB = 0.20, KiB = 0.20, KdB = 0;
PID motorA(&inputA, &outputA, &setpointA, KpA, KiA, KdA, DIRECT);
PID motorB(&inputB, &outputB, &setpointB, KpB, KiB, KdB, DIRECT);
double storeB = 0;               // used for debug print
void setup(){
 initMotors();
 initEncoders();
 initPWM();
 Serial.begin(9600);
 while(!Serial){
  // wait for serial to start
 }
}
void loop(){
  nowTime = millis();
  motorA.Compute();
  motorB.Compute();
  forwardA((int)outputA);
  forwardB((int)outputB);
  if (storeB != outputB){
    storeB = outputB;
    Serial.println("inputA, inputB, errorA, errorB");
    Serial.print(inputA); Serial.print("  ");
    Serial.print(inputB); Serial.print("  ");
    Serial.print(100*(setpointA-inputA)/setpointA); Serial.print("  ");
    Serial.print(100*(setpointB-inputB)/setpointB); Serial.println("\n");
  }
}
void forwardA(uint16_t pwm){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);
}
void forwardB(uint16_t pwm){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}
void reverseA(uint16_t pwm){
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}
void reverseB(uint16_t pwm){
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);  
  analogWrite(PWMB, pwm);
}
void brakeA(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
void brakeB(){
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
void standbyMotors(bool standby){
  if (standby == true){
    digitalWrite(STBY, LOW);
  }
  else{
    digitalWrite(STBY, HIGH);
  }
}
void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  standbyMotors(false);
}
void initEncoders(){
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isr_A, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), isr_B, RISING);
}
void initPWM(){
  startTimeA = millis();
  startTimeB = millis();
  motorA.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorB.SetOutputLimits(MIN_PWM, MAX_PWM);
  motorA.SetSampleTime(SAMPLE_TIME);
  motorB.SetSampleTime(SAMPLE_TIME);
  motorA.SetMode(AUTOMATIC);
  motorB.SetMode(AUTOMATIC);
}
void isr_A(){
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntA++;
  if (countIntA == INT_COUNT){
    inputA = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeA);
    startTimeA = nowTime;
    countIntA = 0;
  }
}
void isr_B(){
  // count sufficient interrupts to get accurate timing
  // inputX is the encoder frequency in Hz
  countIntB++;
  if (countIntB == INT_COUNT){
    inputB = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeB);
    startTimeB = nowTime;
    countIntB = 0;
  }
}