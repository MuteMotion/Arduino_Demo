#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>

int minDistance=25,Kp=(5),Ki=(1),Kd=(0),T=50,prevErr=0,newSpeed=0;

#define NUM_OF_MOTORS 2
#define NUM_OF_ULTRASONICS  6

typedef struct 
{
  int FRO;
  int BACK;
  int SPEED;
  int ENC;
}Motors_t;

unsigned long last_time=0;

typedef struct
{
 int TRIG;
 int ECHO;
}ULTRASONIC_t;

typedef enum 
{
  FRONT,FRONT_RIGHT,FRONT_LEFT,BACKWARD,BACKWARD_RIGHT,BACKWARD_LEFT
}US_DIR;

Motors_t motors[NUM_OF_MOTORS]=
{
  {4,5,6,10},
  {7,8,9,11}
};

ULTRASONIC_t ultrasonic[NUM_OF_ULTRASONICS]
{
  {22,23},
  {24,25},
  {26,27},
  {28,29},
  {30,31},
  {32,33}
};

int ultrasonic_readings[NUM_OF_ULTRASONICS]={0};

double error;
double p,i,d;

void Task_ACC(void *param);
void Task_CollisionWarning(void *param);
void Task_CollisionWarning2(void *param);

void Task_ReadAllUS(void *param);
TaskHandle_t Task_ACC_Handler;

TaskHandle_t Task_CollisionWarning_Handler;
TaskHandle_t Task_CollisionWarning_Handler2;

TaskHandle_t Task_ReadAllUS_Handler;

/*************************************************************************************/
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

void setup() {
  // put your setup code here, to run once:
  for(int i=0;i<NUM_OF_ULTRASONICS;i++)
  {
    pinMode(ultrasonic[i].TRIG,OUTPUT);
    pinMode(ultrasonic[i].ECHO,INPUT);
  }
  for(int i=0;i<NUM_OF_MOTORS;i++)
  {
    pinMode(motors[i].FRO,OUTPUT);
    pinMode(motors[i].BACK,OUTPUT);
    pinMode(motors[i].SPEED,OUTPUT);
    /*******************************************************/
    pinMode(motors[i].ENC, INPUT_PULLUP);
  }

  xTaskCreate(Task_ACC,"ACC",1000,NULL,1,&Task_ACC_Handler);
  xTaskCreate(Task_ReadAllUS,"readAllUS",1000,NULL,1,&Task_ReadAllUS_Handler);
  xTaskCreate(Task_CollisionWarning,"Collision_warning",1000,NULL,1,&Task_CollisionWarning_Handler);
  xTaskCreate(Task_CollisionWarning2,"Collision_warning",1000,NULL,1,&Task_CollisionWarning_Handler2);
    /*******************************************************/
  attachInterrupt(digitalPinToInterrupt(motors[0].ENC), isr_A, RISING);
  attachInterrupt(digitalPinToInterrupt(motors[1].ENC), isr_B, RISING);
  initPWM();
  Serial.begin(115200);

}

void loop() {}

void Task_ACC(void *param)
{
  (void) param;
  while(1)
  {
    Serial.println("ACC");
    if(ultrasonic_readings[FRONT] != 0)
    {
      if(ultrasonic_readings[FRONT] <= minDistance)
      {
        AEB();
      }
      else
      {
        pidAlgo(ultrasonic_readings[FRONT]); //pid for ACC
      }
    }
    vTaskDelay(200/portTICK_PERIOD_MS);
  }
}
void Task_CollisionWarning(void *param)
{
  (void) param;
  while(1)
  {
     vTaskDelay(1000/portTICK_PERIOD_MS);
    //  TestWarning(FRONT);
    //  TestWarning(FRONT_RIGHT);
    //  TestWarning(FRONT_LEFT);
     TestWarning(BACKWARD);
    TestWarning(BACKWARD_LEFT);
    TestWarning(BACKWARD_RIGHT);
  }
}
void Task_CollisionWarning2(void *param)
{
  (void) param;
  while(1)
  {
     vTaskDelay(800/portTICK_PERIOD_MS);
     TestWarning(FRONT);
     TestWarning(FRONT_RIGHT);
     TestWarning(FRONT_LEFT);
    //  TestWarning(BACKWARD);
    // TestWarning(BACKWARD_LEFT);
    // TestWarning(BACKWARD_RIGHT);
  }
}
void Task_ReadAllUS(void *param)
{
  (void) param;
  while(1)
  {
    Serial.println("US");
    ultrasonic_readings[FRONT]=readUS(ultrasonic[FRONT]);
    ultrasonic_readings[FRONT_RIGHT]=readUS(ultrasonic[FRONT_RIGHT]);
    ultrasonic_readings[FRONT_LEFT]=readUS(ultrasonic[FRONT_LEFT]);
    ultrasonic_readings[BACKWARD_LEFT]=readUS(ultrasonic[BACKWARD_LEFT]);
    ultrasonic_readings[BACKWARD]=readUS(ultrasonic[BACKWARD]);
    ultrasonic_readings[BACKWARD_RIGHT]=readUS(ultrasonic[BACKWARD_RIGHT]);
    vTaskDelay(150/portTICK_PERIOD_MS);
  }
}

int readUS(ULTRASONIC_t currentUS)
{
  int distance=0;
  digitalWrite(currentUS.TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(currentUS.TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(currentUS.TRIG, LOW);
  distance = pulseIn(currentUS.ECHO, HIGH);
  distance = distance * 0.034 / 2;
  return distance;
}

void TestWarning(int DIR)
{
  switch(DIR)
  {
    case FRONT: Serial.print("FRONT"); break;
    case FRONT_LEFT: Serial.print("FRONT_LEFT"); break;
    case FRONT_RIGHT: Serial.print("FRONT_RIGHT"); break;
    case BACKWARD: Serial.print("BACKWARD"); break;
    case BACKWARD_LEFT: Serial.print("BACKWARD_LEFT"); break;
    case BACKWARD_RIGHT: Serial.print("BACKWARD_RIGHT"); break;
    default : break;
  }
  if(ultrasonic_readings[DIR] < minDistance)
  {
    Serial.println(" RED");
  }
  else if(ultrasonic_readings[DIR] > minDistance+20)
  {
    Serial.println(" GREEN");
  }
  else
  {
    Serial.println(" Yellow");
  }
}
void AEB()
{
  digitalWrite(motors[0].SPEED,LOW);
  digitalWrite(motors[1].SPEED,LOW);
}

void pidAlgo(int position)
{
  if(millis() - last_time >= T)
  {
    last_time=millis();
    error=position-minDistance;
    p=error;
    i=error+i;
    if(i <= -255) i=-255;
    if (i>=0) i=0;
    d=error-prevErr;
    newSpeed=(Kp*p)+(Ki*i*T)+((Kd/T)*d);
    prevErr=error;
    motorControl(newSpeed);
  }
}

void motorControl(int newSpeed)
{
  if (newSpeed <=255 & newSpeed >=0){
  nowTime = millis();
  setpointA=newSpeed;
  setpointB=newSpeed;
  motorA.Compute();
  motorB.Compute();
  forwardA((int)outputA);
  forwardB((int)outputB);
  }
  else
  {
    AEB();
  }
  // digitalWrite(motors[0].FRO,HIGH);
  // digitalWrite(motors[0].BACK,LOW);
  // digitalWrite(motors[1].FRO,HIGH);
  // digitalWrite(motors[1].BACK,LOW);
  // if (newSpeed <=255 & newSpeed >=0){
  //   analogWrite(motors[0].SPEED, newSpeed);  //set motor speed 
  //   analogWrite(motors[1].SPEED, newSpeed);  //set motor speed 
  //   Serial.println(newSpeed);
  // }
  // else{
  //   if (newSpeed >=255){
  //     analogWrite(motors[0].SPEED, 255);  //set motor speed 
  //     analogWrite(motors[1].SPEED, 255);  //set motor speed 
  //     Serial.println(newSpeed);
  //   }
  //   else{
  //     AEB();
  //     Serial.println(newSpeed);
  //   }
  // }
}
/*******************************************************/
void forwardA(uint16_t pwm){
  digitalWrite(motors[0].BACK, LOW);
  digitalWrite(motors[0].FRO, HIGH);
  analogWrite(motors[0].SPEED, pwm);
}
void forwardB(uint16_t pwm){
  digitalWrite(motors[1].BACK, LOW);
  digitalWrite(motors[1].FRO, HIGH);
  analogWrite(motors[1].SPEED, pwm);
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