#define PPR 60            // Pulses Per Revolution of the encoder.
#define WCFC 39           // Wheel circumference in cm.  // 37.7143 old value
#define distance_test 100 
#define RM_Int 20  
#define LM_Int 21  
#define R_DIR   5
#define L_DIR   8
#define R_speed 6
#define L_speed 9
#define forward  1
#define backward 0
#define Speed 70
boolean Stop;
int RM_Int_current, LM_Int_current, current, distance_in_pulses;
void setup() {
  distance_in_pulses = ( distance_test / WCFC ) * PPR * 2;
  pinMode(R_DIR,  OUTPUT);
  pinMode(L_DIR,  OUTPUT);
  pinMode(R_speed, OUTPUT);
  pinMode(R_speed, OUTPUT);
  pinMode(RM_Int, INPUT_PULLUP);
  pinMode(LM_Int, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RM_Int), RM_Int_Handler, RISING);
  attachInterrupt(digitalPinToInterrupt(LM_Int), LM_Int_Handler, RISING);
  Serial.begin(9600);
  Serial.println("... Starting .../n/n");
  Serial.print("distance_in_pulses: "); 
  Serial.println(distance_in_pulses);
}
void loop()
{
  delay(5000); 
  if(!Stop)
  {   
    analogWrite(R_speed, Speed);
    analogWrite(L_speed, Speed);
    digitalWrite(R_DIR, forward);
    digitalWrite(L_DIR, forward);
  }  
}
void RM_Int_Handler()
{
  RM_Int_current++;  
  Serial.print("RM_Int_current: ");
  Serial.println(RM_Int_current);   
}
void LM_Int_Handler()
{
  LM_Int_current++;  
  Serial.print("LM_Int_current: ");
  Serial.println(LM_Int_current);
}