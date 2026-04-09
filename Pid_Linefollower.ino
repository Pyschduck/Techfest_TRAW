const float Kp = 25.0; 
const float Ki = 0.0;
const float Kd = 0.0;
const int setpoint = 35;    
const int baseSpeeda = 25;  
const int baseSpeedb = 25;
const int maxSpeeda = 150;
const int maxSpeedb = 150;   
int P,D;
static int I = 0;
const byte rx = 16;    
const byte tx = 17;    
const byte serialEn = 4;    
const byte junctionPulse = 15;   
const byte dir1 = 25;   
const byte dir2 = 26;   
const byte pwm1 = 27;   
const byte pwm2 = 14;  
int positionVal =0;
int error;
double motorSpeed;
void setup() {
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(serialEn,OUTPUT);   
  pinMode(junctionPulse,INPUT); 
  Serial1.begin(115200, SERIAL_8N1, rx, tx);
  Serial.begin(115200, SERIAL_8N1,3,1);
  digitalWrite(serialEn,HIGH);

  digitalWrite(pwm1,LOW);
  digitalWrite(pwm2,LOW);

  digitalWrite(dir1,HIGH);
  digitalWrite(dir2,LOW);

  Serial.flush();
  Serial1.flush();

}

int lastError = 0;    

void loop() {
  digitalWrite(serialEn,LOW);  
  if(Serial1.available() ){ 
  positionVal = Serial1.read();  
  // Serial.println(positionVal);   
  if(positionVal == 255) {
    error=lastError;
  }else if(digitalRead(junctionPulse)){
    Serial.println("junction");
        analogWrite(pwm1,0);
    analogWrite(pwm2,0);
    }else{
    error = setpoint - positionVal;  
    P = error;
    I = I + error;
    D = error-lastError;
    motorSpeed = Kp*P + Ki*I + Kd*D;   
    lastError = error; 
    // Serial.println(motorSpeed);

    int rightMotorSpeed = baseSpeeda + (int)motorSpeed;
    int leftMotorSpeed = baseSpeedb - (int)motorSpeed;

    if(rightMotorSpeed > maxSpeeda)
    { 
      rightMotorSpeed = maxSpeeda;
    }
    if(leftMotorSpeed > maxSpeedb) {
      leftMotorSpeed = maxSpeedb;
    }
    if(rightMotorSpeed < 0) {

      rightMotorSpeed = 0;
    }
    if(leftMotorSpeed < 0) 
    {
      leftMotorSpeed = 0;
    }
    Serial.printf("L %d\n",leftMotorSpeed);
    Serial.println(rightMotorSpeed);
    // analogWrite(pwm1,rightMotorSpeed);
    // analogWrite(pwm2,leftMotorSpeed);
  }
  }
    digitalWrite(serialEn,HIGH); 
  }
