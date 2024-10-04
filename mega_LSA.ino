const float Kp = 0.5; 
const float Ki = 0.0;
const float Kd = 0.0;
const int setpoint = 35;    
const int baseSpeed = 200;  
const int maxSpeed = 255;   
int P,D;
static int I = 0;
const byte rx = 16;    
const byte tx = 17;    
#define serialEn 4    
#define junctionPulse 3   
#define inp1 33   
#define inp2 35
#define inp3 37
#define inp4 39
#define ENA A4   
#define ENB A5  
int positionVal =0;
int error;
double motorSpeed;
#define buttonpin 12
int lastError = 0; 
void forwards(int rightMotorSpeed,int leftMotorSpeed){
  
  digitalWrite(inp1,HIGH);
  digitalWrite(inp2,LOW);
  digitalWrite(inp3,HIGH);
  digitalWrite(inp4,LOW);
  analogWrite(ENA,rightMotorSpeed);
  analogWrite(ENB,leftMotorSpeed);
}
void backwards(){  
  digitalWrite(inp1,LOW);
  digitalWrite(inp2,HIGH);
  digitalWrite(inp3,LOW);
  digitalWrite(inp4,HIGH);
  analogWrite(ENA,100);
  analogWrite(ENB,100);
}

void right(){
  digitalWrite(inp1,HIGH);
  digitalWrite(inp2,LOW);
  digitalWrite(inp3,HIGH);
  digitalWrite(inp4,LOW);
  analogWrite(ENA,100);
  analogWrite(ENB,0);
}

void left(){
  digitalWrite(inp1,HIGH);
  digitalWrite(inp2,LOW);
  digitalWrite(inp3,HIGH);
  digitalWrite(inp4,LOW);
  analogWrite(ENA,0);
  analogWrite(ENB,100);
}
void stop(){
    digitalWrite(inp1,HIGH);
  digitalWrite(inp2,LOW);
  digitalWrite(inp3,HIGH);
  digitalWrite(inp4,LOW);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  Serial.println("stop");
}
void compute_PID(int positionVal)
{
    error = setpoint - positionVal;  
    P = error;
    I = I + error;
    D = error-lastError;
    motorSpeed = Kp*P + Ki*I + Kd*D;   
    lastError = error; 
    // Serial.println(motorSpeed);

    int rightMotorSpeed = baseSpeed + (int)motorSpeed;
    int leftMotorSpeed = baseSpeed - (int)motorSpeed;

    if(rightMotorSpeed > maxSpeed)
    { 
      rightMotorSpeed = maxSpeed;
    }
    if(leftMotorSpeed > maxSpeed) {
      leftMotorSpeed = maxSpeed;
    }
    if(rightMotorSpeed < 0) {

      rightMotorSpeed = 0;
    }
    if(leftMotorSpeed < 0) 
    {
      leftMotorSpeed = 0;
    }
    // Serial.println("L %d\n",leftMotorSpeed);
    // Serial.println("R %d\n",rightMotorSpeed);
    forwards(rightMotorSpeed,leftMotorSpeed);
  }


void setup() {
  pinMode(inp1, OUTPUT);
  pinMode(inp2, OUTPUT);
  pinMode(inp3, OUTPUT);
  pinMode(inp4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(serialEn,OUTPUT);   
  pinMode(junctionPulse,INPUT); 
  Serial2.begin(115200);
  Serial.begin(115200);
  digitalWrite(serialEn,HIGH);
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);

  digitalWrite(inp1,HIGH);
  digitalWrite(inp2,LOW);
  digitalWrite(inp3,HIGH);
  digitalWrite(inp4,LOW);

  Serial.flush();
  Serial2.flush();

}

   

void loop() {
  digitalWrite(serialEn,LOW);  
  if(Serial1.available() ){ 
  positionVal = Serial1.read();  
  Serial.println(positionVal);   
  if(positionVal == 255) {
    stop();
  }else{
      compute_PID(positionVal);
  }
    digitalWrite(serialEn,HIGH); 
  }
}
