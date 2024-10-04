const float Kp = 20.0; 
const float Ki = 0.006;
const float Kd = 40;
const int setpoint = 35;    
const int baseSpeed = 225;  
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
int positionVal = 0;
int error;
double motorSpeed;
#define buttonpin 12




void setup() {
  pinMode(inp1, OUTPUT);
  pinMode(inp2, OUTPUT);
  pinMode(inp3, OUTPUT);
  pinMode(inp4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(serialEn,OUTPUT);   
  pinMode(junctionPulse,INPUT); 
  Serial2.begin(230400);
  Serial.begin(230400);
  digitalWrite(serialEn,HIGH);
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);

  digitalWrite(inp1,LOW);
  digitalWrite(inp2,HIGH);
  digitalWrite(inp3,LOW);
  digitalWrite(inp4,HIGH);

  Serial.flush();
  Serial2.flush();

}

   
int lastError = 0; 
void loop() {
  digitalWrite(serialEn,LOW);  
  if(Serial2.available()){ 
  positionVal = Serial2.read();  
  // Serial.println(positionVal);   
  if(positionVal == 255) {
  analogWrite(ENA,0);
  analogWrite(ENB,0);
  Serial.println("Stop");
  }{
  digitalWrite(inp1,LOW);

    
    error = setpoint - positionVal;  
    P = error;
    I = I + error;
    D = error-lastError;
    motorSpeed = Kp*P + Ki*I + Kd*D;   
    lastError = error; 
    // Serial.println(motorSpeed);

    int rightMotorSpeed = baseSpeed + (int)motorSpeed;
    int leftMotorSpeed = baseSpeed - (int)motorSpeed;

      rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
      leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);

    Serial.print("leftMotorSpeed  ");
    Serial.println(leftMotorSpeed);
    Serial.print("rightMotorSpeed  ");
    Serial.println(rightMotorSpeed);
    analogWrite(ENA,leftMotorSpeed);
  analogWrite(ENB,rightMotorSpeed);
  }
    digitalWrite(serialEn,HIGH); 
  }
}
