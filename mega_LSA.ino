const float Kp = 20.0;
const float Ki = 0.006;
const float Kd = 40.0;
const int setpoint = 35;
const int baseSpeed = 225;
const int maxSpeed = 255;
int P, D;
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
int nodecount =0;
int lastError = 0;

const int integralLimit = 1000;  

void forwards()
{
    digitalWrite(inp1, LOW);
  digitalWrite(inp2, HIGH);
  digitalWrite(inp3, LOW);
  digitalWrite(inp4, HIGH);
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);

    
}

void left()
{
    digitalWrite(inp1, HIGH);
  digitalWrite(inp2, LOW);
  digitalWrite(inp3, LOW);
  digitalWrite(inp4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);


}
void right()
{
    digitalWrite(inp1, LOW);
  digitalWrite(inp2, HIGH);
  digitalWrite(inp3, HIGH);
  digitalWrite(inp4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}
void handlenode()
{
  switch(nodecount)
  {
    case 1:
    right();
    break;
    case 2:
    left();
    break;
    case 3:
    forwards();
    break;

  }
  nodecount++;
}

void turn180()
{

  digitalWrite(inp1, LOW);
  digitalWrite(inp2, HIGH);
  digitalWrite(inp3, HIGH);
  digitalWrite(inp4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  // delay(1500);
}

void setup() {
  pinMode(inp1, OUTPUT);
  pinMode(inp2, OUTPUT);
  pinMode(inp3, OUTPUT);
  pinMode(inp4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(serialEn, OUTPUT);
  pinMode(junctionPulse, INPUT);
  Serial2.begin(230400);
  Serial.begin(230400);
  digitalWrite(serialEn, HIGH);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(inp1, LOW);
  digitalWrite(inp2, HIGH);
  digitalWrite(inp3, LOW);
  digitalWrite(inp4, HIGH);

  Serial.flush();
  Serial2.flush();
}

void loop() {
  digitalWrite(serialEn, LOW);  
  if (Serial2.available()) {
    positionVal = Serial2.read();


    if (positionVal == 255) {

      turn180();
    }
      else{

    error = setpoint - positionVal;  
    P = error;
    
    I += error;  
    if (I > integralLimit) I = integralLimit;  
    if (I < -integralLimit) I = -integralLimit;
    
    D = error - lastError; 
    motorSpeed = Kp * P + Ki * I + Kd * D; 
    lastError = error;  

    int rightMotorSpeed = baseSpeed + (int)motorSpeed;
    int leftMotorSpeed = baseSpeed - (int)motorSpeed;

    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    if (rightMotorSpeed < 0) {
      digitalWrite(inp3, HIGH);   
      digitalWrite(inp4, LOW);
      rightMotorSpeed = -rightMotorSpeed;  
    } else {
      digitalWrite(inp3, LOW);    
      digitalWrite(inp4, HIGH);
    }

    if (leftMotorSpeed < 0) {
      digitalWrite(inp1, HIGH);  
      digitalWrite(inp2, LOW);
      leftMotorSpeed = -leftMotorSpeed;  
    } else {
      digitalWrite(inp1, LOW);  
      digitalWrite(inp2, HIGH);
    }

    Serial.print("leftMotorSpeed  ");
    Serial.println(leftMotorSpeed);
    Serial.print("rightMotorSpeed  ");
    Serial.println(rightMotorSpeed);

    analogWrite(ENA, leftMotorSpeed);
    analogWrite(ENB, rightMotorSpeed);
  }

  digitalWrite(serialEn, HIGH);  
  }
}
