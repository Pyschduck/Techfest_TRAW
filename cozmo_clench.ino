#include <PS4Controller.h>

int pwm_motor_right=33;               
int pwm_motor_left=25;                 
int dir_motor_right=26;               
int dir_motor_left=27;                 

void forwards(uint8_t speed)
{
  analogWrite(pwm_motor_right,255);
  analogWrite(pwm_motor_left,255);
  digitalWrite(dir_motor_right,LOW);
  digitalWrite(dir_motor_left,HIGH);
}

void backwards(uint8_t speed)
{
  analogWrite(pwm_motor_right,255);
  analogWrite(pwm_motor_left,255);
  digitalWrite(dir_motor_right,HIGH);
  digitalWrite(dir_motor_left,LOW);
}
void right(uint8_t speed)
{
  analogWrite(pwm_motor_right,0);
  analogWrite(pwm_motor_left,255);
  digitalWrite(dir_motor_right,LOW);
  digitalWrite(dir_motor_left,HIGH);
}
void left(uint8_t speed)
{
  analogWrite(pwm_motor_right,255);
  analogWrite(pwm_motor_left,0);
  digitalWrite(dir_motor_right,LOW);
  digitalWrite(dir_motor_left,HIGH);
}
void stop()
{
  analogWrite(pwm_motor_right,0);
  analogWrite(pwm_motor_left,0);
  digitalWrite(dir_motor_right,HIGH);
  digitalWrite(dir_motor_left,HIGH);
}

void setup() {
  pinMode(pwm_motor_right,OUTPUT);
  pinMode(pwm_motor_left,OUTPUT);
  pinMode(dir_motor_right,OUTPUT);
  pinMode(dir_motor_left,OUTPUT);
  digitalWrite(pwm_motor_right,LOW);
  digitalWrite(pwm_motor_left,LOW);
  PS4.begin("00:d4:9e:28:32:f9");
  stop();
}

void loop() {
  if(PS4.isConnected()){
  Serial.println("Connected..");

  int lx = PS4.LStickX();
  int ly = PS4.LStickY();
  int rx = PS4.RStickX();
  int ry = PS4.RStickY();

  printf("lx: %d, ly: %d, rx: %d ry: %d\n", lx ,ly, rx ,ry);

  if (ly > 10)
  {
    forwards(map(ly, 0, 127, 0, 255));
  } 
  else if (ly < -10) 
  {
    backwards(map(ly, -0, -127, 0, 255));
  }
  else if (rx > 10) 
  {
    right(map(rx, 0, 127, 0, 255));
  } 
  else if (rx < -10) {
    left(map(rx, -0, -127, 0, 255));
  } 
  else {
    stop();
  }
}
}