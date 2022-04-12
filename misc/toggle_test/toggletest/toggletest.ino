/*
 * steps:
 * 1) Read
 * 2) Get period length
 * 3) Loop:
 *    3a) get changes
 *    3b) flip switch
 * 
 */
#include <Servo.h>
#define TOGGLE_PIN 3
Servo servo_y, servo_p, servo_r;
volatile boolean lock_flag;

void setup() {
  servo_y.attach(11);
  servo_p.attach(9);
  servo_r.attach(8);
  
  Serial.begin(115200);
  // when pin D3 changes, call the gimbal locker funtion
  attachInterrupt(0,gimbal_locker , CHANGE);
}
 
void loop() {
  if (lock_flag==true){
    servo_y.write(90);
    servo_p.write(90);
    servo_q.write(90);
  }else{
    //do gimbal stuff
  }
}
 
void gimbal_locker() {
  if(digitalRead(TOGGLE_PIN) == HIGH){
    lock_flag=true; 
  }else{
    lock_flag=false;
  }
}
