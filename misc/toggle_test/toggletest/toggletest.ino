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
volatile int pwm_value = 0;
volatile int prev_time = 0;

void setup() {
  servo_y.attach(11);
  servo_p.attach(9);
  servo_r.attach(8);
  
  Serial.begin(115200);
  // when pin D3 changes, call the gimbal locker funtion
  attachInterrupt(0, rising, RISING);
}

void loop() {
  if(lock_flag==true){
    Serial.println("LOCKED");
  }elif(lock_flag==false){
    Serial.println("UNLOCKED");
  }else{
    //donothing
  }
}
 
void rising() {
  attachInterrupt(0, falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  attachInterrupt(0, rising, RISING);
  pwm_value = micros()-prev_time;
  Serial.println(pwm_value);
  if (pwm_value<900){
    lock_flag=true;
  }
  else if(pwm_value>2000){
    lock_flag=false;
  }
  else{
    //donothing
  }
}
