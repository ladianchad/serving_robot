#include <PWM.h>
#include <TimerFive.h>
#include <m_pid.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <robot_msgs/Motor.h>

#define M2_pwm 10
#define M2_dir 12

#define M1_pwm 11
#define M1_dir 13

#define EN_L_1 20
#define EN_L_2 21

#define EN_R_1 18
#define EN_R_2 19


void m1_move(int pwm,bool dir);
void m2_move(int pwm,bool dir);
double m1_speed();
double m2_speed();
void todo_1();
void todo_2();
void messageCb( const robot_msgs::Motor& msg);

ros::NodeHandle  nh;
m_pid motor_contrl(2,10);               // motor contrl scheduler setting, (how many motor , state nubmer)  ** max motor number is 10 , max state number is 10

robot_msgs::Motor sp_msg;
ros::Publisher pub("/Motor/speed", &sp_msg);
ros::Subscriber<robot_msgs::Motor> sub("/Motor/speed_set", &messageCb );

long ctrl_ratio = 10000;
long l_en =0;
long r_en =0;

void messageCb( const robot_msgs::Motor& msg){
  motor_contrl.set_target(0,msg.left.data);
  motor_contrl.set_target(1,msg.right.data);
}


void setup(){
  pinMode(M1_pwm,OUTPUT);
  pinMode(M1_dir,OUTPUT);
  pinMode(M2_pwm,OUTPUT);
  pinMode(M2_dir,OUTPUT);
  pinMode(52,OUTPUT);
  pinMode(EN_L_1,INPUT);
  pinMode(EN_L_2,INPUT);
  pinMode(EN_R_1,INPUT);
  pinMode(EN_R_2,INPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  attachInterrupt(digitalPinToInterrupt(EN_L_1),L_encoder_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_L_2),L_encoder_2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_R_1),R_encoder_1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(EN_R_2),R_encoder_2,CHANGE);
  
  Timer5.initialize(ctrl_ratio);
  Timer5.attachInterrupt(timer_hander);
  Timer5.start();
  motor_contrl.motor_move(0,m1_move);   // setting funtion to move motor
  motor_contrl.motor_move(1,m2_move);
  
  
  motor_contrl.get_speed(0,m1_speed);   // setting funtion to get motor speed
  motor_contrl.get_speed(1,m2_speed);

  
  motor_contrl.pid_set(0,0.4,0.15,0.1);     // setting motor P , I , D value   (motor index , P value, I value , D value)
  motor_contrl.pid_set(1,1,0.2,0.1);
  motor_contrl.set_target(0,0);
  motor_contrl.set_target(1,0);

  motor_contrl.push_todo(3,todo_1);     //  setting to do funtion (todo index , todo funtion )  ** funtion must be return void, and argument is void 
                                        //    ** 0,1 index using motor contrl , so you have to todo index over 1
}

void loop() {
  motor_contrl.ctrl_start();
}
bool led = false;

void timer_hander(){
  motor_contrl.timer_flag = true;
  motor_contrl.state = (motor_contrl.state + 1)%motor_contrl.max_state;
}
void todo_1(){
  pub.publish(&sp_msg);
  nh.spinOnce();
}
void m1_move(int pwm,bool dir){ 
  digitalWrite(M1_dir,dir);
  analogWrite(M1_pwm,pwm);
}
void m2_move(int pwm,bool dir){
  digitalWrite(M2_dir,dir);
  analogWrite(M2_pwm,pwm);
}
double m1_speed(){
  double speed = (l_en *10*60)/2652.0;
  l_en = 0;
  sp_msg.left.data =speed;
  return speed;
}

double m2_speed(){
  double speed = (r_en *10*60)/2652.0;
  r_en = 0;
  sp_msg.right.data =speed;
  return speed;
}

void L_encoder_1(){
    if(digitalRead(EN_L_1) == digitalRead(EN_L_2))
      l_en--;
    else
      l_en++;
}
void L_encoder_2(){
    if(digitalRead(EN_L_1) == digitalRead(EN_L_2))
      l_en++;
    else
      l_en--;
}
void R_encoder_1(){
    if(digitalRead(EN_R_1) == digitalRead(EN_R_2))
      r_en++;
    else
      r_en--;
}

void R_encoder_2(){
    if(digitalRead(EN_R_1) == digitalRead(EN_R_2))
      r_en--;
    else
      r_en++;
}
