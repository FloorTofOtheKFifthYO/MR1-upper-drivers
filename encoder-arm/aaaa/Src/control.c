#include "control.h"
#include "tim.h"

#include "pid.h"

PID_S Speed_PID={0,0,0,0,0,1000,0,0.005};
PID_S Position_PID={0,0,0,0,0,1000,0,0.005};

float Now_Speed;
float Now_Position;

float Target_Speed;
float Target_Position;
typedef enum{
  PWM_Mode=0x00,
  Speed_Mode=0x01,
  Position_Mode=0x02,
  Position_Speed_Mode=0x03
}Mode;

Mode Motor_Mode=PWM_Mode;

void Get_Speed(){
  Now_Speed=(int16_t)TIM2->CNT;
  TIM2->CNT=0;
  
  Now_Position+=Now_Speed*0.005;
}

void Set_Pwm(uint8_t ch,uint16_t  ccr){
  __IO uint32_t * ptr;
  if(ch!=1 && ch!=2){
    return ;
  }
  ptr=&(TIM4->CCR1)+(ch-1);
  *ptr=ccr;
}

void Set_Speed(float  speed){
  speed=speed>2700?2700:speed;
  speed=speed<-2700?-2700:speed;
  
  if(speed>0){
    Set_Pwm(1,0);
    Set_Pwm(2,(uint16_t)speed);
  }else{
    speed=-speed;
    Set_Pwm(2,0);
    Set_Pwm(1,(uint16_t)speed);
  }
}

void Control(){
  float speed_out=0;
  float position_out=0;
  
  if(Motor_Mode==Position_Mode){
    position_out=PID_Control(&Position_PID,Target_Position,Now_Position);
    Set_Speed(position_out);
    return ;
  }
  
  if(Motor_Mode&Speed_Mode){
    if(Motor_Mode&Position_Mode){ //位置速度环
      position_out=PID_Control(&Position_PID,Target_Position,Now_Position);
      Target_Speed=position_out;
    }
    speed_out=PID_Control(&Speed_PID,Target_Speed,Now_Speed);
    Set_Speed(speed_out);
    return ;
  }
}



