#include "tim.h"
#include "speed.h"
#include "imitate_spi.h"
#include "usart.h"
#include "can_func.h"
#include "flash.h"
#include "math.h"
#include <stdlib.h>
#define PI 3.1415926
#define EXP 2.718281828

float target_speed= 220;//140;//1000;
float now_speed;
float target_position=0;
float now_position=0;
float last_position;
float buf_position = 0;
float last_position = 0;

int speed_mode=0;
int position_mode=0;
int time_interval = 100;

float target_angle=1.57;

float angle_K=54.1401;
float angle_B=50;
float param_a = 600;
float param_b = 0.7;
int real_target_speed = 0;
//צ�ӵ�pidΪ 5500 0 0

/*read_pos������Զ�ȡ������λ��
position ��ֵ ���Զ���target_position*/

/*
  �������ܣ�����ռ�ձ�
  ������ֱ�Ӹ���CCR��ռ�ձ�
  ���أ���
*/
void set_speed(int out){  
  if(out>2700){
    out=2700;
  }
  else if(out<-2700){
    out=-2700;
  }
  
  if(out>=0){
    TIM4->CCR1=3000-out;
    TIM4->CCR2=3000; 
  }
  else if(out<0){
    TIM4->CCR1=3000;
    TIM4->CCR2=3000+out;
  }
}

/***********************************************�ٶ�pid*******************************************/
float speed_P=0.5;
float speed_I=1.5;
float speed_D=0;
float speed_err,speed_inter,speed_last_err;
float speed_P_out,speed_I_out,speed_D_out;

/**********��ʼ���ٶ�PID�е�����ۼ�ֵ***************/
void init_speed_PID(void)
{
  speed_err=0;
  speed_inter=0;
  speed_last_err=0;
}

/**************�ٶ�PID���ں���*****************************/
float speed_PID_Control(float target_speed,float now_speed)
{
  speed_err=target_speed-now_speed;
  speed_inter+=speed_err;
  
  if(speed_inter>2000)
    speed_inter=2000;
  if(speed_inter<-2000)
    speed_inter=-2000;
  
  speed_P_out=speed_err*speed_P;
  speed_I_out=speed_inter*speed_I;
  speed_D_out=(speed_err-speed_last_err)*speed_D;
  speed_last_err=speed_err;  
  return speed_P_out+speed_I_out+speed_D_out;
}

/*************************************************λ��pid***********************************************/

float position_err,position_inter,position_last_err;
float position_P_out,position_I_out,position_D_out;
float position_P=0.008;
float position_I=0;
float position_D=0;

/**********��ʼ��λ��PID�е�����ۼ�ֵ***************/
void init_position_PID(void)
{
  position_err=0;
  position_inter=0;
  position_last_err=0;
  TIM2->CNT=32767;
}

/**************λ��PID���ں���*****************************/
float position_PID_Control(float target_position,float now_position)
{
  position_err=target_position-now_position;
  position_inter+=position_err;
  
  if(position_inter>2000)
    position_inter=2000;
  if(position_inter<-2000)
    position_inter=-2000;
  
  position_P_out=position_err*position_P;
  position_I_out=position_inter*position_I;
  position_D_out=(position_err-position_last_err)*position_D;
  position_last_err=position_err;    
  return position_P_out+position_I_out+position_D_out;
}


/*****************************ʱ�ӵδ�******************************/
uint8_t readBuffer[2]={1,1};
uint8_t writeBuffer1[2]={0x40,0x01};
uint8_t writeBuffer2[2]={0xff,0xff};
int read_flag=0;
int buf_flag = 0;
int time1_flag=0;


void HAL_SYSTICK_Callback(void){ 
  static int time_1ms_cnt = 0;
  if(main_flag == 1)
  time_1ms_cnt++;
  if(time_1ms_cnt%10 == 0)
  {
    if(time1_flag==0)
    {
      time1_flag=1;
    }  
  }
  if(time_1ms_cnt%time_interval  == 0)
  {
    if(buf_flag==0)
    {
      buf_flag=1;
    }  
  }
  if(time_1ms_cnt % 10== 0 )
  {
      send_wave((float)real_target_speed,(float)now_speed,(float)now_position,(float)target_position);
  }
  if(time_1ms_cnt >= 65533 )
  {
    time_1ms_cnt=0;
  }
}

/***********����ʵ���ٶȺ�λ��*****************************/
void read_speed_position()
{
  /******************�ž���ֵ�ű�����************/
  /*
    spi_write_read(writeBuffer2, readBuffer,2);
    uprintf("readBuffer1[0]= %x\r\n\r\n",readBuffer[0]);
    uprintf("readBuffer1[1]= %x\r\n\r\n",readBuffer[1]);
    now_position=(float)((uint32_t)(readBuffer[0]&0x3f)*256+readBuffer[1]);
    //uprintf("now_position= %lf\r\n\r\n",now_position);
    now_position=now_position/16384*360;//�Ƕ�
    uprintf("now_position= %lf\r\n\r\n",now_position);
    //now_position=now_position/16384*2*3.14;//����
    */
    /******************��ͨ����ֵ�ű�����************/
    /*
    uint16_t data;
    data = spi_read_encoder();
    //uprintf("data = %d\r\n",data);
    now_position = (float)data*2*PI/1024;
    now_speed = now_position - last_position;
    while(now_speed > PI) now_speed -= 2*PI;
    while(now_speed < -PI) now_speed += 2*PI;
    last_position = now_position;
    //uprintf("now_position = %f\r\n",now_position);*/
    
    /*****************����ʽ������***************/
    
    int ss=TIM2->CNT;
    now_speed=ss-32767;
    now_position+=now_speed;
    //flash_data[7]=now_position;
    //write_prams();
    TIM2->CNT=32767;
}

int acc_calculate_speed(int last_x, int target_x, int max_speed)
{
    
    int x1 = abs((int)now_position - last_x);
    int x2 = abs(target_x - (int)now_position);
    double speed = 0;
    if(x1 <= x2)
    {
         speed = (param_a / 2) * pow(1 - pow(EXP, (-2 * x1 / param_b)), 0.5) + 60;
         if(speed >= max_speed)
         {
             speed = max_speed;
         }
    }
    else 
    {
        speed = (param_a / 2) * pow(1 - pow(EXP, (-2 * x2 / param_b)), 0.5);
         if(speed >= max_speed)
         {
             speed = max_speed;
         }
    }
    return (int)speed;
}

int arm_last_position = 0;//1200;
int arm_target_position = 13800;//15000;
int f_goflag = 0;
int judge_range = 300;

void raise_arm()
{
    read_speed_position();
    int range;
    int arm_target_speed;
    int speed_out;
    switch(f_goflag)
    {
    case 0:
        arm_target_speed=(int)position_PID_Control(arm_last_position,now_position);
        break;
    case 1:
        arm_target_speed = acc_calculate_speed(arm_last_position, arm_target_position, (int)target_speed);
        range = abs(arm_target_position - (int)now_position);
        if(range < judge_range)
        {
            f_goflag = 2;
            arm_msg[3] = 1;
        }
        break;
    case 2:
        arm_target_speed=(int)position_PID_Control(arm_target_position,now_position);
        break;
    case 3:
        arm_target_speed = -acc_calculate_speed(arm_target_position, arm_last_position, (int)target_speed);
        range = abs(arm_last_position - (int)now_position);
        if(range < judge_range)
        {
            f_goflag = 0;
            arm_msg[4] = 1;
        }
        break;
    }
    speed_out=(int)speed_PID_Control(arm_target_speed,now_speed);
    set_speed(speed_out); 
}

/****************************���ٶȣ��ٶȻ���λ�û�*************************/
//��������ʱ�ӵδ����棬���жϴ�����̫��ϵͳ���������ȡ�δ��жϸı��־λ�ķ��������ô�����
void set_speed_position()
{
  if(time1_flag==1)//��ʱ��1
  {
      /*
    read_speed_position();
    if(position_mode==1)//λ�û�
    {
      if(target_position - now_position > 500)
      {
          real_target_speed = target_speed;
      }
      else if(target_position - now_position < -500)
      {
          real_target_speed = -target_speed;
      }
      else real_target_speed = 0;
      int speed_out=(int)speed_PID_Control(real_target_speed,now_speed);
      set_speed(speed_out); 
    }
    else if(speed_mode==1)//�ٶȻ�
    {
      int speed_out=(int)speed_PID_Control(target_speed,now_speed);
      set_speed(speed_out); 
    }
    else if(speed_mode==0)//ֱ�����ٶȲ����ٶȻ���λ�û�
    {
      set_speed((int)target_speed);
    }*/
      
    raise_arm();
    time1_flag=0;
  }
}

void buffer_position_ctrl()
{
    if(buf_flag == 0) return ;
    if(buf_position - target_position > 0.174)//10��
    {
        target_position += 0.174;
    }
    else if(buf_position - target_position < -0.174)
    {
        target_position -= 0.174;
    }
    else
    {
        target_position = buf_position;
    }
    buf_flag = 0;
}