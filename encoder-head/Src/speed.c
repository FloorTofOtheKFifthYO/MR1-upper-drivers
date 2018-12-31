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

float target_speed= 0;
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

int left_bgn_flag = 0;
int right_bgn_flag = 0;
/*
  函数功能：设置占空比
  参数：直接赋给CCR的占空比
  返回：空
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

/***********************************************速度pid*******************************************/
float speed_P=0;//0.005;
float speed_I=0.0001;
float speed_D=0;
float speed_err,speed_inter,speed_last_err;
float speed_P_out,speed_I_out,speed_D_out;

/**********初始化速度PID中的误差累计值***************/
void init_speed_PID(void)
{
  speed_err=0;
  speed_inter=0;
  speed_last_err=0;
}

/**************速度PID调节函数*****************************/
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

/*************************************************位置pid***********************************************/

float position_err,position_inter,position_last_err;
float position_P_out,position_I_out,position_D_out;
float position_P=0.05;
float position_I=0.15;
float position_D=0.03;

/**********初始化位置PID中的误差累计值***************/
void init_position_PID(void)
{
  position_err=0;
  position_inter=0;
  position_last_err=0;
  TIM2->CNT=32767;
}

/**************位置PID调节函数*****************************/
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


/*****************************时钟滴答******************************/
uint8_t readBuffer[2]={1,1};
uint8_t writeBuffer1[2]={0x40,0x01};
uint8_t writeBuffer2[2]={0xff,0xff};
int read_flag=0;
int buf_flag = 0;
int time1_flag=0;


void HAL_SYSTICK_Callback(void){ 
  static int time_1ms_cnt = 1;
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
  if(time_1ms_cnt % 80== 0 )
  {
      send_wave((float)target_speed,(float)now_speed,(float)now_position,(float)target_position);
  }
  if(time_1ms_cnt >= 65533 )
  {
    time_1ms_cnt=0;
  }
}

/***********读出实际速度和位置*****************************/
void read_speed_position()
{
  /******************磁绝对值磁编码器************/
  /*
    spi_write_read(writeBuffer2, readBuffer,2);
    uprintf("readBuffer1[0]= %x\r\n\r\n",readBuffer[0]);
    uprintf("readBuffer1[1]= %x\r\n\r\n",readBuffer[1]);
    now_position=(float)((uint32_t)(readBuffer[0]&0x3f)*256+readBuffer[1]);
    //uprintf("now_position= %lf\r\n\r\n",now_position);
    now_position=now_position/16384*360;//角度
    uprintf("now_position= %lf\r\n\r\n",now_position);
    //now_position=now_position/16384*2*3.14;//弧度
    */
    /******************普通绝对值磁编码器************/
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
    
    /*****************增量式编码器***************/
    
    int ss=TIM2->CNT;
    now_speed=ss-32767;
    now_position+=now_speed;
    //flash_data[7]=now_position;
    //write_prams();
    TIM2->CNT=32767;
}


/****************************调速度，速度环，位置环*************************/
//本来放在时钟滴答里面，但中断处理函数太多系统会崩溃，采取滴答中断改变标志位的方法，外置处理函数
void set_speed_position()
{
  if(time1_flag==1)//计时器1
  {
      
    read_speed_position();
    if(position_mode==1)//位置环
    {
      int speed_out=(int)position_PID_Control(target_position,now_position);
      set_speed(speed_out); 
    }
    else if(speed_mode==1)//速度环
    {
      int speed_out=(int)speed_PID_Control(target_speed,now_speed);
      set_speed(speed_out); 
    }
    else if(speed_mode==0)//直接设速度不加速度环和位置环
    {
      set_speed((int)target_speed);
    }  
    
    arm_ctrl();
    
    
    time1_flag=0;
  }
}

void arm_ctrl()
{
    int flag_num = left_bgn_flag * 10 + right_bgn_flag;
    if(flag_num == 10)
    {
        target_position += 500;
        if(target_position > 170000) target_position = 170000;
    }
    else if(flag_num == 1)
    {
        target_position -= 500;
        if(target_position < -170000) target_position = -170000;   
    }
}