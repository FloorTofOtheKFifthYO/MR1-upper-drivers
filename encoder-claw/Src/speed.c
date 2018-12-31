#include "tim.h"
#include "speed.h"
#include "imitate_spi.h"
#include "usart.h"
#include "can_func.h"
#include "flash.h"
#include "can.h"

float target_speed=0;//1000;
float now_speed;
float target_position=0;
float now_position=0;
float last_position;
float buf_position = 0;

int speed_mode=0;
int position_mode=0;
int time_interval = 100;

float target_angle=1.57;

float angle_K=54.1401;
float angle_B=50;
float position_P=15500;
float position_I=0;
float position_D=0;

int first_send_can_msg = 0;

//爪子的pid为 5500 0 0

//爪臂的pid为 3300 0 11000，脱齿，未安装拐骨，届时p需要更大，爪子调节需要加延时
/*read_pos命令可以读取编码器位置
position 数值 可以定义target_position
*/
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
    /*
  if(out>1500){
    out=1500;
  }
  else if(out<-1500){
    out=-1500; 
  }*/
  
  if(out>=0){
    TIM4->CCR1=3000-out;
    TIM4->CCR2=3000; 
  }
  else if(out<0){
    TIM4->CCR1=3000;
    TIM4->CCR2=3000+out;
  }

  /*
  if(out>=0){
    TIM4->CCR1=0;
    TIM4->CCR2=out; 
  }
  else if(out<0){
    TIM4->CCR1=-out;
    TIM4->CCR2=0;
  }
  */
  //send_wave((float)out,(float)0,(float)now_position,(float)target_position);
}

/***********************************************速度pid*******************************************/
float speed_P=0;
float speed_I=0;
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
  
  if(speed_inter>300)
    speed_inter=300;
  if(speed_inter<-300)
    speed_inter=-300;
  
  speed_P_out=speed_err*speed_P;
  speed_I_out=speed_inter*speed_I;
  speed_D_out=(speed_err-speed_last_err)*speed_D;
  speed_last_err=speed_err;  
  return speed_P_out+speed_I_out+speed_D_out;
}

/*************************************************位置pid***********************************************/


float position_err,position_inter,position_last_err;
float position_P_out,position_I_out,position_D_out;

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
  if(time_1ms_cnt % 80== 0 )
  {
      //send_wave(target_speed,now_speed,0,0);
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
    uint16_t data;
    data = spi_read_encoder();
    //uprintf("data = %d\r\n",data);
    now_position = (float)data*2*3.14/1024;
    //uprintf("now_position = %f\r\n",now_position);
    
    /*****************增量式编码器***************/
    /*
    int ss=TIM2->CNT;
    now_speed=ss-32767;
    now_position+=now_speed;
    //flash_data[7]=now_position;
    //write_prams();
    TIM2->CNT=32767;
    */
}

/****************************调速度，速度环，位置环*************************/
//本来放在时钟滴答里面，但中断处理函数太多系统会崩溃，采取滴答中断改变标志位的方法，外置处理函数
void set_speed_position()
{
  if(time1_flag==1)//计时器1
  {
    read_speed_position();
    
    /*
    //往主机发实际位置
    can_TX_data->in=(uint16_t)now_position;
    can_led(BLUE);
    can_send_msg(flash_data[0], can_TX_data->ch, 2);
    can_led(GREEN);
    */
    
    /*
    target_position=target_angle*angle_K+angle_B;
    //uprintf("target_angle= %lf\r\n\r\n",target_angle);
    //uprintf("target_position= %f\r\n\r\n",target_position);
    
    //send_wave((float)0,(float)0,(float)now_position,(float)target_position);
    if(position_mode==1)//位置环
    {*/
      int position_out=(int)position_PID_Control(target_position,now_position);
      set_speed(position_out);
      if((target_position < 3.5 && target_position - now_position < 1 && target_position - now_position > -1) //放开
         ||(target_position > 3.5 && target_position - now_position < 0.15 && target_position - now_position > -0.15))//抓紧
      {
          if(first_send_can_msg == 0)
          {
              can_send_msg(320,"3", 1);//发给爪臂
              first_send_can_msg = 1;
          }
      }
      else
      {
          first_send_can_msg = 0;
      }
      //send_wave((float)position_out,(float)now_speed,(float)now_position,(float)target_position);
      time1_flag=0;
      /*
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
    
    time1_flag=0;
  
    //send_wave((float)now_speed,(float)target_speed,(float)now_position,(float)target_position);
    */
  }
}

void buffer_position_ctrl()
{
    if(buf_flag == 0) return ;
    if(buf_position - target_position >0.087)//5度
    {
        target_position += 0.087;
    }
    else if(buf_position - target_position < -0.087)
    {
        target_position -= 0.087;
    }
    else
    {
        target_position = buf_position;
    }
    buf_flag = 0;
}