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

//צ�ӵ�pidΪ 5500 0 0

//צ�۵�pidΪ 3300 0 11000���ѳݣ�δ��װ�չǣ���ʱp��Ҫ����צ�ӵ�����Ҫ����ʱ
/*read_pos������Զ�ȡ������λ��
position ��ֵ ���Զ���target_position
*/
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

/***********************************************�ٶ�pid*******************************************/
float speed_P=0;
float speed_I=0;
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

/*************************************************λ��pid***********************************************/


float position_err,position_inter,position_last_err;
float position_P_out,position_I_out,position_D_out;

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
  if(time_1ms_cnt % 80== 0 )
  {
      //send_wave(target_speed,now_speed,0,0);
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
    uint16_t data;
    data = spi_read_encoder();
    //uprintf("data = %d\r\n",data);
    now_position = (float)data*2*3.14/1024;
    //uprintf("now_position = %f\r\n",now_position);
    
    /*****************����ʽ������***************/
    /*
    int ss=TIM2->CNT;
    now_speed=ss-32767;
    now_position+=now_speed;
    //flash_data[7]=now_position;
    //write_prams();
    TIM2->CNT=32767;
    */
}

/****************************���ٶȣ��ٶȻ���λ�û�*************************/
//��������ʱ�ӵδ����棬���жϴ�����̫��ϵͳ���������ȡ�δ��жϸı��־λ�ķ��������ô�����
void set_speed_position()
{
  if(time1_flag==1)//��ʱ��1
  {
    read_speed_position();
    
    /*
    //��������ʵ��λ��
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
    if(position_mode==1)//λ�û�
    {*/
      int position_out=(int)position_PID_Control(target_position,now_position);
      set_speed(position_out);
      if((target_position < 3.5 && target_position - now_position < 1 && target_position - now_position > -1) //�ſ�
         ||(target_position > 3.5 && target_position - now_position < 0.15 && target_position - now_position > -0.15))//ץ��
      {
          if(first_send_can_msg == 0)
          {
              can_send_msg(320,"3", 1);//����צ��
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
    
    else if(speed_mode==1)//�ٶȻ�
    {
      int speed_out=(int)speed_PID_Control(target_speed,now_speed);
      set_speed(speed_out);   
    }
    
    else if(speed_mode==0)//ֱ�����ٶȲ����ٶȻ���λ�û�
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
    if(buf_position - target_position >0.087)//5��
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