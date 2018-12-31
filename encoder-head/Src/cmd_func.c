#include "cmd_func.h"
#include "speed.h"
#include "flash.h"
#include "can.h"
#include "gpio.h"


void cmd_hello_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  uprintf("hello world");
}

//speed 1
void cmd_speed_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  target_speed=atof(argv[1]);
}

//speed_mode 1
void cmd_speed_mode_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  speed_mode=atoi(argv[1]);
}

//speed_pid 1 2 3
void cmd_speed_pid_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  speed_P=atof(argv[1]);
  speed_I=atof(argv[2]);
  speed_D=atof(argv[3]);
  flash_data[1]=speed_P;
  flash_data[2]=speed_I;
  flash_data[3]=speed_D;
  write_prams();
  uprintf("speed_pid=%f %f %f\r\n",speed_P,speed_I,speed_D);
}

//position 1
void cmd_position_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  target_position =atof(argv[1]);
  uprintf("target_position=%f\r\n",target_position);
}

//position_mode 1
void cmd_position_mode_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  position_mode=atoi(argv[1]);
}

//position_pid 1 2 3
void cmd_position_pid_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  position_P=atof(argv[1]);
  position_I=atof(argv[2]);
  position_D=atof(argv[3]);
  flash_data[4]=position_P;
  flash_data[5]=position_I;
  flash_data[6]=position_D;
  position_inter = 0;
  write_prams();
  uprintf("pos_pid=%f %f %f\r\n",position_P,position_I,position_D);
}

//angle_KB 1 2
void cmd_angle_KB_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  angle_K=atof(argv[1]);
  angle_B=atof(argv[2]);
  flash_data[7]=angle_K;
  flash_data[8]=angle_B;
  write_prams();
}

//target_angle 1
void cmd_target_angle_func(int argc,char *argv[])
{
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  target_angle=atof(argv[1]);
}

//write_flash LF
void cmd_write_flash_func(int argc,char *argv[])
{
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
  /**********◊Û«∞Õ»************/
  if(strcmp(argv[1],"LF_side")==0)
  {
    flash_data[0]=0;
  }
  else if(strcmp(argv[1],"LF_hip")==0)
  {
    flash_data[0]=1;
  }
  else if(strcmp(argv[1],"LF_knee")==0)
  {
    flash_data[0]=2;
  }
  /**********”“«∞Õ»************/
  if(strcmp(argv[1],"RF_side")==0)
  {
    flash_data[0]=3;
  }
  else if(strcmp(argv[1],"RF_hip")==0)
  {
    flash_data[0]=4;
  }
  else if(strcmp(argv[1],"RF_knee")==0)
  {
    flash_data[0]=5;
  }
  /**********◊Û∫ÛÕ»************/
  if(strcmp(argv[1],"LB_side")==0)
  {
    flash_data[0]=6;
  }
  else if(strcmp(argv[1],"LB_hip")==0)
  {
    flash_data[0]=7;
  }
  else if(strcmp(argv[1],"LB_knee")==0)
  {
    flash_data[0]=8;
  }
  /**********”“∫ÛÕ»************/
  if(strcmp(argv[1],"RB_side")==0)
  {
    flash_data[0]=9;
  }
  else if(strcmp(argv[1],"RB_hip")==0)
  {
    flash_data[0]=10;
  }
  else if(strcmp(argv[1],"RB_knee")==0)
  {
    flash_data[0]=11;
  }
  
  write_prams();
  can_init();
}
          
//read_flash
void cmd_read_flash_func(int argc,char *argv[])
{
  load_prams();
}

          