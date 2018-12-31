#ifndef __cmd_func_H
#define __cmd_func_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "main.h"
#include "usart.h"
#include "cmd.h"
#include "stdlib.h"
#include "math.h"
#define PI 3.1415926535
     
void cmd_hello_func(int argc,char *argv[]);
   
void cmd_speed_func(int argc,char *argv[]);
void cmd_speed_mode_func(int argc,char *argv[]);
void cmd_speed_pid_func(int argc,char *argv[]);

void cmd_position_func(int argc,char *argv[]);
void cmd_position_mode_func(int argc,char *argv[]);
void cmd_position_pid_func(int argc,char *argv[]);

void cmd_angle_KB_func(int argc,char *argv[]);
void cmd_target_angle_func(int argc,char *argv[]);

void cmd_write_flash_func(int argc,char *argv[]);
void cmd_read_flash_func(int argc,char *argv[]);
void cmd_read_pos_func(int argc,char *argv[]);
void cmd_time_interval_func(int argc,char *argv[]);
   
void cmd_set_arm_position_func(int argc,char *argv[]);
void cmd_arm_run_func(int argc,char *argv[]);
void cmd_set_param_func(int argc,char *argv[]);

#ifdef __cplusplus
}
#endif
#endif /*__ cmd_func_H */
