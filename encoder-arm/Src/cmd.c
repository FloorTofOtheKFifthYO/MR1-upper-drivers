#include "cmd.h"


/*
  *存放命令的结构体
  * 若需添加命令，需要在此加上：
  * CMD_ADD("命令名","命令使用方法（可为空格,但不能不加双引号）",对应命令的执行函数名)
  * 注意最后一个不需要逗号，前面的都需要逗号
  */
static cmd_struct cmd_tbl[] = {
  CMD_ADD("help"," Print all command and usage ",cmd_help_func),
  CMD_ADD("hello"," ",cmd_hello_func),
  
  CMD_ADD("speed","set target speed",cmd_speed_func),
  CMD_ADD("speed_mode","begin to set the speed loop",cmd_speed_mode_func),
  CMD_ADD("speed_pid","speed_pid KP KI KD",cmd_speed_pid_func),
  
  CMD_ADD("position","set target position",cmd_position_func),
  CMD_ADD("interval","set time_interval",cmd_time_interval_func),
  CMD_ADD("position_mode","begin to set the position loop",cmd_position_mode_func),
  CMD_ADD("position_pid","position_pid KP KI KD",cmd_position_pid_func),
  CMD_ADD("read_pos","read pos",cmd_read_pos_func),
  
  CMD_ADD("write_flash","write CAN ID to flash",cmd_write_flash_func),
  CMD_ADD("read_flash","read CAN ID from flash",cmd_read_flash_func),
    
  CMD_ADD("angle_KB"," ",cmd_angle_KB_func),
  CMD_ADD("target_angle"," ",cmd_target_angle_func),
  CMD_ADD("set_position","aa88a",cmd_set_arm_position_func),
  CMD_ADD("arm_run","aaa",cmd_arm_run_func),
  CMD_ADD("set_param","asd",cmd_set_param_func)
};

char cmd_line[MAX_CMD_LINE_LENGTH + 1];
char *cmd_argv[MAX_ARGC]; 

void cmd_init()
{
  for(int i = 0;i < MAX_ARGC;i++){
    cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);//不确定输入数据的内存空间，所以分配一块
  }
}
/*
*解析命令函数
*/
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
  char c_temp;
  int i = 0,arg_index = 0;
  int arg_cnt = 0;
  c_temp = cmd_line[i++];  
  while(c_temp != '\r'){
    if(c_temp == ' '){
      if(arg_index == 0){   //如果命令或者参数字符串第一个是空格，则忽略   
        c_temp = cmd_line[i++];
        continue;
      }
      //空格为参数或者命令的分隔符
      if(arg_cnt == MAX_ARGC){   //如果参数个数过多,则返回
        return -1;
      }
      argv[arg_cnt][arg_index] = 0;
      arg_cnt++;
      arg_index = 0;
      c_temp = cmd_line[i++];
      continue;
    }
    if(arg_index == MAX_CMD_ARG_LENGTH){   //如果参数长度过长，则报错返回
      return -2;
    }
    argv[arg_cnt][arg_index++] = c_temp;
    c_temp = cmd_line[i++];
  }
  if(arg_cnt == 0 && arg_index == 0){  //如果命令或者参数是空的，则返回
    return -3;
  }
  //最后一个参数的结束没有在上面的while循环中解析到
  argv[arg_cnt++][arg_index] = 0;
  *argc = arg_cnt;//命令数
  return 0;
}

int cmd_exec(int argc,char *argv[]){
  int cmd_index = 0;
  uint32_t cmd_num;
  
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  
  if(argc == 0){  //如果参数是空的，则返回
    return -1;
  }
  for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //查找命令
    if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //如果找到了命令，则执行命令相对应的函数
      cmd_tbl[cmd_index].cmd_func(argc,argv);
      memset(USART_RX_BUF,0,MAX_CMD_LINE_LENGTH + 1);
      return 0;
    }
  }
  return -2;
}

void cmd_help_func(int argc,char *argv[]){
  int i;
  uint32_t cmd_num;
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  if(argc > 1){
    uprintf("msg:\n help命令参数过多\r\n\r\n");      
    return;         
  }
  for(i = 0;i < cmd_num;i++){
    uprintf("cmd:%s\r\n",cmd_tbl[i].cmd_name);
    uprintf("usage:%s\r\n\r\n",cmd_tbl[i].cmd_usage);
  }
}
