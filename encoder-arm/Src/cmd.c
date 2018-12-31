#include "cmd.h"


/*
  *�������Ľṹ��
  * ������������Ҫ�ڴ˼��ϣ�
  * CMD_ADD("������","����ʹ�÷�������Ϊ�ո�,�����ܲ���˫���ţ�",��Ӧ�����ִ�к�����)
  * ע�����һ������Ҫ���ţ�ǰ��Ķ���Ҫ����
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
    cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);//��ȷ���������ݵ��ڴ�ռ䣬���Է���һ��
  }
}
/*
*���������
*/
int cmd_parse(char *cmd_line,int *argc,char *argv[]){
  char c_temp;
  int i = 0,arg_index = 0;
  int arg_cnt = 0;
  c_temp = cmd_line[i++];  
  while(c_temp != '\r'){
    if(c_temp == ' '){
      if(arg_index == 0){   //���������߲����ַ�����һ���ǿո������   
        c_temp = cmd_line[i++];
        continue;
      }
      //�ո�Ϊ������������ķָ���
      if(arg_cnt == MAX_ARGC){   //���������������,�򷵻�
        return -1;
      }
      argv[arg_cnt][arg_index] = 0;
      arg_cnt++;
      arg_index = 0;
      c_temp = cmd_line[i++];
      continue;
    }
    if(arg_index == MAX_CMD_ARG_LENGTH){   //����������ȹ������򱨴���
      return -2;
    }
    argv[arg_cnt][arg_index++] = c_temp;
    c_temp = cmd_line[i++];
  }
  if(arg_cnt == 0 && arg_index == 0){  //���������߲����ǿյģ��򷵻�
    return -3;
  }
  //���һ�������Ľ���û���������whileѭ���н�����
  argv[arg_cnt++][arg_index] = 0;
  *argc = arg_cnt;//������
  return 0;
}

int cmd_exec(int argc,char *argv[]){
  int cmd_index = 0;
  uint32_t cmd_num;
  
  cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
  
  if(argc == 0){  //��������ǿյģ��򷵻�
    return -1;
  }
  for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //��������
    if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //����ҵ��������ִ���������Ӧ�ĺ���
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
    uprintf("msg:\n help�����������\r\n\r\n");      
    return;         
  }
  for(i = 0;i < cmd_num;i++){
    uprintf("cmd:%s\r\n",cmd_tbl[i].cmd_name);
    uprintf("usage:%s\r\n\r\n",cmd_tbl[i].cmd_usage);
  }
}
