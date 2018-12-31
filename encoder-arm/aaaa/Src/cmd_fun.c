
#include "cmd_fun.h"
#include "usart.h"
#include "command.h"
#include "control.h"

uint8_t send_flag=0;

typedef struct{
  char * var_name;
  void * value_ptr;
}Var_Edit_Struct;



uint8_t First_Time_Check;
Var_Edit_Struct Var_List[10]={
  {"first",&First_Time_Check},

};


void set_val(int arg_num,char ** s,float * args){
  void * edit_value;
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }

  for(int i=0;i<sizeof(Var_List)/sizeof(Var_Edit_Struct);++i){
    if(compare_string(Var_List[i].var_name,s[0])){
      edit_value=Var_List[i].value_ptr;
      break;
    }
  }
  
  if(compare_string(s[1],"u8")){
    *(uint8_t *)edit_value=(uint8_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(uint8_t *)edit_value);  
  }else if(compare_string(s[1],"int")){
    *(int16_t *)edit_value=(int16_t)args[0];
    uprintf("ok set %s = %d\r\n",s[0],*(int16_t *)edit_value);
  }else if(compare_string(s[1],"f")){
    *(float *)edit_value=args[0];
    uprintf("ok set %s = %f\r\n",s[0],*(float *)edit_value);
  }
}

void set_speed(int arg_num,char **s,float * args){
  if (arg_num!=0x0001){
    uprintf("error arg_num!\r\n");
    return ;
  }
  Set_Speed(args[0]);
}
