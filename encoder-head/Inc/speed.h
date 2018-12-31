#ifndef __speed_H
#define __speed_H
#ifdef __cplusplus
 extern "C" {
#endif
 
extern int speed_mode;
extern int position_mode;
extern int time1_flag;

   
extern float target_speed;
extern float now_speed;
extern float target_position;
extern float now_position;

extern float target_angle;
extern float angle_K;
extern float angle_B;
   
extern float speed_P;
extern float speed_I;
extern float speed_D;

void set_speed(int out);
void init_speed_PID(void);
float speed_PID_Control(float target_speed,float now_speed);

extern uint8_t readBuffer[2];
extern uint8_t writeBuffer1[2];
extern uint8_t writeBuffer2[2];


extern float position_P;
extern float position_I;
extern float position_D;
extern float position_inter;
     
extern int left_bgn_flag;
extern int right_bgn_flag;

void init_position_PID(void);
float position_PID_Control(float target_position,float now_position);
void set_speed_position();
void arm_ctrl();

#ifdef __cplusplus
}
#endif
#endif /*__ speed_H */