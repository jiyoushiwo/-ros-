/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13

#elif defined L298P_MOTOR_DRIVER

  //HL正转，LH反转
  #define AIN1 9
  #define AIN2 8
  #define PWMA 3
  #define STBY 10

  //第二个电机，待测
  #define BIN1 7
  #define BIN2 6
  #define PWMB 5
#endif

void initMotorController();//初始化电机控制
void setMotorSpeed(int i, int spd);//设置单个电机转速
void setMotorSpeeds(int leftSpeed, int rightSpeed);//设置多个电机转速
