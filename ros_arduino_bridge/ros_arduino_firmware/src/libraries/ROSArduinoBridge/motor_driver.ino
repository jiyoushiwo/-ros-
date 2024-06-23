/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef POLOLU_VNH5019
  /* Include the Pololu library */
  #include "DualVNH5019MotorShield.h"

  /* Create the motor driver object */
  DualVNH5019MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 150)
      spd = ;
    
    if (i == LEFT) { 
      if      (reverse == 0) { analogWrite(RIGHT_MOTOR_FORWARD, spd); analogWrite(RIGHT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(RIGHT_MOTOR_BACKWARD, spd); analogWrite(RIGHT_MOTOR_FORWARD, 0); }
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { analogWrite(LEFT_MOTOR_FORWARD, spd); analogWrite(LEFT_MOTOR_BACKWARD, 0); }
      else if (reverse == 1) { analogWrite(LEFT_MOTOR_BACKWARD, spd); analogWrite(LEFT_MOTOR_FORWARD, 0); }
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

#elif defined L298P_MOTOR_DRIVER

//1.初始化
  void initMotorController(){
    pinMode(AIN1,OUTPUT);
    pinMode(AIN2,OUTPUT);
    pinMode(PWMA,OUTPUT);
    
    pinMode(STBY,OUTPUT);
    
    pinMode(BIN1,OUTPUT);
    pinMode(BIN2,OUTPUT);
    pinMode(PWMB,OUTPUT);
  }

  //2.设置单电机转速
  void setMotorSpeed(int i, int spd){
    unsigned char reverse = 0;
// 如果速度小于0，则取反并设置反转标志
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
      // 将速度限制在0-255之间
    if (spd > 150)
      spd = 150;
      
      //tiaoshi
    
    if (i == LEFT) { 
      digitalWrite(STBY, HIGH); // 启用电机驱动器
      if (reverse == 0) { //左电机
        
        digitalWrite(AIN1, LOW);//正转
        digitalWrite(AIN2, HIGH);
      } else if (reverse == 1) { 
        digitalWrite(AIN1, HIGH);//反转
        digitalWrite(AIN2, LOW);
      }
      analogWrite(PWMA,spd);// 设置电机速度
    } else if (i == RIGHT){ // 右电机
      digitalWrite(STBY, HIGH);
      if (reverse == 0) { // 正转
        
        digitalWrite(BIN1, LOW); // 设置电机反转
        digitalWrite(BIN2, HIGH);        
      } else if (reverse ==1) { // 反转
        digitalWrite(BIN1, HIGH); // 设置电机正转
        digitalWrite(BIN2, LOW);
      }
      analogWrite(PWMB, spd); // 设置电机速度
    }
  }

  //3.设置两个电机转速
  void setMotorSpeeds(int leftSpeed, int rightSpeed){
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }

  
#else
  #error A motor driver must be selected!
#endif

#endif
