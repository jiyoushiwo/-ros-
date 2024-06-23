/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //以下引脚可以更改，但应为PORTD引脚；否则需要在代码中进行其他更改
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //以下引脚可以更改，但应为PORTC引脚
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined ARDUINO_MY_COUNTER
//1.定义引脚
  #define LEFT_A 21//中断2
  #define LEFT_B 20//中断3
  #define RIGHT_A 18//中断5
  #define RIGHT_B 19//中断4
//声明函数
  //2.初始化函数，设置引脚操作模式，并且添加中断
  void initEncoders();
  //3.中断函数
  void leftEncoderEventA();
  void leftEncoderEventB();
  void rightEncoderEventA();
  void rightEncoderEventB();
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
