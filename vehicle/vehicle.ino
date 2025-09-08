#include <Ticker.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

BluetoothSerial SerialBT;

//定义引脚名称
#define PWMA 12  //3为模拟引脚，用于PWM控制
#define AIN1 14
#define AIN2 27
#define PWMB 13  //10为模拟引脚，用于PWM控制
#define BIN1 33
#define BIN2 32
#define STBY 26  
#define Voltage 34 

#define ENCODER_L   19   // 左轮 A 相
#define DIRECTION_L 22   // 左轮 B 相
#define ENCODER_R   18   // 右轮 A 相
#define DIRECTION_R 23   // 右轮 B 相

int PwmA, PwmB;
double vol;
char msg;
char lastmsg=' ';
const float WHEEL_DIAMETER = 0.045f;   // 轮径 (m) = 4.5 cm
const float WHEEL_BASE     = 0.160f;   // 轮距 (m) —— 请用实测值替换！
const int   COUNTS_PER_REV = 600;      // 每转计数（按当前二倍频方式得到的“每转中断次数”）

// ====== 运行参数 ======
const float SAMPLE_T = 0.05f; // 定时器采样周期 10ms

// ====== 变量 ======
volatile long cntL = 0, cntR = 0;      // 累计脉冲（ISR里修改）
int   pulsesL = 0, pulsesR = 0;         // 10ms 内脉冲
float rpsL = 0, rpsR = 0;               // 每轮转速 rev/s
float vL = 0, vR = 0;                   // 每轮线速度 m/s
float v = 0, w = 0;                     // 车体线速度 m/s 与角速度 rad/s

Ticker timer0;   // 用 Ticker 替代硬件定时器

void onTimer() {
  pulsesL = cntL;  cntL = 0;
  pulsesR = cntR;  cntR = 0;

  // rev/s
  rpsL = (float)pulsesL / (COUNTS_PER_REV * SAMPLE_T);
  rpsR = (float)pulsesR / (COUNTS_PER_REV * SAMPLE_T);

  // 每轮线速度：v = rps * (π * D)
  const float C = (float)M_PI * WHEEL_DIAMETER;
  vL = rpsL * C;
  vR = rpsR * C;

  // 差速车学模型
  v = 0.5f * (vR + vL);
  w = (vR - vL) / WHEEL_BASE;  // rad/s
}


// ====== 编码器外部中断（A 相双边沿，B 相判相）======
void IRAM_ATTR READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {
    if (digitalRead(DIRECTION_L) == LOW) cntL--; else cntL++;
  } else {
    if (digitalRead(DIRECTION_L) == LOW) cntL++; else cntL--;
  }
}
void IRAM_ATTR READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) {
    if (digitalRead(DIRECTION_R) == LOW) cntR++; else cntR--;
  } else {
    if (digitalRead(DIRECTION_R) == LOW) cntR--; else cntR++;
  }
}

void setup() {
  //TB6612电机驱动模块控制信号初始化
  SerialBT.begin("vehicle");  // 从机名称
  pinMode(AIN1, OUTPUT);//控制电机A的方向，(AIN1, AIN2)=(1, 0)为正转，(AIN1, AIN2)=(0, 1)为反转
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);//控制电机B的方向，(BIN1, BIN2)=(0, 1)为正转，(BIN1, BIN2)=(1, 0)为反转
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);//A电机PWM
  pinMode(PWMB, OUTPUT);//B电机PWM
  pinMode(STBY, OUTPUT);//TB6612FNG使能, 置0则所有电机停止, 置1才允许控制电机
  pinMode(ENCODER_L,   INPUT);
  pinMode(DIRECTION_L, INPUT);
  pinMode(ENCODER_R,   INPUT);
  pinMode(DIRECTION_R, INPUT);
  pinMode(Voltage,INPUT); //初始化作为输入端
  
  //初始化TB6612电机驱动模块
  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

 //初始化串口，用于输出电池电压
  Serial.begin(115200);

  attachInterrupt(ENCODER_L, READ_ENCODER_L, CHANGE);
  attachInterrupt(ENCODER_R, READ_ENCODER_R, CHANGE);



  Serial.println("Ticker 定时器启动成功!");
}


void SetPWM(int motor, int pwm)//因为一些神奇的原因调试不好。导致最终只能AB管脚反着接，于是在这个函数这里再反一下，就对应了
{
  if(motor==2&&pwm>=0)
  {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, pwm);
  }
  else if(motor==2&&pwm<0)
  {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, -pwm);
  }
  else if(motor==1&&pwm>=0)
  {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, pwm);
  }
  else if(motor==1&&pwm<0)
  {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, -pwm);
  }
}

void loop() 
{
  
  if (SerialBT.available()) {
    msg = SerialBT.read();
    Serial.println(msg);
    if(msg=='0'){
      SetPWM(1, 70);
      SetPWM(2, 70);
    }
    if(msg=='1'){
        SetPWM(1, 0);
        SetPWM(2, 0);
    }
    if(msg=='3'){
      SetPWM(1,40);
      SetPWM(2,70);
    }
    if(msg=='2'){
      SetPWM(1,70);
      SetPWM(2,40);
    }
    lastmsg=msg;
  }

 // vol=analogRead(Voltage); //读取模拟引脚A0模拟量
  //Serial.print(vol*0.05371);  //对模拟量转换并通过串口输出
  //Serial.println("vol");
  //delay(500);//正转3s
  
  //Serial.print("vL=");
  //Serial.print(vL, 4);
  //Serial.print(" m/s  vR=");
  //Serial.print(vR, 4);
  //Serial.print(" m/s  |  v=");
  //Serial.print(v, 4);
  //Serial.print(" m/s  w=");
  //Serial.print(w, 4);
  //Serial.println(" rad/s");

  //Serial.print("cntL=");
  //Serial.println(cntL);
  //Serial.print("cntR=");
  //Serial.println(cntR);
  delay(100);
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
//  
//  SetPWM(1, 128);//电机AB同时半速正转
//  SetPWM(2, 128);
//  delay(3000);//半速正转3s
//  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
//  
//  SetPWM(1, -255);//电机AB同时满速反转
//  SetPWM(2, -255);
//  delay(3000);//反转3s
//  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
//  
//  SetPWM(1, 255);//电机A满速正转
//  SetPWM(2, -255);//电机B满速反转
//  delay(3000);//持续3s
//  
//  SetPWM(1, 0);//电机AB停止
//  SetPWM(2, 0);
//  delay(1000);//停止1s
}
