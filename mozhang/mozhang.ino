#include "mpu6050.h"
#include <math.h>
#include <string.h>
#include "classifier.h"
#include "BluetoothSerial.h"
#include "esp_bt_device.h"
#define PI 3.1415926

const int LED_PIN=2;

BluetoothSerial SerialBT;
Eloquent::ML::Port::SVM clf;//实例化模型

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0;  // used to control display output rate

float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
float GyroMeasDrift = PI * (2.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
float deltat = 0.0f;                              // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;         // used to calculate integration interval
uint32_t Now = 0;  
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; //四元数  

float pitch, yaw, roll;//三轴角
float worldAccX,worldAccY,worldAccZ;//世界坐标系下三轴加速度

const int buttonPin=13;//按钮GPIO13
int buttonstate=0;//按钮状态
int lastbuttonstate=0;//上一次按钮状态
int i=0;//下标
float x_acc[50];//加速度
float y_acc[50]; 
float z_acc[50];
float x_v[50];//速度
float y_v[50]; 
float z_v[50];
float x_x[50];//位移
float y_x[50]; 
float z_x[50];
int k;//工具变量
float input[100]={0};//用于跑模型的变量
int pred=0;//预测结果

float sumwhat(int i,float tool[50]){//辅助函数求tool数组的前i项和
  int j;
  float sum=0.0;
  for (j=0;j<=i;j++){
    sum+=tool[j]*0.05;
  }
  return sum;
}

unsigned long starttime=0;//开始时间
int flag1=0;//为了标志2s已到但是按钮未松开的情况的flag
int flag2=1;//为了标志开始计时的flag

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;
  
    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;
    
    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;
    
    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;
    
    // Compute and remove gyroscope biases
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gyrox -= gbiasx;
    gyroy -= gbiasy;
    gyroz -= gbiasz;
    
    // Compute the quaternion derivative
    qDot1 = -_halfq2 * gyrox - _halfq3 * gyroy - _halfq4 * gyroz;
    qDot2 =  _halfq1 * gyrox + _halfq3 * gyroz - _halfq4 * gyroy;
    qDot3 =  _halfq1 * gyroy - _halfq2 * gyroz + _halfq4 * gyrox;
    qDot4 =  _halfq1 * gyroz + _halfq2 * gyroy - _halfq3 * gyrox;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(beta * hatDot1)) * deltat;
    q2 += (qDot2 -(beta * hatDot2)) * deltat;
    q3 += (qDot3 -(beta * hatDot3)) * deltat;
    q4 += (qDot4 -(beta * hatDot4)) * deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Init_mpu6050();
  pinMode(buttonPin,INPUT_PULLUP);
  SerialBT.begin("ESP32_BT",true);
  uint8_t macAddr[6]={0x84, 0x1F, 0xE8, 0x15, 0xBE, 0x4E};

  if (SerialBT.connect(macAddr)) {
    Serial.println("Connected to Slave!");
    for (k=0;k<3;k++) {//蓝牙连接成功提升:灯闪三下
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(300);
    }
  } else {
    Serial.println("Failed to connect to Slave.");
    digitalWrite(LED_PIN, HIGH);//蓝牙连接失败提示:灯一直亮
  }
}

void loop() {
  buttonstate=digitalRead(buttonPin);
  ReadMPU6050();
  //可视化线加速度曲线
  //Serial.print("Acc_x:");
  //Serial.print(mpu6050_data.Acc_X);
  //Serial.print(",");
  //Serial.print("Acc_Y:");
  //Srial.print(mpu6050_data.Acc_Y);
  //Serial.print(",");
  //Serial.print("Acc_Z:");
  //Serial.println(mpu6050_data.Acc_Z);

  //可视化角速度曲线
  //Serial.print("Angle_velocity_R:");
  //Serial.println(mpu6050_data.Angle_Velocity_R);
  //Serial.print(",");
  //Serial.print("Angle_velocity_P:");
  //Serial.print(mpu6050_data.Angle_Velocity_P);
  //Serial.print(",");
  //Serial.print("Angle_velocity_Y:");
  //Serial.println(mpu6050_data.Angle_Velocity_Y);

  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  MadgwickQuaternionUpdate(mpu6050_data.Acc_X/9.8065, mpu6050_data.Acc_Y/9.8065, mpu6050_data.Acc_Z/9.8065, mpu6050_data.Angle_Velocity_R, mpu6050_data.Angle_Velocity_P, mpu6050_data.Angle_Velocity_Y);
  delt_t = millis() - count;
  if (delt_t > 50) { // update LCD once per half-second independent of read rate


  yaw=atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch=-asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll=atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

  //Serial.print(yaw, 2);
  //Serial.print(", ");
  //Serial.print(pitch, 2);
  //Serial.print(", ");
  //Serial.println(roll, 2);

  float R[3][3];
  R[0][0]=cos(pitch)*cos(yaw);
  R[0][1]=cos(yaw)*sin(roll)*sin(pitch)-cos(roll)*sin(yaw);
  R[0][2]=sin(roll)*sin(yaw)+cos(roll)*cos(yaw)*sin(pitch);

  R[1][0]=cos(pitch)*sin(yaw);
  R[1][1]=cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
  R[1][2]=cos(roll)*sin(pitch)*sin(yaw)-cos(yaw)*sin(roll);

  R[2][0]=-sin(pitch);
  R[2][1]=cos(pitch)*sin(roll);
  R[2][2]=cos(roll)*cos(pitch);

  worldAccX=R[0][0]*mpu6050_data.Acc_X+R[0][1]*mpu6050_data.Acc_Y+R[0][2]*mpu6050_data.Acc_Z;
  worldAccY=R[1][0]*mpu6050_data.Acc_X+R[1][1]*mpu6050_data.Acc_Y+R[1][2]*mpu6050_data.Acc_Z;
  worldAccZ=R[2][0]*mpu6050_data.Acc_X+R[2][1]*mpu6050_data.Acc_Y+R[2][2]*mpu6050_data.Acc_Z;

  // 减去重力
  worldAccZ-=9.8065;

  //Serial.print(worldAccX, 2);
  //Serial.print(", ");
  //Serial.print(worldAccY, 2);
  //Serial.print(", ");
  //Serial.println(worldAccZ, 2);
  if(buttonstate==LOW&&flag1==0){//如果按下按钮
    if(lastbuttonstate==HIGH){//首次按下按钮
      i=0;//下标更新
      starttime=millis();//起始时间更新
      flag2=0;//开始计时标志
      memset(x_x,0,sizeof(x_x));
      memset(y_x,0,sizeof(y_x));
      memset(z_x,0,sizeof(z_x));
      Serial.println("首次按下");
    } 
    x_acc[i]=worldAccX;//更新加速度
    y_acc[i]=worldAccY;
    z_acc[i]=worldAccZ;
    i++;
  }
  
  if((millis()-starttime>=2000||(lastbuttonstate==LOW&&buttonstate==HIGH))&&flag2==0){
    flag1=1;//关闭记录通道
    flag2=1;//关闭积分运算通道
    //积分运算
    Serial.println("进入积分");
    for (k=0;k<i;k++){
      x_v[k]=sumwhat(k,x_acc);
      y_v[k]=sumwhat(k,y_acc);
      z_v[k]=sumwhat(k,z_acc);
    }
    for (k=0;k<i;k++){
      x_x[k]=sumwhat(k,x_v);
      y_x[k]=sumwhat(k,y_v);
      z_x[k]=sumwhat(k,z_v);
    }
    for (k=0;k<50;k++){
      input[k]=x_x[k];
      input[k+50]=z_x[k];
    }
    pred=clf.predict(input);
    Serial.println("===START===");   // 一次动作开始
    for (k=0;k<i;k++){
      Serial.print("x:");
      Serial.print(x_x[k]);
      Serial.print(",y:");
      Serial.print(y_x[k]);
      Serial.print(",z:");
      Serial.println(z_x[k]);
      }
    Serial.println(pred);
    Serial.println("===END===");     // 一次动作结束
    SerialBT.println(pred);
  }
  if(buttonstate==HIGH){flag1=0;}//如果按钮是松开的允许再按记录，如果是按到时间到了结束，不触发flag1变化，不进记录
  lastbuttonstate=buttonstate;//记录上一次按钮状态

  count = millis();  


}

}