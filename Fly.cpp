#include <Wire.h>
#include <Arduino.h>
#include <CurieIMU.h>
#include <CurieBLE.h>
#include <CurieTimerOne.h>
#include <MadgwickAHRS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <gp20u7.h>

#include "Fly.h"

// 比例、积分、微分控制器
class PIDController {
  private:
    float ErrPrev, ErrLast, ErrDiff;// 上一次误差， 这一次误差，误差微分
    uint16_t dT, cycleTimeFirst, cycleTimeSecond;// 时间差，当前时间，上次时间
  public:
    static float Kp, Ki, Kd;// 比例，积分，微分
    float Actual, Target, ErrIntg, Result;// 实际值，目标值，误差积分，输出值
    // PID控制器初始化
    PIDController() {
      cycleTimeFirst = millis();
      ErrPrev = ErrLast = ErrIntg = ErrDiff = 0;
      Actual = Target = Result = 0;
      cycleTimeSecond = cycleTimeFirst;
    }
    // PID计算输出
    void Compute(float Value) {
      // 更新时间
      cycleTimeFirst = millis();
      dT = (cycleTimeFirst - cycleTimeSecond) / 1000.0;
      cycleTimeSecond = cycleTimeFirst;
      // PID计算
      Actual = Value;
      ErrLast = Target - Actual;            // 误差  = 设定值 - 实际值
      ErrIntg += (dT * ErrLast);            // 误差积分 += 误差*dT
      ErrIntg = constrain(ErrIntg, MIN_INTG_ERROR, MAX_INTG_ERROR); // 积分限制
      ErrDiff = (ErrLast - ErrPrev) / dT;   // 误差微分 = (本次误差 - 上次误差)/dT
      ErrPrev = ErrLast;                    // 前一次误差 = 本次误差
      Result = Kp * ErrLast + Ki * ErrIntg + Kd * ErrDiff; // PID计算输出值
      Result = constrain(Result, MIN_MOTOR_ERROR, MAX_MOTOR_ERROR); // 输出限制
    }
};

float PIDController::Kp = CFG_Kp;
float PIDController::Ki = CFG_Ki;
float PIDController::Kd = CFG_Kd;

// 电机类
class Motor {
  private:
    int pin;      // 设置的引脚
    int speed;    // 速度
  public:
    Motor() {
      pin = 0;
      speed = 0;
    }
    // 设置引脚
    void setPin(int p) {
      pin = p;
      pinMode(pin, OUTPUT);
    }
    // 获取引脚
    int getPin() {
      return pin;
    }
    // 设置速度
    void setSpeed(int s) {
      speed = s;
    }
    // 获取速度
    int getSpeed() {
      return speed;
    }
    // 使用PWM向电调写入速度值
    void Write() {
      // 手动实现方式
      int angle = map(speed, 0, 360, 500, 2480);
      digitalWrite(pin, HIGH);
      delayMicroseconds(angle);
      digitalWrite(pin, LOW);
      delay(20 - speed / 2000);
        // pwm自动实现
//        analogWrite(pin, speed);
    }
    // 点击停止
    void Stop() {
      speed = 0;
      Write();
    }
};

// 超声波模块
class Sonar {
  private:
    int TrigPin; // 超声波发射引脚-2
    int EchoPin; // 超声波接收引脚-13
  public:
    float altitude; // 高度【cm】
    float offset; // 偏差【cm】，设定起始高度为0
    Sonar() {
      TrigPin = 2;
      EchoPin = 13;
      pinMode(TrigPin, OUTPUT);
      pinMode(EchoPin, INPUT);
      altitude = 0;
    }
    // 设置超声波模块
    void Setup(int trig, int echo) {
      Serial.println("[ Setup sonar... ]");
      TrigPin = trig;
      EchoPin = echo;
      pinMode(TrigPin, OUTPUT);
      pinMode(EchoPin, INPUT);
      altitude = 0;
      // 保证起始测距高度为0
      Compute();
      offset = altitude;
    }
    // 计算高度
    void Compute() {
      // 产生一个10us的高脉冲去触发TRIG_PIN
      digitalWrite(TrigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(TrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(TrigPin, LOW);
      // 检测脉冲宽度，并计算出距离
      altitude = pulseIn(EchoPin, HIGH) / 58.00 - offset;
    }
};

// 电子罗盘
class Compass {
  private:
    Adafruit_HMC5883_Unified mag; // 地磁传感器
  public:
    float headingDegrees;// 航向【度】，正北方位0度，按照东南西北顺序角度依次增加
    Compass() {
      mag = Adafruit_HMC5883_Unified(12345);
    }
    // 设置电子罗盘
    void Setup() {
      Serial.println("[ Setup compass... ]");
      // 启动电子罗盘
      if (!mag.begin())
      {
        Serial.println("no HMC5883 compass...");
        while (1);
      }
    }
    // 计算飞行器航向
    void Compute(void)
    {
      // 传感器事件
      sensors_event_t event;
      // 获取传感器事件
      mag.getEvent(&event);
      // 计算航向，弧度制
      float heading = atan2(event.magnetic.y, event.magnetic.x);
      float declinationAngle = 0.22;
      heading += declinationAngle;
      // 矫正航向到0-360之间
      if (heading < 0)
        heading += 2 * PI;
      if (heading > 2 * PI)
        heading -= 2 * PI;
      // 航向转角度制
      headingDegrees = heading * 180 / PI;
    }
};

// GPS模块
class GPS {
  private:
    GP20U7 gps = GP20U7(Serial);
    Geolocation currentLocation;// 当前GPS坐标
  public:
    double latitude, longitude;// 飞行器当前经纬度
    double end_lat, end_lon;//  飞行目标的经纬度
    double Result;// 飞行器需要顺时针转过的角度
    GPS() {
    }
    // SetupGPS
    void Setup() {
      Serial.println("[ Setup GPS... ]");
      gps.begin();
    }
    // 获取经纬度
    bool Compute() {
      if (gps.read()) {
        // 读取经纬度
        currentLocation = gps.getGeolocation();
        latitude = currentLocation.latitude;
        longitude = currentLocation.longitude;
        // 计算目标航向偏差
        double dx = end_lat - latitude;
        double dy = end_lon - longitude;
        int i;
        if (dx > 0 && dy >= 0)        //第一象限
          i = 0;
        else if (dx < 0 && dy > 0)    //第二象限
          i = 180;
        else if (dx < 0 && dy < 0)    //第三象限
          i = 180;
        else if (dx > 0 && dy < 0)    //第四象限
          i = 360;
        else i = 0;
        if (dx == 0 && dy < 0) {
          // printf("计算得出相差角度为:270 度\n");
          Result = 180;
        } else {
          Result = atan(dy / dx) * 180.0 / 3.1416 + i - 90;
          // printf("计算得出相差角度为:%.2lf 度\n", a);
        }
        return true;
      }
      return false;
    }
};

BLEService ledService("00001523-1212-efde-1523-785feabcd123"); // BLE LED Service
BLECharCharacteristic switchCharacteristic("00001527-1212-efde-1523-785feabcd123", BLEWrite);

class BlueTooth {
  private:
    BLEPeripheral blePeripheral;
  public:
    int p = 0;
    // 蓝牙初始化
    void Setup() {
      // set advertised local name and service UUID:
      blePeripheral.setLocalName("fly");
      blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

      // add service and characteristic:
      blePeripheral.addAttribute(ledService);
      blePeripheral.addAttribute(switchCharacteristic);

      // set the initial value for the characeristic:
      switchCharacteristic.setValue(0);

      // begin advertising BLE service:
      blePeripheral.begin();
    }
    // 蓝牙接收端
    bool Receive() {
      // listen for BLE peripherals to connect:
      BLECentral central = blePeripheral.central();
      // if a central is connected to peripheral:
      if (central) {
        // while the central is still connected to peripheral:
        if (central.connected()) {
          if (switchCharacteristic.written()) {
            return (p = switchCharacteristic.value());
          }
        }
      }
      return false;
    }
};

// 更新PID参数
float dat;
#define UPDATE_PID(PID) { \
    dat = 0; \
    while (Serial.available()) { \
      c = Serial.read(); \
      dat = 10 * dat + c - '0'; \
    } \
    dat /= 1000; PID = dat; \
  }

// 混合PID输出值
// 电机输入 = 基础油门
//			+ alt高度的pid
//			+ x * roll横滚角的pid
//			+ y * pitch俯仰角的pid
//			+ 航向正向定义 * z * yaw航向的pid
// 其中x,y,z代表该pid结果对相应电机的控制是促进还是抑制
#define PIDMIX(X,Y,Z) \
  Throttle + alt.Result \
  + roll.Result*X + pitch.Result*Y + YAW_DIRECTION*yaw.Result*Z

// 飞行器
class Fly {
  private:
    Motor motor[4];        // 电机
    PIDController roll, pitch, yaw, alt; // 横滚角、俯仰角、偏航角和飞行高度的PID控制器
    BlueTooth ble;         // 蓝牙
    Sonar sonar;           // 超声波模块
    Compass compass;       // 电子罗盘模块
    GPS gps;               // GPS模块
    Madgwick filter;      // IMU滤波器
    uint8_t turn = 0;      // 定义传感器使用顺序，轮转调度各个低速传感器模块
    uint8_t Throttle;      // 油门
    // 转换加速度
    float convertRawAcceleration(int aRaw) {
      // since we are using 2G range
      // -2g maps to a raw value of -32768
      // +2g maps to a raw value of 32767

      float a = (aRaw * 2.0) / 32768.0;
      return a;
    }
    // 转换陀螺仪数据
    float convertRawGyro(int gRaw) {
      // since we are using 250 degrees/seconds range
      // -250 maps to a raw value of -32768
      // +250 maps to a raw value of 32767

      float g = (gRaw * 250.0) / 32768.0;
      return g;
    }
  public:
    int MotorFlag, FlyFlag;// 电机初始化标志，启动标志
    Fly() {
      Throttle = THROTTLE;
      MotorFlag = FlyFlag = false;
    }
    // Setup飞控
    void Setup() {
      Serial.begin(9600);
      // 蓝牙
      ble.Setup();
      // IMU
      Serial.println("[ Setup IMU... ]");
      CurieIMU.begin();
      CurieIMU.setGyroRate(25);
      CurieIMU.setAccelerometerRate(25);
      CurieIMU.setAccelerometerRange(2);
      CurieIMU.setGyroRange(250);
      filter.begin(25);
      // 超声波模块 电子罗盘 GPS模块
      sonar.Setup(2, 13);
      compass.Setup();
      gps.Setup();
      // 电机
      Serial.println("[ Preparing Motor... ]");
      motor[FL].setPin(MOTOR_PIN_0);
      motor[FR].setPin(MOTOR_PIN_1);
      motor[BL].setPin(MOTOR_PIN_2);
      motor[BR].setPin(MOTOR_PIN_3);
      motor[FL].setSpeed(300);
      motor[FR].setSpeed(300);
      motor[BL].setSpeed(300);
      motor[BR].setSpeed(300);
      for (int i = 0; i <= 110; i++) {
        motor[FL].Write();
        motor[FR].Write();
        motor[BL].Write();
        motor[BR].Write();
      }
      motor[FL].setSpeed(20);
      motor[FR].setSpeed(20);
      motor[BL].setSpeed(20);
      motor[BR].setSpeed(20);
      for (int i = 0; i <= 55; i++) {
        motor[FL].Write();
        motor[FR].Write();
        motor[BL].Write();
        motor[BR].Write();
      }
      Serial.println("[ Ready to take up ]");
      MotorFlag = true, FlyFlag = false;
    }
    // 接收数据
    void Receive() {
      while (Serial.available()) {
        char c = Serial.peek();
        Serial.print("cmd:\t"); Serial.println(c);
        switch (c) {
          case 'P':
            UPDATE_PID(PIDController::Kp);
            Serial.print("Kp:"); Serial.println(dat);
            break;
          case 'I':
            UPDATE_PID(PIDController::Ki);
            Serial.print("Ki:"); Serial.println(dat);
            break;
          case 'D':
            UPDATE_PID(PIDController::Kd);
            Serial.print("Kd:"); Serial.println(dat);
            break;
          case 'x':
            FlyFlag = true; Throttle = 80;
            c = Serial.read();
            break; // 启动
          case 'q':
            alt.Target = 0; c = Serial.read();
            break; // 退出
          case 'u':
            alt.Target += 1;
            alt.Target = alt.Target > MAX_ALT ? MAX_ALT : alt.Target;
            roll.ErrIntg = pitch.ErrIntg = yaw.ErrIntg = alt.ErrIntg = 0;
            c = Serial.read();
            break; // 上升
          case 'd':
            alt.Target -= 1;
            alt.Target = alt.Target < MIN_ALT ? MIN_ALT : alt.Target;
            roll.ErrIntg = pitch.ErrIntg = yaw.ErrIntg = alt.ErrIntg = 0;
            c = Serial.read();
            break; // 下降
          case 'f':
            pitch.Target = 12.0; c = Serial.read();
            break; // 前进
          case 'b':
            pitch.Target = -12.0; c = Serial.read();
            break; // 后退
          case 'p':
            roll.Target =  12.0; c = Serial.read();
            break; // 左飞
          case 'n':
            roll.Target =  -12.0; c = Serial.read();
            break; // 右飞
          case 'l':
            yaw.Target += 6.0;
            if (yaw.Target > 360.0) yaw.Target -= 360.0;
            c = Serial.read();
            break; // 左转
          case 'r':
            yaw.Target -= 6.0;
            if (yaw.Target <   0.0) yaw.Target += 360.0;
            c = Serial.read();
            break; // 右转
          case 's':
            roll.Target = pitch.Target = 0;
            yaw.Target = yaw.Actual; alt.Target = alt.Actual;
            c = Serial.read();
            break; // 悬停
          default:
            break;
        }
      }
    }
    // 蓝牙接收模块
    void ReceiveBLE() {
      if (ble.Receive()) {
        int p = ble.p;
        switch (ble.p) {
          case 10:
            FlyFlag = true; Throttle = THROTTLE;
            roll.ErrIntg = pitch.ErrIntg = yaw.ErrIntg = alt.ErrIntg = 0;
            Serial.println(p);
            break; // 启动
          case 7 :
            alt.Target = 0.0; Stop(); Serial.println(p);
            break; // 退出 0
          case 2 :
            alt.Target += 2;
            alt.Target = alt.Target > MAX_ALT ? MAX_ALT : alt.Target;
            roll.ErrIntg = pitch.ErrIntg = yaw.ErrIntg = alt.ErrIntg = 0;
            Serial.println(p);
            break; // 上升
          case 5 :
            alt.Target -= 2;
            alt.Target = alt.Target < MIN_ALT ? MIN_ALT : alt.Target;
            roll.ErrIntg = pitch.ErrIntg = yaw.ErrIntg = alt.ErrIntg = 0;
            Serial.println(p);
            break; // 下降
          case 1 :
            pitch.Target = 12.0; Serial.println(p);
            break; // 前进
//          case 7 :
//            pitch.Target = -12.0; Serial.println(p);
//            break; // 后退
          case 3 :
            yaw.Target += 6.0;
            if (yaw.Target > 360.0) yaw.Target -= 360.0;
            Serial.println(p);
            break; // 左转
          case 4 :
            yaw.Target -= 6.0;
            if (yaw.Target < 0.0) yaw.Target += 360.0;
            Serial.println(p);
            break; // 右转
          case 6 :
            roll.Target = pitch.Target = 0;
            yaw.Target = yaw.Actual; alt.Target = alt.Actual;
            Serial.println(p);
            break; // 悬停
          case 8:
            break; //经纬度
          case 9:
            break; //PID
          default:
            break;
        }
      }
    }
    // 获取传感器信息
    void Compute() {
      // 不在同一个循环一次调用多个函数，避免计算延迟
      switch (turn) {
        case 0:
          turn = 1;
          sonar.Compute();// 计算高度
          alt.Compute(sonar.altitude);// 高度pid控制
          break;
        case 1:
          turn = 2;
          compass.Compute();// 计算航向
          yaw.Compute(compass.headingDegrees);// 航向pid控制
          break;
        case 2:
          turn = 0;
          if (gps.Compute()) { // 计算经纬度
            yaw.Target = gps.Result; // 设定目标航向
            if (abs(yaw.Actual - gps.Result) < 5.0) {
              roll.Target = 0; // 如果航向误差很小，飞行器直接前进
              pitch.Target = 12.0;
            } else {
              roll.Target = 0; // 如果航向误差很大，调整飞行器航向
              pitch.Target = 0.0;
            }
          }
          break;
        default:
          break;
      }
    }
    // 定时pid控制【暂时未使用定时器控制pid】
    void timedPID() {
      // 获取角度制下的姿态角
      int aix, aiy, aiz, gix, giy, giz;
      float ax, ay, az, gx, gy, gz;
      CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
      // convert from raw data to gravity and degrees/second units
      ax = convertRawAcceleration(aix);
      ay = convertRawAcceleration(aiy);
      az = convertRawAcceleration(aiz);
      gx = convertRawGyro(gix);
      gy = convertRawGyro(giy);
      gz = convertRawGyro(giz);

      // update the filter, which computes orientation;
      filter.updateIMU(gx, gy, gz, ax, ay, az);

      // 根据姿态角，使用PID得到相应控制信息
      roll.Compute(filter.getRoll());
      pitch.Compute(filter.getPitch());
    }
    // 电机输出
    void Output() {
      // 综合各个模块的pid输出值，计算四个电机所需的输入，+1代表促进作用，-1代表抑制作用
      motor[FL].setSpeed(PIDMIX(+1, -1, -1));
      motor[FR].setSpeed(PIDMIX(-1, -1, +1));
      motor[BL].setSpeed(PIDMIX(+1, +1, +1));
      motor[BR].setSpeed(PIDMIX(-1, +1, -1));

      // 水平倾角大于30度的时候，将电机停下来
      if ( abs(roll.Actual) > MAX_ANGLE || abs(pitch.Actual) > MAX_ANGLE ) {
        Stop();
      }

      // 更新电机速度
      if (MotorFlag && FlyFlag) {
        motor[FL].Write();
        motor[FR].Write();
        motor[BL].Write();
        motor[BR].Write();
      }
    }
    // 飞行日志
    void Logger() {
      Serial.print("in:"); Serial.print(roll.Actual);
      Serial.print("\t"); Serial.print(pitch.Actual);
      Serial.print("\t"); Serial.print(yaw.Actual);
      Serial.print("\t"); Serial.print(alt.Actual);
      Serial.print("\tpid:"); Serial.print(roll.Result);
      Serial.print("\t"); Serial.print(pitch.Result);
      Serial.print("\t"); Serial.print(yaw.Result);
      Serial.print("\t"); Serial.print(alt.Result);
      Serial.print("\tout:"); Serial.print(motor[FL].getSpeed());
      Serial.print("\t"); Serial.print(motor[FR].getSpeed());
      Serial.print("\t"); Serial.print(motor[BL].getSpeed());
      Serial.print("\t"); Serial.print(motor[BR].getSpeed());
      Serial.print("\tloc:"); Serial.print(gps.longitude, 5);
      Serial.print("\t"); Serial.println(gps.latitude, 5);
    }
    // 停止电机转动
    void Stop() {
      MotorFlag = false;
      FlyFlag = false;
      motor[FL].Stop();
      motor[FR].Stop();
      motor[BL].Stop();
      motor[BR].Stop();
    }
};

const int oneSecInUsec = 1000000;
int ticks = 5;

// 飞控
Fly fly;

void setup() {
  fly.Setup();
}

void loop() {
  fly.Receive();      // 接收串口数据
  fly.ReceiveBLE();   // 接收蓝牙指令
  if (fly.FlyFlag) {
    fly.Compute();    // 获取传感器数据
    fly.timedPID();   // 计算pid输出
    fly.Output();     // 电机输出控制
  }
  fly.Logger();       // 输出日志信息
}
