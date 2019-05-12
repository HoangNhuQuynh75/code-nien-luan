
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// Kiểm soát MPU
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];//Bộ đệm lưu trữ FIFO

//định hướng chuyển động
Quaternion q; // [w, x, y, z]
VectorFloat gravity; // [x, y, z]vector trọng lực
float ypr[3];

//PID
double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

// điều chỉnh các giá trị Kp Ki Kd để phù hợp với thân xe làm cho xe cân bằng
double Kp = 60;   
double Kd = 1.9;
double Ki = 112;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.8;
double motorSpeedFactorRight = 0.8;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
mpuInterrupt = true;
}


void setup()
{

// I2C
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

mpu.initialize();

devStatus = mpu.dmpInitialize();

// cung cấp bù đắp con quay hồi chuyển, được chia tỷ lệ cho độ nhạy tối thiểu
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788);


if (devStatus == 0)
{

mpu.setDMPEnabled(true);


attachInterrupt(0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus();


dmpReady = true;

// lấy kích thước gói DMP
packetSize = mpu.dmpGetFIFOPacketSize();

//setup PID
pid.SetMode(AUTOMATIC);
pid.SetSampleTime(10);
pid.SetOutputLimits(-255, 255); 
}
else
{

Serial.print(F("DMP Initialization failed (code "));
Serial.print(devStatus);
Serial.println(F(")"));
}
}


void loop()
{

if (!dmpReady) return;

// chờ MPU ngắt
while (!mpuInterrupt && fifoCount < packetSize)
{
//nếu không có dữ liệu mpu - thực hiện tính toán PID và xuất ra động cơ
pid.Compute();
motorController.move(output, MIN_ABS_SPEED);

}


// đặt lại ngắt và nhận byte INT_STATUS
mpuInterrupt = false;
mpuIntStatus = mpu.getIntStatus();


// lấy số lượng FIFO hiện tại
fifoCount = mpu.getFIFOCount();


// kiểm tra lỗi tràn
if ((mpuIntStatus & 0x10) || fifoCount == 1024)
{
// thiết lập lại
mpu.resetFIFO();
Serial.println(F("FIFO overflow!"));

//hãy kiểm tra ngắt dữ liệu DMP
}
else if (mpuIntStatus & 0x02)
{

// chờ độ dài dữ liệu có sẵn chính xác
while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

// đọc một gói từ FIFO
mpu.getFIFOBytes(fifoBuffer, packetSize);

// theo dõi số lượng FIFO ở đây trong trường hợp có> 1 gói có sẵn
//cho phép chúng tôi ngay lập tức đọc thêm mà không cần chờ ngắt
fifoCount -= packetSize;

mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
input = ypr[1] * 180/M_PI + 180;
}
}
