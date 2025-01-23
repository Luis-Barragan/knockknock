#include <Arduino.h>
#include <Wire.h> // arduino I2C library

#define IMU_ADDR    0x6A
#define STATUS_REG  0x1E  // status register
#define CTRL1_XL    0x10  // address of CTRL1_XL register
#define OUTX_L_XL   0x28  // X-axis linear acceleration value (LSbyte) D[7:0]
#define OUTX_H_XL   0x29  // X-axis linear acceleration value (MSbyte) D[15:8]
#define OUTY_L_XL   0x2A  // Y-axis linear acceleration value (LSbyte) D[7:0]
#define OUTY_H_XL   0x2B  // Y-axis linear acceleration value (MSbyte) D[15:8]
#define OUTZ_L_XL   0x2C  // Z-axis linear acceleration value (LSbyte) D[7:0]
#define OUTZ_H_XL   0x2D  // Z-axis linear acceleration value (MSbyte) D[15:8]

struct AccelData {
  int8_t ax1;
  int8_t ax2;
  int8_t ay1;
  int8_t ay2;
  int8_t az1;
  int8_t az2;
  int16_t ax;
  int16_t ay;
  int16_t az;
  bool ready;
};

void imu_INIT() {
  Wire.beginTransmission(IMU_ADDR); // write to slave
  Wire.write(CTRL1_XL); // write to address of CTRL1_XL register
  Wire.write(0x60); // 0b01000000 sets ODR to 104Hz
  Wire.endTransmission();
}

AccelData get_AccelData() 
{
  AccelData data = {0,0};
  Wire.beginTransmission(IMU_ADDR); // write to slave IMU
  Wire.write(STATUS_REG); // write to address of STATUS_REG register
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDR, 1); // Request data from STATUS_REG register

  if (Wire.available()) {  // check if bus is available (data is received)
    uint8_t statusReg = Wire.read();  // read data
    if (statusReg & 0b00000001) {   // check if XLDA bit is 1, new data check
      //Serial.printf("STATUS_REG: 0x%02X\n", statusReg);
      Wire.beginTransmission(IMU_ADDR); // write to slave
      Wire.write(OUTX_L_XL); // write to address of accel C LS register
      Wire.endTransmission();
      Wire.requestFrom(IMU_ADDR, 1);
      if (Wire.available()) {
        data.ax1 = Wire.read();   // write data to accel data struct
      }
      Wire.beginTransmission(IMU_ADDR); // write to slave IMU
      Wire.write(OUTX_H_XL);  // write to address of accel C MS register
      Wire.endTransmission();
      Wire.requestFrom(IMU_ADDR, 1);
      if (Wire.available()) {
        data.ax2 = Wire.read();
      }
      Wire.beginTransmission(IMU_ADDR); // write to slave
      Wire.write(OUTY_L_XL); // write to address of CTRL1_XL register
      Wire.endTransmission();
      Wire.requestFrom(IMU_ADDR, 1);
      if (Wire.available()) {
        data.ay1 = Wire.read();
      }
      Wire.beginTransmission(IMU_ADDR); // write to slave
      Wire.write(OUTY_H_XL); // write to address of CTRL1_XL register
      Wire.endTransmission();
      Wire.requestFrom(IMU_ADDR, 1);
      if (Wire.available()) {
        data.ay2 = Wire.read();
      }
      Wire.beginTransmission(IMU_ADDR); // write to slave
      Wire.write(OUTZ_L_XL); // write to address of CTRL1_XL register
      Wire.endTransmission();
      Wire.requestFrom(IMU_ADDR, 1);
      if (Wire.available()) {
        data.az1 = Wire.read();
      }
      Wire.beginTransmission(IMU_ADDR); // write to slave
      Wire.write(OUTZ_H_XL); // write to address of CTRL1_XL register
      Wire.endTransmission();
      Wire.requestFrom(IMU_ADDR, 1);
      if (Wire.available()) {
        data.az2 = Wire.read();
      }

      data.ax = (int16_t)((data.ax2 << 8) | data.ax1); // Combine bytes
      data.ay = (int16_t)((data.ay2 << 8) | data.ay1); // Combine bytes
      data.az = (int16_t)((data.az2 << 8) | data.az1); // Combine bytes
      data.ready = 1;
    } else {
      Serial.println("Data is not ready");
      data.ready = 0;
    }
  } else {
      Serial.println("Failed to read STATUS_REG");
      data.ready = 0;
    }
  return data;
}
                               
void setup() 
{
  Serial.begin(115200);
  Wire.setPins(GPIO_NUM_21, GPIO_NUM_22); // manually set I2C pins
  Wire.begin();   // starts I2C
  Wire.setClock(100000 );  // Set to 400kHz for Fast Mode I2C
  imu_INIT();
}

void loop() 
{
  AccelData accel = get_AccelData();
  if (accel.ready) {
    Serial.printf("X Accel:, %f\n", accel.ax*0.061);
    Serial.printf("Y Accel:, %f\n", accel.ay*0.061);
    Serial.printf("Z Accel:, %f\n", accel.az*0.061);
  }
  
 // delay(10);
}
