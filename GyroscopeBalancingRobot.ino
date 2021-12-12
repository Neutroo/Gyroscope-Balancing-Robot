#include <HCSR04.h>
#include <MPU6050.h>
#include "I2Cdev.h"

#define TIME_GYRO 2000      // период опроса mpu6050 в микросекундах

MPU6050 mpu;

int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw; 
long int time_timer;                            // переменная таймера для опроса

// <<------------------------ ГИРОСКОП ---------------------------->> //
float gx, gx0, gy, gz;                          // значения угловой скорости в градусах в секунду
float angle_gx, angle_gx0, angle_gy, angle_gz;  // углы, рассчитанные по гироскопу
float gyro_x_zero, gyro_y_zero, gyro_z_zero;    // калибровочные углы смещения нуля гироскопа

//Motor driver pins
#define ENA       3
#define ABack     5
#define AForward  7
#define BBack     4
#define BForward  2
#define ENB       6

void setup() 
{
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);
  pinMode(ABack, OUTPUT);
  pinMode(AForward, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BBack, OUTPUT);
  pinMode(BForward, OUTPUT);

  digitalWrite(ABack, LOW);
  digitalWrite(AForward, HIGH);
  digitalWrite(BBack, LOW);
  digitalWrite(BForward, HIGH);

  mpu.initialize();
  calibrate_gyro();
}

uint8_t speedA = 120; 
uint8_t speedB = 120;  

uint8_t increaseA(int distance, uint8_t value)
{
  return value - distance;
}

uint8_t increaseB(int distance, uint8_t value)
{
  if (distance > 70)
    distance = 70;
  return distance - (value - 10);
}

void loop() 
{
  if( time_timer < micros() )
  {
    time_timer = micros() + TIME_GYRO;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    get_angle_gyro();

    if (angle_gz > 1)
    { 
      while (angle_gz > 0)
        {
          mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
          get_angle_gyro();
          analogWrite(ENA, 90);                  
          analogWrite(ENB, 0); 
        }                                              
    }
    else if (angle_gz < -1)
    {
      while (angle_gz < 0)
        {
          mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
          get_angle_gyro();
          analogWrite(ENA, 0);   
          analogWrite(ENB, 90); 
        }                  
    }
    else 
    {
      analogWrite(ENA, speedA);                  
      analogWrite(ENB, speedB);
    }
  }
}

void get_angle_gyro() 
{ 
  gx = (gx_raw - gyro_x_zero) / 131.0;
  gy = (gy_raw - gyro_y_zero) / 131.0;
  gz = (gz_raw - gyro_z_zero) / 131.0;  

  angle_gx = angle_gx + gx * TIME_GYRO / 1000000.0; 
  angle_gy = angle_gy + gy * TIME_GYRO / 1000000.0;
  angle_gz= angle_gz + gz * TIME_GYRO / 1000000.0;  
}

void calibrate_gyro()
{
  int i = 0;
  int NUM_CALIBRATE = 1000;
  while (i < NUM_CALIBRATE) 
  {
    if (time_timer < micros())
    {
      time_timer = micros() + TIME_GYRO;
      mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
      gyro_x_zero += gx_raw;
      gyro_y_zero += gy_raw;
      gyro_z_zero += gz_raw;
      ++i;
    }
  }
  gyro_x_zero /= NUM_CALIBRATE;
  gyro_y_zero /= NUM_CALIBRATE;
  gyro_z_zero /= NUM_CALIBRATE;
}
