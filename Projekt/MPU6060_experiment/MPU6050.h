#ifndef MPU6050_h
#define MPU6050_h
#include "Arduino.h"
#include "Wire.h"
class MPU6050{
  public:
    int MPU = 0x68;
    float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
    float roll, pitch, yaw;
    float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
    float elapsedTime, currentTime, previousTime;
    int c=0;
    MPU6050()
    {

    }
    void beginMPU(){
      Serial.begin(9600);
      Wire.begin();         //initialize communication
      TWBR=12;
      Wire.beginTransmission(MPU);
      Wire.write(0x6B);     //Talk to register 6B
      Wire.write(0x00);     //Reset -> place 0 in register 6B (disable sleep mode)
      Wire.endTransmission(); //end transmission
      //Set the full scale of the gyro to +- 250 degrees/second
      Wire.beginTransmission(MPU);
      Wire.write(0x1B);     
      Wire.write(0x00);     
      Wire.endTransmission(); //end transmission
      //Set accelerometor sensitivity to +- 4g
      Wire.beginTransmission(MPU);
      Wire.write(0x1C);     
      Wire.write(0x08);     
      Wire.endTransmission(); //end transmission
      //filtering to improve raw data
      Wire.beginTransmission(MPU);
      Wire.write(0x1A);     
      Wire.write(0x03);     
      Wire.endTransmission(); //end transmission      
    }
    
    void updateMeasurements(){
      Wire.beginTransmission(MPU);
      Wire.write(0x3B); // start with first register 
      Wire.endTransmission();
      Wire.requestFrom(MPU, 6, true); //read 6 registers
      AccX=(Wire.read() << 8 | Wire.read())/8192.0;
      AccY=(Wire.read() << 8 | Wire.read())/8192.0;
      AccZ=(Wire.read() << 8 | Wire.read())/8192.0;
      
      //calculate roll and pitch
      //if(AccX>8200)AccX=8200;
      //if(AccX<-8200)AccX=-8200;
      accAngleX=(atan(AccY / sqrt(pow(AccX,2)+pow(AccZ,2)))*180/PI); // Put acceleration error here
      accAngleY=(atan(-1*AccX / sqrt(pow(AccY,2)+pow(AccZ,2)))*180/PI); // put accelecation error here
      
      //=========DO THE SAME FOR GYRO DATA=====================
      
      previousTime = currentTime;
      currentTime = millis();
      elapsedTime = (currentTime - previousTime) / 1000;
      Wire.beginTransmission(MPU);
      Wire.write(0x43); // start with first register 
      Wire.endTransmission();
      Wire.requestFrom(MPU, 6 , true); //read 6 registers
      GyroX=(Wire.read() << 8 | Wire.read())/131.0;
      GyroY=(Wire.read() << 8 | Wire.read())/131.0;
      GyroZ=(Wire.read() << 8 | Wire.read())/131.0;
      // correct the outputs
      GyroX = GyroX;  //gyro offsets deducted here 
      GyroY = GyroY; 
      GyroZ = GyroZ;  

      // calculate in degrees 
      gyroAngleX += GyroX*elapsedTime;
      gyroAngleY += GyroY*elapsedTime;
      yaw += GyroZ * elapsedTime;

      //complementary filter
      roll=0.96 * gyroAngleX + 0.04*accAngleX;
      pitch=0.96 * gyroAngleY + 0.04*accAngleY;

      //print values on serial
      /*Serial.print(roll);
      Serial.print("/");
      Serial.print(pitch);
      Serial.print("/");
      Serial.println(yaw); */
    }
    
    void calculate_IMU_error(){
      while (c < 500)
      {
      Wire.beginTransmission(MPU);
      Wire.write(0x43); // start with first register 
      Wire.endTransmission();
      Wire.requestFrom(MPU, 6 , true); //read 6 registers
      GyroErrorX +=(Wire.read() << 8 | Wire.read())/131.0;
      GyroErrorY +=(Wire.read() << 8 | Wire.read())/131.0;
      GyroErrorZ +=(Wire.read() << 8 | Wire.read())/131.0;
      Wire.beginTransmission(MPU);
      Wire.write(0x3B); // start with first register 
      Wire.endTransmission();
      Wire.requestFrom(MPU, 6, true); //read 6 registers
      AccX=(Wire.read() << 8 | Wire.read())/8192.0;
      AccY=(Wire.read() << 8 | Wire.read())/8192.0;
      AccZ=(Wire.read() << 8 | Wire.read())/8192.0;
      AccErrorX+=(atan(AccY / sqrt(pow(AccX,2)+pow(AccZ,2)))*180/PI);
      AccErrorY+=(atan(-1*AccX / sqrt(pow(AccY,2)+pow(AccZ,2)))*180/PI);
        c++;
      }
      //take average error
      AccErrorX /= 500;
      AccErrorY /= 500;
      GyroErrorX /= 500;
      GyroErrorY /= 500;
      GyroErrorZ /= 500;
      Serial.print("GyroErrorX: ");
      Serial.print(GyroErrorX);
      Serial.print("/");
      Serial.print("GyroErrorY: ");
      Serial.print(GyroErrorY);
      Serial.print("/");
      Serial.print("GyroErrorZ: ");
      Serial.println(GyroErrorZ);
      Serial.print("/");
      Serial.print("AccErrorX: ");
      Serial.print(AccErrorX);
      Serial.print("/");
      Serial.print("AccErrorY: ");
      Serial.println(AccErrorY);
    }
};


#endif
