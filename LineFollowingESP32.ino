#include <Wire.h>
#include <HCSR04.h>
#include <MPU6500_WE.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal


// the minimum and maximum values here are determined by the amount of bits used by the chosen variable type
// for int, this is either 16-bits or 32-bits
// due to two's complement, the minimum value is -2^(N-1), and the maximum is (2^(N-1))-1; where N is the number of bits

MPU6500_WE myMPU6500 = MPU6500_WE(0x68);
byte triggerPin = 2;
byte echoPin= 13;

  
void CarSpeedAngle(int x,int y,int z){
    Wire.beginTransmission(0x04); // transmit to device #4    >> X refers to a shift right operator by X bits
  
    Wire.write((byte)((x & 0x0000FF00) >> 8));    // first byte of left, containing bits 16 to 9  
    Wire.write((byte)(x & 0x000000FF));           // second byte of left, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((y & 0x0000FF00) >> 8));    // first byte of right, containing bits 16 to 9
    Wire.write((byte)(y & 0x000000FF));           // second byte of right, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((z & 0x0000FF00) >> 8));    // first byte of angle, containing bits 16 to 9
    Wire.write((byte)(z & 0x000000FF));           // second byte of angle, containing the 8 LSB - bits 8 to 1
  
    Wire.endTransmission();   // stop transmitting
    }

void ReversingDistance(){                   //Reverses a set distance
    Wire.requestFrom(0x04,16);
    int BaseLeftEnco=Wire.read();
    int BaseRightEnco=Wire.read();
    Serial.println(BaseLeftEnco);
    Serial.println(BaseRightEnco);
    delay(200);
    Wire.requestFrom(0x04,16);
    int LeftEnco=Wire.read();
    int RightEnco=Wire.read();
    CarSpeedAngle(-150,-150,90);
    while (LeftEnco!=BaseLeftEnco+10||RightEnco!=BaseRightEnco+10){
      Serial.println(LeftEnco);
      Serial.println(RightEnco);
      Wire.requestFrom(0x04,16);
      LeftEnco=Wire.read();
      RightEnco=Wire.read();
      }
    CarSpeedAngle(0,0,90);
  }

float Error(float a,float b, float c, float d, float e, float f){  //Error= Setpoint - WeightedAverage
    float Weight=(1*a + 0.666*b + 0.333*c + -0.333*d + -0.666*e + -1*f )/(a+b+c+d+e+f);   //Weighted Average
    float error=0-Weight;              //board will work as 3,2,1,0,1,2,3
    return error;
  }

void setup()
{
  Serial.begin(115200);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)

  Serial.println("Hold still\n");
  delay(1000);
  myMPU6500.autoOffsets();
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  delay(200);
  
  HCSR04.begin(triggerPin,echoPin);     //Tells where the UltraSonic Sensor is connected to on the ESP32
  
}

void loop()
{
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.println(gValue.x);
  Serial.print("   ");
  Serial.println(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);
  Serial.println("********************************************");
  Serial.println("Gyroscope data in degrees/s: ");
  Serial.println(gyr.x);
  Serial.print("   ");
  Serial.println(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);
  Serial.print("Temperature in °C: ");
  Serial.println(temp);
  Serial.println("********************************************");
  delay(1000);

/////////////////////////////////////////////////////////////////////////////////
  
  double* distances = HCSR04.measureDistanceCm();
  
  Serial.print("1: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
  
  Serial.println("---");
  delay(250);

//////////////////////////////////////////////////////////////////////////////////// Line following code

int Left=analogRead(14);                            //Reads the signal coming from each sensor
int LeftMid=analogRead(27);
int MidLeft=analogRead(26);
int MidRight=analogRead(25);
int RightMid=analogRead(33);
int Right=analogRead(32);

Serial.println("==============================");
Serial.print("Left: ");
Serial.println(Left);
Serial.print("LeftMid: ");
Serial.println(LeftMid);
Serial.print("MidLeft: ");
Serial.println(MidLeft);
Serial.print("MidRight: ");
Serial.println(MidRight);
Serial.print("RightMid: ");
Serial.println(RightMid);
Serial.print("Right: ");
Serial.println(Right);
Serial.println("==============================");

float Kp,Kd,Ki,PEn,servoAngle,leftMotorSpeed,rightMotorSpeed;
Kp=400;   Kd=Ki=0;
float En=Error(Left,LeftMid,MidLeft,MidRight,RightMid,Right);
Serial.println(En);
float SEn=SEn+En;
float u= Kp*En + Ki*SEn + Kd*(En-PEn);
Serial.println(u);
servoAngle=90-u;
leftMotorSpeed=100 + 0.1*u;               //baseSpeed+K*u     where K is the scaling factor (K<1)
rightMotorSpeed=100 - 0.1*u;
CarSpeedAngle(leftMotorSpeed,rightMotorSpeed,servoAngle);

PEn=En;

}
