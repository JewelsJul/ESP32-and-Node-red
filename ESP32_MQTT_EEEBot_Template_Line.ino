//****************************************//
//* ESP32 MQTT EEEBot Template           *//
//* Modified from Rui Santos'            *//
//* https://randomnerdtutorials.com      *//
//*                                      *//
//* ESP32 Code                           *//
//*                                      *//
//* UoN 2022 - ND                        *//
//****************************************//

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HCSR04.h>
#include <MPU6050_tockn.h>
#include <MPU6500_WE.h>

MPU6500_WE myMPU6500 = MPU6500_WE(0x68);
MPU6050 mpu6050(Wire);
byte triggerPin = 2;
byte echoPin= 13;
int LeftSpeed,RightSpeed;
int Angle=90;

// Replace the next variables with your SSID/Password combination
const char* ssid = "C11Chip";                      //CHANGE ME
const char* password = "00000000";              //CHANGE ME     

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";                      
const char* mqtt_server = "192.168.137.52";          //CHANGE ME

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void MPUstart6500(){
  Serial.println("Hold still\n");
  myMPU6500.autoOffsets();
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
}

void Datapublish(char location[],int value){                  //Publishes the data to client. each with a respective name and value
    char VtoC[8]; 
    dtostrf(value,1,2,VtoC);                                   //Converst value into char array 
    client.publish(location,VtoC);
}

void CarSpeedAngle(int x,int y,int z){
    Wire.beginTransmission(0x04);
    Wire.write((byte)((x & 0x0000FF00) >> 8));    // first byte of left, containing bits 16 to 9
    Wire.write((byte)(x & 0x000000FF));           // second byte of left, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((y & 0x0000FF00) >> 8));    // first byte of right, containing bits 16 to 9
    Wire.write((byte)(y & 0x000000FF));           // second byte of right, containing the 8 LSB - bits 8 to 1
    Wire.write((byte)((z & 0x0000FF00) >> 8));    // first byte of angle, containing bits 16 to 9
    Wire.write((byte)(z & 0x000000FF));           // second byte of angle, containing the 8 LSB - bits 8 to 1
    Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  HCSR04.begin(triggerPin,echoPin);     //Tells where the UltraSonic Sensor is connected to on the ESP32
  delay(200);

  MPUstart6500();                       //MPU inilitsation, handles angle,acceleration and temperature
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(200);

  client.publish("Wready","on");
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {                                //Called everytime when there is a message from the RedPI. This will control driving of Bot
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
  // Add your subscribed topics here i.e. statements to control GPIOs with MQTT
  // --
    if (String(topic) == "vroom") {                                               //Drive Option
      if(messageTemp == "on"){
        Serial.println("go");
        LeftSpeed=RightSpeed=255;                                                        //Speed of the car (Left motor,Right motor)
      }
      else if(messageTemp == "off"){
        Serial.println("stop");
        LeftSpeed=RightSpeed=0; 
      }
    }
    else if (String(topic)=="angle"){                                             //Steering Direction
      if (messageTemp=="right")
      {
        Serial.println("Turning Right");
        Angle=135;                                                            //Angles of which range from 0-180  
      } 
      else if (messageTemp=="left")
      {
        Serial.println("Turning Left");
        Angle=45;
      } 
      else if (messageTemp=="straight")
      {
        Serial.println("Going Straight");
        Angle=90;
      }
      else if (messageTemp=="back")
      {
        Serial.println("Going backwards");
        LeftSpeed=RightSpeed=-255;
      }
      else if (messageTemp=="forward")
      {
        Serial.println("Going foward");
        LeftSpeed=RightSpeed=255;
      }   
    }
    CarSpeedAngle(LeftSpeed,RightSpeed,Angle);
  // --
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("WillClient")) {
      Serial.println("connected");
      
      // Add your subscribe topics here. These will match the ones on REDPI
      // --;
      client.subscribe("vroom");            //Subscribe to only things you want to receive from the PI
      client.subscribe("angle");
//      client.subscribe("WgyrZ");
//      client.subscribe("WgyrY");
//      client.subscribe("WgyrX");
//      client.subscribe("WTempMPU");
//      client.subscribe("WDistance");
      // --
         
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  long now = millis();
  if (now - lastMsg > 100) {
    lastMsg = now;
    
    mpu6050.update();                                 //Static angle
                                              
    Serial.println("Gyroscope data in degrees/s: ");                                  //Prints everything the MPU can provide which is relevant to the serial monitor and REDPI
    Serial.print(mpu6050.getAngleX());
    Datapublish("WgyrX",mpu6050.getAngleX());
    Serial.print("   ");

    
    Serial.print(mpu6050.getAngleY());
    Datapublish("WgyrY",mpu6050.getAngleY());
    Serial.print("   ");

    
    Serial.println(mpu6050.getAngleZ());
    Datapublish("WgyrZ",mpu6050.getAngleZ());

    Serial.print("Temperature in °C: ");
    Serial.println(temp);
    Datapublish("WTempMPU",temp);
    
    Serial.println("********************************************");
    delay(200);

    //////////////////////////////////////////////////////////////////////////////////////////

    double* distances = HCSR04.measureDistanceCm();                                                 //Measures distance from HC-SR04 to an object(Physical Hard surface works best)
    Serial.print(distances[0]);
    Serial.println(" cm");
    Serial.println("---");
    Datapublish("WDistance",distances[0]);

    /////////////////////////////////////////////////////////////////////////////////////////////
  }
}
