#include <PID_v1.h>
#include <Ewma.h>
#include <EwmaT.h>
#include <Geometry.h>
#include <Kinematics.h>
#include "WiFi.h"
#include "AsyncUDP.h"
#include "MPU9250.h"
#include <ESP32Servo.h>
#include "esp32-hal-cpu.h"

const char * ssid = "dog-net";
const char * password = "dog123";




// Set your Static IP address
IPAddress local_IP(10, 0, 0, 88);
// Set your Gateway IP address
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

AsyncUDP udp;

MPU9250 IMU(Wire,0x68);
int status;

float magX,magY,magZ, rotX,rotY,rotZ, accX,accY,accZ;

String servoString = " 90 90 0 90 90 0 90 90 0 90 90 0 ";

Servo myservo1;  
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;
Servo myservo9;
Servo myservo10;
Servo myservo11;
Servo myservo12;


void initServos()
{
  myservo1.attach(26, 500 , 2400);
  myservo2.attach(27, 500, 2400);
  myservo3.attach(14, 500, 2400);
  myservo4.attach(25, 500, 2400);
  myservo5.attach(33, 500, 2400);
  myservo6.attach(32, 500, 2400);
  myservo7.attach(4, 500, 2400);
  myservo8.attach(2, 500, 2400);
  myservo9.attach(15, 500, 2400);
  myservo10.attach(16, 500, 2400);
  myservo11.attach(13, 500, 2400);
  myservo12.attach(12, 500, 2400);
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setup()
{
  setCpuFrequencyMhz(240);
  initServos();
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  // Configures static IP address
  
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed");
    while (1) {
      delay(1000);
    }
  }

   // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  
  if (udp.listen(1234)) {
    Serial.print("UDP Listening on IP: ");
    Serial.println(WiFi.localIP());
    udp.onPacket([](AsyncUDPPacket packet) {  
      if(packet.length()>3) {
          //Serial.print(", Data: ");
          servoString = (char*)packet.data();
          //Serial.print(servoString);     
        }
      //reply to the client
      packet.printf("%3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f %3.3f",magX,magY,magZ,rotX,rotY,rotZ,accX,accY,accZ);
    });
  }
}

void loop()
{
  myservo1.write(180-getValue(servoString,' ',1).toInt());
  myservo2.write(getValue(servoString,' ',2).toInt()+10);
  myservo3.write(getValue(servoString,' ',3).toInt());
  
  myservo4.write(180-getValue(servoString,' ',4).toInt()-12);
  myservo5.write(180-getValue(servoString,' ',5).toInt());
  myservo6.write(180-getValue(servoString,' ',6).toInt());

  myservo7.write(180-getValue(servoString,' ',7).toInt());
  myservo8.write(getValue(servoString,' ',8).toInt());
  myservo9.write(getValue(servoString,' ',9).toInt());

  myservo10.write(180-getValue(servoString,' ',10).toInt()-12);
  myservo11.write(180-getValue(servoString,' ',11).toInt());
  myservo12.write(180-getValue(servoString,' ',12).toInt());

  //IMU.readSensor();
  //magX = IMU.getMagX_uT();
  //magY = IMU.getMagY_uT();
  //magZ = IMU.getMagZ_uT();
  
  //rotX = IMU.getGyroX_rads();
  //rotY = IMU.getGyroY_rads();
  //rotZ = IMU.getGyroZ_rads();

  //accX = IMU.getAccelX_mss();
  //accY = IMU.getAccelY_mss();
  //accZ = IMU.getAccelZ_mss();



}
