#include <PID_v1.h>
#include <Ewma.h>
#include <EwmaT.h>
#include <Geometry.h>
#include <Kinematics.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// wifi Details
const char* ssid = "dog-net";
const char* password = "dog123";


// Create servo objects
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

// Kinematic object
Kinematics leg(40, 40);


Angle angLF;
Angle angRF;
Angle angLB;
Angle angRB;

float animationTimer = 0;
float   off2 = 0;
bool sw1 = false;


Rotation orientationB;
float xdot, zdot, dt = 0.5;

Ewma gyroF(0.10);
float xdotF;
double ydotF, ydot, Setpoint=0,kp=0.08,ki=0.008,kd=0.00007;

//Specify the links and initial tuning parameters
PID myPID(&ydot, &ydotF, &Setpoint,kp,ki,kd,P_ON_M, REVERSE);



/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

bool sensorStart = false;
int pitchInit = 0;
int rollInit = 0;
int pitchX = 0;
int pithY = 0;

void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
}

void initServos()
{
  myservo1.attach(32, 500 , 2400);
  myservo2.attach(33, 500, 2400);
  myservo3.attach(25, 500, 2400);
  myservo4.attach(26, 500, 2400);
  myservo5.attach(27, 500, 2400);
  myservo6.attach(14, 500, 2400);
  myservo7.attach(12, 500, 2400);
  myservo8.attach(13, 500, 2400);
  myservo9.attach(15, 500, 2400);
  myservo10.attach(2, 500, 2400);
  myservo11.attach(4, 500, 2400);
  myservo12.attach(16, 500, 2400);
}


void setup() {
  Kinematics leg(40, 40);
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  /**************************************************************************/
  /*!

  */
  /**************************************************************************/



  /* Initialise the sensors */
  initSensors();
  initServos();


myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(-90,90);

}

void loop() {
  
  if (abs(xdotF -xdot) < 50) {
    kp = 100;
    ki = 0 ;
    kd = 0.0022 ;
    }
  else {
    kp = 2;
    ki = 0 ;
    kd = 0 ;
    }  
  
  myPID.Compute();

  
  animationTimer =  animationTimer + 1;
  if (animationTimer > 10)
  {
     //xdotF = gyroF.filter(xdot);
     //ydotF = gyroF.filter(ydot);

    //xdotF = map(xdotF,-30,30,-90,90);
    orientationB.FromEulerAngles(ydotF*(M_PI/360),0,0);


    Point AnimatePos ;
    AnimatePos(0)= 3 ;
    AnimatePos(1)=0 ;
    AnimatePos(2)=50+sin(off2)*10;
    Point RotatedPos = orientationB * AnimatePos;
    leg.moveToPosition(RotatedPos.X(), RotatedPos.Y(),RotatedPos.Z());
    angLF = leg.getAngles();
    angRB = angLF;

     AnimatePos(2)=50-sin(off2)*10;
     RotatedPos = orientationB * AnimatePos;
     leg.moveToPosition(RotatedPos.X(), RotatedPos.Y(),RotatedPos.Z());
    angRF = leg.getAngles();
    angLB = angRF;
    
    animationTimer = 0;
    if (!sw1)
    {
      off2=off2+(M_PI/10);
    }
    else {
      off2=off2-(M_PI/10);
    }
    if (off2 > M_PI or off2 < 0) sw1 = !sw1;
  }

  ///Handle OTA upload
  ArduinoOTA.handle();

  
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    if (sensorStart == false) {
      pitchInit = orientation.pitch;
      rollInit = orientation.roll;
     sensorStart= true;
      //Serial.print(F("asdajsdkfhsldkfalsdfhl "));
    }
    xdot = orientation.roll + rollInit;
    ydot = gyroF.filter(orientation.pitch + pitchInit);
    zdot = 0;
  }

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
  }

  myservo1.write(angLF.theta3 + 90);
  myservo2.write(180 - angLF.theta1-5);
  myservo3.write(map(angLF.theta2, 0, -180, 0, 180)-9);
  
  myservo4.write(angRF.theta3 + 90);
  myservo5.write(angRF.theta1-5);
  myservo6.write(map(angRF.theta2, 0, -180, 180, 0)+15);

  myservo7.write(angLB.theta3 + 90 + 4);
  myservo8.write(180 - angLB.theta1+10);
  myservo9.write(map(angLB.theta2, 0, -180, 0, 180)-30);

  myservo10.write(angRB.theta3 + 90);
  myservo11.write(angRB.theta1);
  myservo12.write(map(angRB.theta2, 0, -180, 180, 0));
  
}
