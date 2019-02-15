#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B
float AcX,AcY,AcZ,GyX,GyY,GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
float axC, ayC, azC, gxC, gyC, gzC;

boolean fall = false; //stores if a fall has occurred
boolean trigger1=false; //stores if first trigger (lower threshold) has occurred
boolean trigger2=false; //stores if second trigger (upper threshold) has occurred
boolean trigger3=false; //stores if third trigger (orientation change) has occurred

byte trigger1count=0; //stores the counts past since trigger 1 was set true
byte trigger2count=0; //stores the counts past since trigger 2 was set true
byte trigger3count=0; //stores the counts past since trigger 3 was set true
int angleChange=0;


#if SIMULATED_DATA

void initSensor()
{
    // use SIMULATED_DATA, no sensor need to be inited
}

#else

static DHT dht(DHT_PIN, DHT_TYPE);
void initSensor()
{
    delay(1000); //relax...
    Serial.println("Processor came out of reset.\n");
   
    //Call .begin() to configure the IMU
    myIMU.begin();
}

boolean readFallDetection()
{
    mpu_read();

    ax = AcX;
    ay = AcY;
    az = AcZ;
    gx = GyX;
    gy = GyY;
    gz = GyZ;
  
    // calculating Amplitute vactor for 3 axis
    float Raw_AM = pow(pow(ax,2)+pow(ay,2)+pow(az,2),0.5);
    int AM = Raw_AM * 10;  // as values are within 0 to 1, I multiplied 
                           // it by for using if else conditions 
  
    //  Serial.println("Accelerometer:");
    //  Serial.println(" X = " + String(AcX) + " " + String(ax));
    //  Serial.println(" Y = " + String(AcY) + " " + String(ay));
    //  Serial.println(" Z = " + String(AcZ) + " " + String(az));
    //  Serial.println("\nGyroscope:");
    //  Serial.println(" X = " + String(GyX) + " " + String(gx));
    //  Serial.println(" Y = " + String(GyY) + " " + String(gy));
    //  Serial.println(" Z = " + String(GyZ) + " " + String(gz));
    //
    //  Serial.println("Row: ");
    //  Serial.println(Raw_AM);
  
  
       if (trigger3==true) {
          trigger3count++;
          //Serial.println(trigger3count);
          if (trigger3count>=10) {
              angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
              //delay(10);
              Serial.println(angleChange);
              if ((angleChange>=0) && (angleChange<=10)) { //if orientation changes remains between 0-10 degrees
                  fall=true;
                  trigger3=false;
                  trigger3count=0;
                  Serial.println(angleChange);
              }
              else { //user regained normal orientation
                  trigger3=false;
                  trigger3count=0;
                  Serial.println("TRIGGER 3 DEACTIVATED");
              }
          }
      }
      if (fall==true) { //in event of a fall detection
          Serial.println("FALL DETECTED");
          digitalWrite(11, LOW);
          delay(20);
          digitalWrite(11, HIGH);
          fall=false;
          // exit(1);
          return true;
      }
      if (trigger2count>=6) { //allow 0.5s for orientation change
          trigger2=false;
          trigger2count=0;
          Serial.println("TRIGGER 2 DEACTIVATED");
      }
      if (trigger1count>=6) { //allow 0.5s for AM to break upper threshold
          trigger1=false;
          trigger1count=0;
          Serial.println("TRIGGER 1 DEACTIVATED");
      }
      if (trigger2==true) {
          trigger2count++;
          //angleChange=acos(((double)x*(double)bx+(double)y*(double)by+(double)z*(double)bz)/(double)AM/(double)BM);
          angleChange = pow(pow(gx,2)+pow(gy,2)+pow(gz,2),0.5);
          Serial.println(angleChange);
          if (angleChange >= 30) {
          //if (angleChange>=30 && angleChange<=400) { //if orientation changes by between 80-100 degrees
              trigger3=true;
              trigger2=false;
              trigger2count=0;
              Serial.println(angleChange);
              Serial.println("TRIGGER 3 ACTIVATED");
          }
      }
      if (trigger1==true) {
          trigger1count++;
          if (AM>=12) { //if AM breaks upper threshold (3g)
              trigger2=true;
              Serial.println("TRIGGER 2 ACTIVATED");
              trigger1=false;
              trigger1count=0;
          }
      }
      if (AM<=2 && trigger2==false) { //if AM breaks lower threshold (0.4g)
          trigger1=true;
          Serial.println("TRIGGER 1 ACTIVATED");
      }
      //Delay needed in order not to clog the port
      delay(100);
      return false;
}

void mpu_read() {
    //Get all parameters
    AcX = myIMU.readFloatAccelX();
    AcY = myIMU.readFloatAccelY();
    AcZ = myIMU.readFloatAccelZ();
    GyX = myIMU.readFloatGyroX();
    GyY = myIMU.readFloatGyroY();
    GyZ = myIMU.readFloatGyroZ();
  
    //  Serial.print("\nAccelerometer:\n");
    //  Serial.println(" X = " + AcX);
    //  Serial.println(" Y = " + AcY);
    //  Serial.println(" Z = " + AcZ);
    //  Serial.print("\nGyroscope:\n");
    //  Serial.println(" X = " + GyX);
    //  Serial.println(" Y = " + GyY);
    //  Serial.println(" Z = " + GyZ);
}

#endif

bool readMessage(int messageId, char *payload)
{
    boolean fallD = readFallDetection();
    StaticJsonBuffer<MESSAGE_MAX_LEN> jsonBuffer;
    JsonObject &root = jsonBuffer.createObject();
    root["deviceId"] = DEVICE_ID;
    root["messageId"] = messageId;
    bool fallAlert = false;

    if (fallD)
    {
       fallAlert = true;
    }
    
    root.printTo(payload, MESSAGE_MAX_LEN);
    return fallAlert;
}

void parseTwinMessage(char *message)
{
    StaticJsonBuffer<MESSAGE_MAX_LEN> jsonBuffer;
    JsonObject &root = jsonBuffer.parseObject(message);
    if (!root.success())
    {
        Serial.printf("Parse %s failed.\r\n", message);
        return;
    }

    if (root["desired"]["interval"].success())
    {
        interval = root["desired"]["interval"];
    }
    else if (root.containsKey("interval"))
    {
        interval = root["interval"];
    }
}
