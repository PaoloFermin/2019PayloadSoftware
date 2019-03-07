#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h>


#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);
float telem_gpslat;
float telem_gpslon;
float telem_gpssats;
float telem_gpsalt;
float telem_gpstime;
  
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  if (! baro.begin()) Serial.print("Barometer Failed");
  if (!lsm.begin()) Serial.print("Accelerometer Failed");
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(getSensorData());
  delay(100);
  
}

String getSensorData() {
  float pascals = baro.getPressure();
  float altm = baro.getAltitude();
  float tempC = baro.getTemperature();
  
  if (GPS.fix){
        telem_gpslat  = GPS.latitudeDegrees;
        telem_gpslon  = GPS.longitudeDegrees;
        telem_gpssats = (int) GPS.satellites;
        telem_gpsalt  = GPS.altitude;
        telem_gpstime = GPS.minute;
  }else{
    Serial.println("GPS waiting for fix");
  }

  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp); 

  float accX, accY, accZ, magX, magY, magZ, gyrX, gyrY, gyrZ;
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  magX = m.magnetic.x;
  magY = m.magnetic.y;
  magZ = m.magnetic.z;
  gyrX = g.gyro.x;
  gyrY = g.gyro.y;
  gyrZ = g.gyro.z;
  String answer = "pascals " + (String)pascals + " alt " + (String)altm + " tempC " + (String)tempC + 
    " lat " + (String)telem_gpslat + " lon " + (String)telem_gpslon + " time " + (String)telem_gpstime +
    " accX " + (String)accX + " magX " + (String)magX + " gyrX " + (String)gyrX;
  return answer;
} 

