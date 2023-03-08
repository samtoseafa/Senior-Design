// created by youngtose on 03/1/2023 at 7:41pm

// FeatherSense headers
// Include all the files that we need for the built-in sensors
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <PDM.h>

// GPS headers
// Include all the files that we need for the GPS featherwing
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Particle Sensor headers
#include "Adafruit_PM25AQI.h"

// Create objects for the FeatherSense sensors
Adafruit_APDS9960 apds9960; // proximity, light, color, gesture
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity

// Create objects for the GPS sensor
// Pin numbers can be changed to match wiring:
SoftwareSerial mySerial(6, 5);
Adafruit_GPS GPS(&mySerial);

// Create objects for the Particle Sensor
// Pin numbers can be changed to match wiring:
SoftwareSerial pmSerial(11, 13);
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// Create variables for the sensor data 
uint8_t proximity;
uint16_t r, g, b, c;
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
int32_t mic;
int counter = 0;
extern PDMClass PDM;
short sampleBuffer[256];  // buffer to read samples into, each sample is 16-bits
volatile int samplesRead; // number of samples read

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

//int32_t getPDMwave(int32_t samples);

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Remote System Active");

  // initialize the built-in sensors
  apds9960.begin();
  apds9960.enableProximity(true);
  apds9960.enableColor(true);
  bmp280.begin();
  lis3mdl.begin_I2C();
  lsm6ds33.begin_I2C();
  sht30.begin();
  PDM.onReceive(onPDMdata);
  PDM.begin(1, 16000);

  // initialize the GPS featherwing
  // default buad rate
  GPS.begin(9600);
  // recommended minimum, altitude and fix data 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // updates on antenna status
  //GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  // request firmware version
  mySerial.print("GPS Firmware Version: ");
  mySerial.println(PMTK_Q_RELEASE);

  // initialize the Particle Sensor
  pmSerial.begin(9600);
  // check that system is connected to master system
  if (! aqi.begin_UART(&pmSerial)) {
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }
  Serial.println("PM25 found!");
  Serial.println("Ready!");
}


// Get CPU time
uint32_t timer = millis();

void loop(void) {
  proximity = apds9960.readProximity();
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&r, &g, &b, &c);
  temperature = bmp280.readTemperature();
  pressure = bmp280.readPressure();
  altitude = bmp280.readAltitude(1013.25);

  lis3mdl.read();
  magnetic_x = lis3mdl.x;
  magnetic_y = lis3mdl.y;
  magnetic_z = lis3mdl.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_x = accel.acceleration.x;
  accel_y = accel.acceleration.y;
  accel_z = accel.acceleration.z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;

  humidity = sht30.readHumidity();

  mic = getPDMwave(4000);
  // approximately every 2 seconds or so, print out the current sensor info
  if (millis() - timer > 2000){
    timer = millis();
    counter += 1;
    Serial.print("\nSample (");
    Serial.print(counter);
    Serial.print(") \n");
    Serial.println("---------------------------------------------");
    Serial.print("Proximity: ");
    Serial.println(apds9960.readProximity());
    Serial.print("Red: ");
    Serial.print(r);
    Serial.print(" Green: ");
    Serial.print(g);
    Serial.print(" Blue :");
    Serial.print(b);
    Serial.print(" Clear: ");
    Serial.println(c);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.print("Barometric pressure: ");
    Serial.println(pressure);
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");
    Serial.print("Magnetic: ");
    Serial.print(magnetic_x);
    Serial.print(" ");
    Serial.print(magnetic_y);
    Serial.print(" ");
    Serial.print(magnetic_z);
    Serial.println(" uTesla");
    Serial.print("Acceleration: ");
    Serial.print(accel_x);
    Serial.print(" ");
    Serial.print(accel_y);
    Serial.print(" ");
    Serial.print(accel_z);
    Serial.println(" m/s^2");
    Serial.print("Gyro: ");
    Serial.print(gyro_x);
    Serial.print(" ");
    Serial.print(gyro_y);
    Serial.print(" ");
    Serial.print(gyro_z);
    Serial.println(" dps");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
    Serial.print("Mic: ");
    Serial.println(mic);
  }
  delay(300);
  // get GPS data
  char c = GPS.read();
  // debug check here **** 
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, run checksum and parse it
//  while (GPS.newNMEAreceived()) {
//    while (!GPS.parse(GPS.lastNMEA())){}
//      // if we fail to parse a sentence, we should just wait for another
//      // need a better method of handling this ****
//      // just returning truncates the data collection which is undersirable
//      // perhaps fill data with zeros and skip GPS for the cycle?
//      // return;
//  }

  // approximately every 2 seconds or so, print out the current GPS info
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: "); Serial.println((int)GPS.antenna);
     }
    }
    delay(300);
    
    PM25_AQI_Data data;
    while (! aqi.read(&data)) {
//      Serial.println("Could not read from AQI");
//      delay(500);  // try again in a bit!
//      return;
     }
    
    // approximately every 2 seconds or so, print out the particle info
    if (millis() - timer > 2000){
      timer = millis();
      Serial.println();
      Serial.println(F("---------------------------------------"));
      Serial.println(F("Concentration Units (standard)"));
      Serial.println(F("---------------------------------------"));
      Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
      Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
      Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
      Serial.println(F("Concentration Units (environmental)"));
      Serial.println(F("---------------------------------------"));
      Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
      Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
      Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
      Serial.println(F("---------------------------------------"));
      Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
      Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
      Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
      Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
      Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
      Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
      Serial.println(F("---------------------------------------"));
   }
   delay(100);
}

/*****************************************************************/
int32_t getPDMwave(int32_t samples) {
  short minwave = 30000;
  short maxwave = -30000;

  while (samples > 0) {
    if (!samplesRead) {
      yield();
      continue;
    }
    for (int i = 0; i < samplesRead; i++) {
      minwave = min(sampleBuffer[i], minwave);
      maxwave = max(sampleBuffer[i], maxwave);
      samples--;
    }
    // clear the read count
    samplesRead = 0;
  }
  return maxwave - minwave;
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
