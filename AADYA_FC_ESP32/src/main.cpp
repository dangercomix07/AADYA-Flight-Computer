#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SdFat.h>

SdFat SD;
const int chipSelectPin = 5; 
File dataFile;

unsigned long startTime;
const unsigned long duration = 60 * 1000; // 60 seconds (one minute)

Adafruit_BMP280 bmp; 
Adafruit_MPU6050 mpu;

void listFiles(const char *dirname);

#define SEA_LEVEL_PRESSURE 1013.25 //hectoPascal (milibars)

//////////////////////////////////SETUP////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  while ( !Serial ) delay(100);   // wait for native usb
  
  //Initialisation of Sensor
  unsigned mpu_init,bmp_init;
  int mpu_status,bmp_status;

  mpu_status = 1;
  bmp_status = 1;

  try{
  bmp_init = bmp.begin(0x76);
  mpu_init = mpu.begin(0x68);

  if(!mpu_status){
    throw mpu_status = 0;
  }

  if(!bmp_status){
    throw bmp_status = 0;
  }
  
  /*
  if (!mpu_status) {
    Serial.println("Failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }
  
  if (!bmp_status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1){
      delay(10);
    }
  }
  */

  ////// SENSOR-CONFIGS ///////////////////

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  //Default settings from datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,    // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,   // Pressure oversampling 
                  Adafruit_BMP280::FILTER_X16,     // Filtering. 
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time. 
  ///*
 
  }       
  catch(int mpu_status){

    if(mpu_status==0){
      Serial.println("Failed to find MPU6050");
    }

    if(bmp_status==0){
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
      Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
    }
    Serial.println("Exception");
  }




  if (!SD.begin(chipSelectPin, 10000000)) { // 10 MHz SPI speed
        Serial.println("SD card initialization failed!");

        return;
    }
    Serial.println("SD card initialized successfully.");

    // List files on the SD card
    listFiles("/");
  
  //*/
  // Open a new file for data logging
  dataFile = SD.open("sensor_data.csv", FILE_WRITE);
  if (!dataFile) {
    Serial.println("Error opening data file.");
    while (1);
  }

  // Write CSV header
  dataFile.println("Timestamp (ms),Acceleration X (m/s^2),Acceleration Y (m/s^2),Acceleration Z (m/s^2),Pressure (Pa)");
  delay(1000); //Waiting for 1s for file to open
  // Record the start time
  startTime = millis();
}

/////////////////////////////////MAIN-LOOP////////////////////////////////////////////////////////
void loop() {
    // Check if the elapsed time has reached the desired duration
    if (millis() - startTime >= duration) {
      // Close the file before exiting the program
      dataFile.close();
      Serial.println("Data logging complete. Program will now exit.");
      while (1); // End the program
    }

    sensors_event_t a, g, temp;   
    mpu.getEvent(&a, &g, &temp);
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float Pressure = bmp.readPressure(); //Pa

    ///*
    Serial.print(az);
    Serial.println(" m/s^2");

    
    Serial.print(Pressure/100.0F); //Reads pressure in milibars/hectopascals
    Serial.println(" hPa");


    Serial.print(bmp.readAltitude(SEA_LEVEL_PRESSURE)); //Has to be adjusted locally
    Serial.println();
    //*/
    
    
    // Get timestamp
    unsigned long timestamp = millis();

    // Log data to the SD card
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.print(ax);
    dataFile.print(",");
    dataFile.print(ay);
    dataFile.print(",");
    dataFile.print(az);
    dataFile.print(",");
    dataFile.println(Pressure);

    //Sampling rate = 10Hz
    delay(100);
}

/////////////////////////////FUNCTIONS//////////////////////////////////////////////

///*
void listFiles(const char *dirname) {
    Serial.println("Listing files in root directory:");
    File root = SD.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    while (true) {
        File entry = root.openNextFile();
        if (!entry) {
            break;
        }
        Serial.print("  - ");
        Serial.println(entry.name());
        entry.close();
    }
}
//*/
