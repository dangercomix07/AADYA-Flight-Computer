
/* AADYA THRUST TEST BENCH
** Author: Ameya Marakarkandy
** Last Updated: 19/07/2024

FreeRTOS based
1. Load Cell Reader
2. Serial Data Logger
3. Web Server Sender

To implement"
mutexes to safeguard critical sections
hardware timers for load cell reading
*/

// Including dependencies
#include<Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "soc/rtc.h"
#include "HX711.h"

// Core definitions (assuming you have dual-core ESP32)
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Settings
const char* ssid = "Galaxy A34 5G EB0E";
const char* password = "ameya010703";
static const uint8_t queue_len= 5; // Queue Length
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 16;
const int LOADCELL_SCK_PIN = 4;

// Globals
WebServer server(80);
long randomValue = 0;
static QueueHandle_t queue;

HX711 scale;
String known_weight = ""; 
float calib_factor = 0;


// Task Handles
static TaskHandle_t LoadCellReader = NULL;
static TaskHandle_t CommLink = NULL;
static TaskHandle_t DataLogger = NULL;

// Functions
void handleRoot() {
  long msg;
  if (xQueueReceive(queue, &msg, portMAX_DELAY) == pdTRUE){
    server.sendHeader("Access-Control-Allow-Origin", "*"); // Allow all origins
    server.send(200, "text/plain", String(msg));
  }
}

// Tasks

// LOAD CELL READER
void ReadLoadCell(void *parameters){
  long reading;
  while(1){
    //randomValue = random(0, 1000); // Generate a random value between 0 and 1000
    reading = scale.get_units();
    //randomValue++; //To test data received
    //xQueueSend(queue,&randomValue,portMAX_DELAY); 
    xQueueSend(queue,&reading,portMAX_DELAY); 
    //Avoid max delays, should use timeout but here I dont think it will be necessary
    vTaskDelay(100/portTICK_PERIOD_MS); // 10Hz update rate approx
  }

}

// SERIAL TERMINAL LOGGER
void LogData(void *parameters){
  // Local variable to store received data from queue
  long val;

  while(1){
    if(xQueueReceive(queue,&val,portMAX_DELAY)==pdTRUE){
      Serial.println(val);
    }
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
  
}

// WEB COMMUNICATIONS
void WebComms(void *parameters){
  long msg;
  while(1){
    server.handleClient();
    vTaskDelay(10/portTICK_PERIOD_MS);
  }
}

// SETUP TASK
void setup() {

  // Configure Serial
  Serial.begin(115200);

  // Wait some time to start so that we dont miss output
  vTaskDelay(1000/portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("------------AADYA-THRUST-TESTBENCH------------------");

  //Slow down the ESP32 Processor
  rtc_cpu_freq_config_t config;
  rtc_clk_cpu_freq_get_config(&config);
  rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &config);
  rtc_clk_cpu_freq_set_config_fast(&config);

  //Initialise the load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  //randomSeed(analogRead(0)); // Initialize the random number generator
  
  // Connect to WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Print the IP address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  queue= xQueueCreate(queue_len, sizeof(long));

  //Defining the root and the handler function
  server.on("/", handleRoot);

  //Start the server
  server.begin();
  Serial.println("Started Web Server");

  if (scale.is_ready()) {
    scale.set_scale();    
    Serial.println("Tare... remove any weights from the scale.");
    delay(5000);
    scale.tare();
    Serial.println("Tare done...");
    Serial.print("Place a known weight on the scale...");
    delay(5000);
    long reading = scale.get_units(10);
    float reading1 = static_cast<float>(reading);
    Serial.print("Result: ");
    Serial.println(reading);
    Serial.println("Enter known weight");
    while (Serial.available() == 0){}
    known_weight = Serial.readString();
    Serial.println(known_weight);
    float knownWeightValue = known_weight.toFloat();
    calib_factor = reading1/knownWeightValue;
    Serial.println("Calibration Fctor:");
    Serial.println(calib_factor);
  } 
  else {
    Serial.println("HX711 not found.");
  }

  vTaskDelay(1000/portTICK_PERIOD_MS);
  //calibration factor will be the (reading)/(known weight)
  scale.set_scale(calib_factor);
  //scale.set_scale(-471.497);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  Serial.println("Waiting to tare");
  delay(5000);

  scale.tare();               // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));   // print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5),1);        // print the average of 5 readings from the ADC minus tare weight, divided
            // by the SCALE parameter set with set_scale

  Serial.println("Readings:");

  Serial.println("---------------Setup-Complete---------------------");

  // Creating Tasks
  xTaskCreatePinnedToCore(
                        ReadLoadCell,
                        "Load Cell Reader",
                        2048,
                        NULL, //GAIN VALUE AS INPUT
                        1,
                        &LoadCellReader,
                        app_cpu);                      
  
  xTaskCreatePinnedToCore(
                        LogData,
                        "Serial Data Logger",
                        2048,
                        NULL,
                        1,
                        &DataLogger,
                        app_cpu);
  
  xTaskCreatePinnedToCore(
                        WebComms,
                        "Web Server Sender",
                        2048,
                        NULL, //GAIN VALUE AS INPUT
                        1,
                        &CommLink,
                        pro_cpu);   
  vTaskDelete(NULL);     
}

void loop() {
  // Do Nothing
}
