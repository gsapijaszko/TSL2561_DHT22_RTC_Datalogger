
// Compiled from Adafruit examples TSL2561, SD Data Logger and RealTimeClock1307 + DHT22
// G. Sapijaszko, 19.05.2016, v. 1.2

#include <Adafruit_TSL2561_U.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

DHT dht;

#define redLEDpin 2
#define greenLEDpin 3

#include <SPI.h>
/*ensure to install the SdFat Library from:
https://github.com/greiman/SdFat
*/
#include <SdFat.h>   //New SdFat library
SdFat SD;            //Add this line for backwards compatability with coding
// We are using Adafruit clone - so chipSelect = 10
const int chipSelect = 10;

/*
   Connections:
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
*/

#include <Wire.h>
#include <RealTimeClockDS1307.h>

char formatted[] = "00-00-00 00:00:00x";

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Displays basic information on TSL2591 sensor

void displaySensorDetails(void){
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    "));; Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    "));; Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(" lux");
  delay(500);
}

// Configures the gain and integration time for the TSL2561

void configureSensor(void) {
  // You can manually set the gain or enable auto-gain support - uncomment desired option
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  // Changing the integration time gives you better sensor resolution (402ms = 16-bit data)
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      // fast but low resolution
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  // medium resolution and speed   
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  // 16-bit data but slowest conversions

  // Update these displayed values to reflect what was set above!
  Serial.print  (F("Gain:         ")); Serial.println(F("Auto"));
  Serial.print  (F("Timing:       ")); Serial.println(F("402 ms"));
  Serial.print (F("-----------------------------------"));
 }
 
/**************************************************************************/

#define LOG_INTERVAL  10000 // ---------------------------------SENSOR READ / LOG DELAY 1 of 2
// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to
// the last 10 reads if power is lost but it uses less power and is much faster!

#define SYNC_INTERVAL 1000 //------------------------------SENSOR READ / LOG DELAY 2 of 2
// mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define WAIT_TO_START    0 // Wait for serial input in setup()

void error(char *str){
    Serial.print(F("error: "));
    Serial.println(str);

    // red LED indicates error
    digitalWrite(redLEDpin, HIGH);

    while (1);
 }

File SensorData; // Data object to write sensor data to

//=======================================================================


void setup() {
  dht.setup(5); // data pin 5

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  Serial.println(F("BME280-TSL2561 Data Logger"));

  pinMode(redLEDpin, OUTPUT); // debugging LED
  pinMode(greenLEDpin, OUTPUT); // debugging LED

  // Initialise TSL2561 sensor
  if (!tsl.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print(F("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!"));
    while (1);
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
  /* Setup the sensor gain and integration time */
  configureSensor();
  /* We're ready to go! */
  Serial.println("");

  // create a new file
  char filename[] = "LOGGER00.CSV";  // Basic Filename
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i / 10 + '0';
      filename[7] = i % 10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        SensorData = SD.open(filename, FILE_WRITE);
        break;  // leave the loop!
      }
    }
    if (! SensorData) {
      error("couldnt create file");
    }

    Serial.print(F("Logging data to: "));
    Serial.println(filename);
    Serial.println("-----------------------------------");
    digitalWrite(greenLEDpin, HIGH);  // Green light on

    // connect to RTC
    Wire.begin();
}

void loop() {
  delay(dht.getMinimumSamplingPeriod());
    
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
    
  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.println(dht.toFahrenheit(temperature), 1);

  // delay between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));
  digitalWrite(greenLEDpin, HIGH);

  // fetch the time
  RTC.readClock();
  RTC.getFormatted(formatted);

  SensorData.print(formatted);
  SensorData.print(", ");

  SensorData.print(temperature);
  SensorData.print(", ");

  SensorData.print(humidity);
  SensorData.print(", ");

  //-------------------------------------------------------------------
  // new TSL2561 sensor reading
  sensors_event_t event;
  tsl.getEvent(&event);

  // Display the results (light is measured in lux)
  if (event.light) {
    SensorData.print(event.light);
    SensorData.println(",");
  }
  else {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
    SensorData.print("0.00");
    SensorData.println(",");
  }
  //-------------------------------------------------------------------

  SensorData.flush();
  digitalWrite(greenLEDpin, LOW);

  Serial.print(formatted);
  Serial.println();
  Serial.print(temperature);
  Serial.println(" Degrees C");
  Serial.print(humidity);
  Serial.println(" % Relative Humidity");
  Serial.print(event.light);
  Serial.println(" Lux");
  Serial.println();
}
