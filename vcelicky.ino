//Libs for sensors and modules
#include "DHT.h"
#include "arduinoFFT.h"  //frequency analysis
#include "Wire.h"        //I2C communication
#include "I2Cdev.h"      //I2C communication with MPU6050 - for easier reading of registers
#include "MPU6050.h"
#include "HX711.h"
#include <DFRobot_ENS160.h>

#include "string.h"

//Libs for arduino sleep
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


//DHT22
#define DHTTYPE DHT22  //sensor type
#define DnM_PIN 36     // for control of DHT22 and MAX9814

DHT dhtOut(12, DHTTYPE);
DHT dhtIn(13, DHTTYPE);


//MAX9814
#define MIC_PIN A5
#define SAMPLES 128
#define SAMPLING_FREQUENCY 8192


unsigned int samplingPeriod;
unsigned long microSeconds;


arduinoFFT FFT = arduinoFFT();
double vReal[SAMPLES];  //vector of real values
double vImag[SAMPLES];  //vector of imaginary values


//MPU6050
#define MOTION_LIMIT 0.2
MPU6050 accelgyro;


float default_accX = 0;
float default_accY = 0;
float default_accZ = 0;


//a7670
#define a7670 Serial1
#define fourG_PIN 34


//EMS40
HX711 scale;
#define DT_PIN 28
#define SCK_PIN 26
#define calibration_factor 1527.47

//ENS160
DFRobot_ENS160_I2C ENS160(&Wire, /*I2CAddr*/ 0x53);



//For arduino sleep between measurements
#define SLEEPING_ITERATIONS 112  //112 = (roughly 15 minutes)


//Size of cache
#define CACHE_NUMBER 100

struct CACHING {
  int8_t temperatureIn : 7;  //to measure temperature, 7 bits of an 8-bit signed int (-64 to +63) are enough
  int8_t temperatureOut : 7;
  uint8_t humidityIn : 7;  //for values ​​from 0 to 100, 7 bits of an 8-bit unsigned int (0 to 127) are enough
  uint8_t humidityOut : 7;
  uint16_t weight : 14;         //with 14 bits, we are able to store the weight up to 163.83 kg
  uint16_t peakFrequency : 12;  //maximum value that can fit into 12 bits is 4095.
  uint16_t TVOC : 16;
  uint16_t ECO2 : 16;
  bool movedHive : 1;           //true/false fits into 1 bit
  bool sent : 1;

  //constructor for setting of default values
  CACHING()
    : temperatureIn(0), temperatureOut(0), humidityIn(0), humidityOut(0), weight(0), peakFrequency(0), TVOC(0), ECO2(0), movedHive(false), sent(true) {}
};

//creating an array of structures
CACHING cacheMemory[CACHE_NUMBER];

//variables for the cache system
int lastWrite = -1;
int lastSend = -1;
uint64_t lastTimestamp = 0;
uint64_t currTimestamp = 0;






//Auxiliary functions for communication with the 4G module

//Reads incoming data from the serial line and prints it to the Serial monitor
void readResponse(unsigned long timeout = 2000) {
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (a7670.available()) {
      char c = a7670.read();
      Serial.write(c);
      start = millis();  // RESET timeout if response came
    }
  }
  Serial.println();
}

//Reads incoming data and returns it as a String
String readResponseAsString(unsigned long timeout = 2000) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (a7670.available()) {
      char c = a7670.read();
      response += c;
      start = millis();  // RESET timeout
    }
  }
  return response;
}

String readResponseAsStringFixed(unsigned long timeout = 2000) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (a7670.available()) {
      char c = a7670.read();
      response += c;
      // tu NEresetuj start
    }
  }
  return response;
}

//Sends AT command to module and clears buffer before sending
void sendCommand(String cmd, unsigned long sleep = 100) {
  while (a7670.available()) a7670.read();  // clear buffer
  a7670.println(cmd);
  delay(sleep);
}




//Measurement will be repeated when obtained value is invalid
float readWithRetry(float (*readFunc)(), float minValid, float maxValid, float errorValue) {
  byte tries = 0;
  float value;

  do {
    value = readFunc();  //call the function (e.g., readHumidity or readTemperature)
    if (value > minValid && value < maxValid) {
      return value;  //valid value → return immediately
    }
    tries++;
    delay(2000);  //wait 2 seconds because DHT22 can send data only once every 2 seconds or so
  } while (tries < 3);

  //if all attempts fail → return the errorValue
  return errorValue;
}




//Converts message to json
String messageConvert(int temperatureIn, int humidityIn, int temperatureOut, int humidityOut, bool movedHive, int peakFrequency, uint16_t weight, uint16_t TVOC, uint16_t ECO2, bool cached, uint64_t timestamp) {

  //change weight back to decimal format
  float decimalWeight = float(weight) / 100.0;

  // prepare a string with reserved memory
  String message;
  message.reserve(130);

  if (!cached) {
    // without timestamp (current live data)
    message = "{";
  } else {
    // cached (historical data with TS in ms)

    message = "{\"ts\":";
    message += u64ToString(timestamp);
    message += ", \"values\":{";
  }

  
  message += "\"tr\": "; //turnover
  message += (movedHive ? "true" : "false");
  message += ", \"f\": "; //frequency
  message += String(peakFrequency);
  message += ", \"w\": "; //weight
  message += String(decimalWeight, 2);
  message += ", \"tI\": "; //temperatureIn
  message += String(temperatureIn);
  message += ", \"hI\": "; //humidityIn
  message += String(humidityIn);
  message += ", \"tO\": ";
  message += String(temperatureOut);
  message += ", \"hO\": "; //humidityOut
  message += String(humidityOut);
  message += ", \"tv\": "; //TVOC
  message += String(TVOC);
  message += ", \"e\": "; //ECO2
  message += String(ECO2);

  if (!cached) {
    message += "}";
  } else {
    message += "}}";
  }

  return message;
}










//Check if module is connected to network
bool check_internet_connection() {

  //Serial.println(F("Kontrola či je zadaný pin"));

  sendCommand("AT+CPIN?");
  String pinStatus = readResponseAsString();
  Serial.println(pinStatus);

  if (pinStatus.indexOf("SIM PIN") >= 0) {
    Serial.println(F("SIM vyžaduje PIN, odosielam..."));
    sendCommand("AT+CPIN=5965", 5000);
    readResponse();
  }


  //Set module to LTE only
  //Serial.println(F("Nastavenie LTE only"));
  sendCommand("AT+CNMP=38");
  readResponse();

  //APN setting
  //Serial.println(F("Nastavenie APN"));
  sendCommand("AT+CGDCONT=1,\"IP\",\"internet\"");
  readResponse();

  //Print actual APN settings - dev command
  /*Serial.println(F("Kontrola nastaveného APN"));  
  sendCommand("AT+CGDCONT?");
  readResponse();*/

  //Set detailed print of CREG command - dev command
  /*Serial.println(F("Povolenie vypisu creg")); 
  sendCommand("AT+CREG=2");
  readResponse();*/

  //Print signal strength - dev command
  Serial.println(F("Sila signalu"));
  sendCommand("AT+CSQ");
  readResponse();

  //Print available networks - dev command
  Serial.println(F("Vypis dostupnych sieti"));
  sendCommand("AT+COPS?");
  readResponse();

  //Print if module is registered in network - dev command
  Serial.println(F("Kontrola siete: AT+CREG?"));
  sendCommand("AT+CREG?");
  readResponse();

  //Connect module to GRPS
  //Serial.println(F("Attach na GPRS: AT+CGATT=1"));
  sendCommand("AT+CGATT=1");
  readResponse();


  for (int i = 0; i < 3; i++) {
    Serial.println("Pokus o aktiváciu PDP (#" + String(i + 1) + ")...");

    sendCommand("AT+CGACT=1,1");
    readResponse();


    sendCommand("AT+CGACT?");
    String response = readResponseAsString();
    Serial.println("CGACT? ➜ " + response);

    if (response.indexOf("1,1") >= 0) {
      Serial.println(F("Úspešne pripojené."));
      return true;
    }
    delay(2000);  // short pause before next attempt
  }

  Serial.println(F("Zlyhalo pripojenie po 3 pokusoch."));
  return false;
}






//wait for HTTP response and check if code is 200
bool waitForHttpActionResponse(unsigned long timeout = 15000) {
  String resp = "";
  unsigned long start = millis();

  //Serial.println(F("Čakám na +HTTPACTION..."));

  while (millis() - start < timeout) {
    // čítaj a ukladaj živú odpoveď, aj ju vypisuj
    while (a7670.available()) {
      char c = a7670.read();
      resp += c;
      Serial.write(c);  // live debug
    }

    // Skontroluj, či už riadok HTTPACTION prišiel
    if (resp.indexOf("+HTTPACTION:") != -1) {
      delay(200);                              
      resp += readResponseAsStringFixed(300);  // read the remainder without resetting the timeout

      Serial.println();  

      if (resp.indexOf(",200,") != -1) {
        return true;
      } else {
        Serial.println(resp);
        return false;
      }
    }
  }

  Serial.println(F("⏱ Čakanie na +HTTPACTION vypršalo."));
  return false;
}



//Send message to server
bool sendMessage(String message, int temperatureIn, int humidityIn, int temperatureOut, int humidityOut, bool movedHive, int peakFrequency, uint16_t weight, uint16_t TVOC, uint16_t ECO2) {

  //if module is unable to connect to network data will be stored in cache
  if (!check_internet_connection()) {

    if (lastWrite == CACHE_NUMBER - 1) {
      overfill();  //full cache - FIFO function
    } else {
      lastWrite++;  //save data to cache
    }

    cacheMemory[lastWrite].temperatureIn = temperatureIn;
    cacheMemory[lastWrite].temperatureOut = temperatureOut;
    cacheMemory[lastWrite].humidityIn = humidityIn;
    cacheMemory[lastWrite].humidityOut = humidityOut;
    cacheMemory[lastWrite].peakFrequency = peakFrequency;
    cacheMemory[lastWrite].movedHive = movedHive;
    cacheMemory[lastWrite].weight = weight;
    cacheMemory[lastWrite].TVOC = TVOC;
    cacheMemory[lastWrite].ECO2 = ECO2;
    cacheMemory[lastWrite].sent = false;

    return false;
  }

  //initializing of HTTP service
  sendCommand("AT+HTTPINIT", 1000);
  readResponse();

  //Set which PDP context will be used (APN connection)
  sendCommand("AT+HTTPPARA=\"CID\",1", 500);
  readResponse();

  //Defines the target URL for an HTTP request
  sendCommand("AT+HTTPPARA=\"URL\",\"http://147.175.150.184:8080/api/v1/o84l0slh4nri8ptjy1sq/telemetry\"", 500);
  readResponse();


  //Set the Content-Type header to "application/json"
  sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 500);
  readResponse();



  byte attempt = 0;


  while (attempt < 3) {
    //Prepares the module to receive data that will be sent in an HTTP POST request.
    sendCommand("AT+HTTPDATA=" + String(message.length()) + ",10000", 100);

    //waiting to "DOWNLOAD" prompt
    unsigned long start = millis();
    while (millis() - start < 3000) {
      if (a7670.available()) {
        String r = readResponseAsString(500);
        if (r.indexOf("DOWNLOAD") >= 0) break;
      }
    }


    Serial.println(F("Odosielam dáta..."));
    a7670.print(message);
    delay(500);


    //Serial.println(F("Spúšťam HTTPACTION=1 (POST)..."));
    while (a7670.available()) a7670.read();
    sendCommand("AT+HTTPACTION=1");




    if (!waitForHttpActionResponse()) {
      Serial.println(F("HTTP 200 sa nenašlo – retry..."));
      attempt++;
      continue;
    }

    //If successful → possibly send cache and end connection
    if (lastWrite > -1) {
      sendCache();
    }

    sendCommand("AT+HTTPTERM");
    while (a7670.available()) a7670.read();

    Serial.println(F("Úspešne odoslané a ukončené spojenie."));
    return true;
  }


  Serial.println(F("Nepodarilo sa odoslať ani po 4 pokusoch."));




  //if the message could not be sent, we will save the data to cache
  if (lastWrite == CACHE_NUMBER - 1) {
    overfill();  //full cache - FIFO function
  } else {
    lastWrite++;  //save data to cache
  }

  cacheMemory[lastWrite].temperatureIn = temperatureIn;
  cacheMemory[lastWrite].temperatureOut = temperatureOut;
  cacheMemory[lastWrite].humidityIn = humidityIn;
  cacheMemory[lastWrite].humidityOut = humidityOut;
  cacheMemory[lastWrite].peakFrequency = peakFrequency;
  cacheMemory[lastWrite].movedHive = movedHive;
  cacheMemory[lastWrite].weight = weight;
  cacheMemory[lastWrite].TVOC = TVOC;
  cacheMemory[lastWrite].ECO2 = ECO2;
  cacheMemory[lastWrite].sent = false;

  Serial.println(F("Data cached."));
  return false;
}




//✔️HOTOVA✔️
uint64_t parseUInt64(String s) {
  uint64_t value = 0;
  for (int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') {
      value = value * 10 + (c - '0');
    } else {
      break;
    }
  }
  return value;
}


String u64ToString(uint64_t value) {
  if (value == 0) return "0";

  char buf[21]; // 20 digits + '\0' (uint64 max: 18446744073709551615)
  int i = 20;
  buf[i] = '\0';

  while (value > 0 && i > 0) {
    uint8_t digit = value % 10;
    value /= 10;
    buf[--i] = '0' + digit;
  }

  return String(&buf[i]);
}



//Get the current Unix timestamp from the HTTP RPC response
uint64_t getTimestamp() {
  //terminate original HTTP connection
  sendCommand("AT+HTTPTERM");

  while (a7670.available()) a7670.read();


  sendCommand("AT+HTTPINIT", 1000);
  readResponse();

  sendCommand("AT+HTTPPARA=\"CID\",1", 500);
  readResponse();

  //enter the url parameter (url for rpc api)
  sendCommand("AT+HTTPPARA=\"URL\",\"http://147.175.150.184:8080/api/v1/o84l0slh4nri8ptjy1sq/rpc\"", 500);
  readResponse();

  sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 500);
  readResponse();



  //sent JSON file is an RPC request to get a timestamp
  String message = "{\"method\":\"getCurrentTime\",\"params\":{}}";


  byte attempt = 0;
  String response;

  while (attempt < 3) {

    sendCommand("AT+HTTPDATA=" + String(message.length()) + ",10000", 100);

    //waiting to "DOWNLOAD" prompt
    unsigned long start = millis();
    while (millis() - start < 3000) {
      if (a7670.available()) {
        String r = readResponseAsString(500);
        if (r.indexOf("DOWNLOAD") >= 0) break;
      }
    }


    Serial.println(F("Odosielam dáta..."));
    a7670.print(message);
    delay(500);

    //send request to ThingsBoard
    //Serial.println(F("Spúšťam HTTPACTION=1 (POST)..."));
    sendCommand("AT+HTTPACTION=1");

    String actionResp = readResponseAsString(5000);

    /*int method, httpCode, length;
    sscanf(actionResp.c_str(), "+HTTPACTION: %d,%d,%d", &method, &httpCode, &length);

    Serial.print("Dĺžka JSON: ");
    Serial.println(length);*/

    sendCommand("AT+HTTPREAD=0," + String(23), 500);

    String response = readResponseAsString(5000);

    Serial.println(response);

    // Nájdeme "timestamp":
    int pos = response.indexOf("\"time\":");

    if (pos != -1) {

      pos += 7;  // move behind "time":
      int end = response.indexOf("}", pos);
      String t = response.substring(pos, end);
      t.trim();
      uint64_t timestamp = parseUInt64(t);


      char tsBuf[24];
      Serial.println("debug");
      String tsStr = u64ToString(timestamp);


      sendCommand("AT+HTTPTERM");
      while (a7670.available()) a7670.read();
      return timestamp;  // return the entire 13-digit number
    }

    Serial.println(F("Timestamp sa nenasiel, opakujem..."));
    attempt++;
  }


  sendCommand("AT+HTTPTERM");
  while (a7670.available()) a7670.read();
  return 0;
}









//leftBound - position of the latest "old" record or the beginning of the cache
void moveCache(int leftBound) {
  Serial.println(F("Moving cache.."));

  //using firstUnsent we find the closest "new" record
  int firstUnsent = leftBound + 1;

  //if the next record is not sent - the cache has not moved, no space removal is needed
  if (!cacheMemory[firstUnsent].sent) {
    return;
  }
  //we start from position and look for the first unsent record or the end of the cache
  while (cacheMemory[firstUnsent].sent && firstUnsent != CACHE_NUMBER) {
    firstUnsent++;
  }

  //if we have reached the end of the cache list, there is nothing to move
  //set lastWrite to leftBound, because we sent everything that was written.
  if (firstUnsent == CACHE_NUMBER) {
    lastWrite = leftBound;
    return;
  }

  int newPlacement = leftBound + 1;

  //move the unsent records to the left (newPlacement) and set their previous places to sent
  for (int i = firstUnsent; i <= lastWrite; i++) {
    if (!cacheMemory[i].sent) {
      cacheMemory[newPlacement] = cacheMemory[i];
      cacheMemory[newPlacement].sent = false;
      newPlacement++;
    }
  }

  //at the end, the position of the last written record will be moved to the newPlacement position, because we moved the cache
  //-1, because newPlacement is 1 greater than the last (latest) number
  lastWrite = newPlacement - 1;
}






//Resolves cache overflow: discards oldest record and moves lastSend
void overfill() {
  Serial.println(F("!!!!!!!!!!OVERFILL!!!!!!!!!!"));
  //all records are moved to the left, the first record from the left (oldest) is dropped
  for (int i = 0; i < CACHE_NUMBER - 1; i++) {
    cacheMemory[i] = cacheMemory[i + 1];
  }
  lastSend--;

  //lastSend will also be moved along with the records, when it reaches -1, records that were not sent will be dropped
  //so we set lastSend to -1 and lastTimestamp to 0
  if (lastSend < 0) {
    lastSend = -1;
    lastTimestamp = 0;
  }
}


void sendCache() {

  const uint64_t STEP = 896000ULL;

  check_internet_connection();
  uint64_t nowTs = getTimestamp();
  if (nowTs == 0) return;

  nowTs -= STEP;

  if (lastTimestamp != 0)
      nowTs = lastTimestamp;


  // Starting from the newest
  int index = (lastSend == -1 ? lastWrite : lastSend);

  while (index >= 0) {

    if (cacheMemory[index].humidityIn == 0) {
      index--;
      continue;
    }

    // correct timestamp for this record
    uint64_t tsForRecord = nowTs - (uint64_t)( (lastWrite - index) * STEP );

    if (sendCachedMessage(index, tsForRecord)) {
      index--;
    } else {
      lastSend = index;
      lastTimestamp = nowTs;
      return;
    }
  }

  lastWrite = -1;
  lastSend = -1;
  lastTimestamp = 0;

  Serial.println("CACHE OK");
}






//Sends cached messages
bool sendCachedMessage(int index, uint64_t timestamp) {

  // Convert data from cache → JSON for ThingsBoard (TS in milliseconds)
  String message = messageConvert(
    cacheMemory[index].temperatureIn,
    cacheMemory[index].humidityIn,
    cacheMemory[index].temperatureOut,
    cacheMemory[index].humidityOut,
    cacheMemory[index].movedHive,
    cacheMemory[index].peakFrequency,
    cacheMemory[index].weight,
    cacheMemory[lastWrite].TVOC,
    cacheMemory[lastWrite].ECO2,
    true,      // cached mode
    timestamp  
  );

  Serial.println("Mess: " + message);

  //rest of the function inspired by sendMessage() function

  //if there is no connection, return false - cannot send
  if (!check_internet_connection()) {
    return false;
  };

  sendCommand("AT+HTTPINIT", 1000);
  readResponse();

  sendCommand("AT+HTTPPARA=\"CID\",1", 500);
  readResponse();

  sendCommand("AT+HTTPPARA=\"URL\",\"http://147.175.150.184:8080/api/v1/o84l0slh4nri8ptjy1sq/telemetry\"", 500);
  readResponse();

  sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 500);
  readResponse();

  byte attempt = 0;
  while (attempt < 4) {
    Serial.println("Mess: " + message);
    sendCommand("AT+HTTPDATA=" + String(message.length()) + ",10000", 100);

    unsigned long start = millis();
    while (millis() - start < 3000) {
      if (a7670.available()) {
        String r = readResponseAsString(500);
        if (r.indexOf("DOWNLOAD") >= 0) break;
      }
    }

    Serial.println(F("Odosielam dáta..."));
    a7670.print(message);
    delay(500);

    sendCommand("AT+HTTPACTION=1");


    if (waitForHttpActionResponse()) {
      sendCommand("AT+HTTPTERM");
      while (a7670.available()) a7670.read();

      return true;
    }

    attempt++;
  }

  if (attempt == 3) {
    sendCommand("AT+HTTPTERM");
    while (a7670.available()) a7670.read();

    return false;
  }
}






//Calibration of MPU6050
void mpu6050_Calibrate() {
  accelgyro.CalibrateGyro();
  accelgyro.CalibrateAccel();
  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);
  default_accX = ax / 16384.0;
  default_accY = ay / 16384.0;
  default_accZ = az / 16384.0;
  Serial.print(F("Calibration aX = "));
  Serial.print(default_accX);
  Serial.print(F(" | aY = "));
  Serial.print(default_accY);
  Serial.print(F(" | aZ = "));
  Serial.println(default_accZ);
}






//Returns true if the MPU6050 has moved beyond the defined motion threshold
bool isMoved(int16_t ax, int16_t ay, int16_t az) {
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float changeX = fabs(default_accX - ax_g);
  float changeY = fabs(default_accY - ay_g);
  float changeZ = fabs(default_accZ - az_g);

  return (changeX > MOTION_LIMIT || changeY > MOTION_LIMIT || changeZ > MOTION_LIMIT);
}











void enterSleep(void) {


  for (int a = 0; a < SLEEPING_ITERATIONS; a = a + 1) {

    //Serial.println("I will go sleep now");
    //delay(1000); //allow for serial print to complete

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  //set sleep mode to lowest power consumption possible
    sleep_enable();                       // tells the core that it can sleep

    //now enter sleep mode
    sleep_mode();

    //program will continue from here after the WDT timeout
    sleep_disable();  //exit sleep mode

    //re-enable the peripherals
    power_all_enable();

    //Serial.println("Good morning, good morning :-) ");
  }


  //delay(2000);
}

//Watchdog Timer interrupt – wakes Arduino from sleep, nothing else needs to be there
ISR(WDT_vect) {
}










void setup() {
  Serial.begin(9600);
  a7670.begin(115200);

  Wire.begin();

  //MAX9814 - calculation of sampling period
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY));


  //MPU6050
  while (!Serial)
    ;

  accelgyro.initialize();

  Serial.println(F("Initializing MPU6050..."));

  if (!accelgyro.testConnection()) {
    Serial.println(F("MPU6050 connection failed!"));
    while (1)
      ;
  }

  Serial.println(F("MPU6050 initialized."));
  mpu6050_Calibrate();

  //ENS160
  while( NO_ERR != ENS160.begin() ){
    Serial.println(F("Communication with device failed, please check connection"));
    delay(3000);
  }
  Serial.println(F("Begin ok!"));

  ENS160.setPWRMode(ENS160_STANDARD_MODE);
    /**
   * Set power mode
   * mode Configurable power mode:
   *   ENS160_SLEEP_MODE: DEEP SLEEP mode (low power standby)
   *   ENS160_IDLE_MODE: IDLE mode (low-power)
   *   ENS160_STANDARD_MODE: STANDARD Gas Sensing Modes
   */


  //a7670
  pinMode(fourG_PIN, OUTPUT);
  digitalWrite(fourG_PIN, HIGH);


  //DnM
  pinMode(DnM_PIN, OUTPUT);
  digitalWrite(DnM_PIN, LOW);

  delay(200);  
  Serial.println(F("HX711 - warming up..."));
  // wake up the converter
  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, LOW); 

  delay(5000);


  scale.begin(DT_PIN, SCK_PIN, 128);
  scale.set_scale();
  scale.tare();  // zero point setting

  delay(200);

  pinMode(DnM_PIN, OUTPUT);
  digitalWrite(DnM_PIN, HIGH);





  //clear the reset flag
  MCUSR &= ~(1 << WDRF);

  //in order to change WDE or the prescaler, we need to set WDCE (This will allow updates for 4 clock cycles)
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  //set new watchdog timeout prescaler value
  WDTCSR = 1 << WDP0 | 1 << WDP3;  //8.0 seconds // NOTE from JP: this is not exactly 8.0 !, also seems to pauze system clock during sleep

  //Enable the WD interrupt (note no reset)
  WDTCSR |= _BV(WDIE);


  /*Serial.println(F("SRAM check:"));
  extern int __heap_start, *__brkval;
  int v;
  int freeMem = (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
  Serial.println(freeMem);*/
  delay(200000);
}





void loop() {
  int humidityOut = 0;
  int temperatureOut = 0;
  int humidityIn = 0;
  int temperatureIn = 0;
  uint16_t TVOC = 0;
  uint16_t ECO2 = 0;


  bool movedHive = false;


  //a7670 on
  digitalWrite(fourG_PIN, LOW);
  Serial.println(F("4G module POWER ON"));

  //ENS160 on
  ENS160.setPWRMode(ENS160_STANDARD_MODE);

  //DHT22 measuring of temperature and humidity
  digitalWrite(DnM_PIN, LOW);
  Serial.println(F("DnM POWER ON"));

  //wake up the converter
  digitalWrite(SCK_PIN, LOW); 
  delay(2000);


  dhtOut.begin();
  dhtIn.begin();



  float humidityOutF = readWithRetry(
    []() {
      return dhtOut.readHumidity();
    },           //function that is called
    0.0, 100.0,  //valid range
    101.0        //error value
  );

  float temperatureOutF = readWithRetry(
    []() {
      return dhtOut.readTemperature();
    },
    -50.0, 80.0,
    -64.0);

  float humidityInF = readWithRetry(
    []() {
      return dhtIn.readHumidity();
    },
    0.0, 100.0,
    101.0);

  float temperatureInF = readWithRetry(
    []() {
      return dhtIn.readTemperature();
    },
    -50.0, 80.0,
    -64.0);


  humidityOut = (int)round(humidityOutF);
  temperatureOut = (int)round(temperatureOutF);

  humidityIn = (int)round(humidityInF);
  temperatureIn = (int)round(temperatureInF);

  //MAX9814 measuring of frequency
  for (int i = 0; i < SAMPLES; i++) {
    microSeconds = micros();
    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;
    while (micros() - microSeconds < samplingPeriod) {}
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  //calibration equation for actual frequency
  peak = (peak - 0.875367) / 1.08148;

  //convert to integer
  int peakFrequency = peak;

  //clamp frequency to valid range (0–4095 for 12-bit storage)
  if (peakFrequency > 4095) {
    peakFrequency = 4095;
  } else if (peakFrequency < 0) {
    peakFrequency = 0;
  }


  //EMS40 measuring of weight
  delay(3000);  //other part of delay is delay in the start of DnM

  scale.set_scale(calibration_factor);

  // Získaj hmotnosť (v kilogramoch)
  float weight = scale.get_units(30);  // priemer z 5 meraní

  if (weight > 0) {
    weight = weight * 100;
  } else {
    weight = 0;
  }

  //set converter to sleep
  digitalWrite(SCK_PIN, HIGH);
  delay(100);

  digitalWrite(DnM_PIN, HIGH);
  Serial.println(F("DnM POWER OFF"));




  //MPU6050 chcecking if hive is rolled
  accelgyro.setSleepEnabled(0);
  delay(50);

  int16_t ax, ay, az;
  accelgyro.getAcceleration(&ax, &ay, &az);
  movedHive = isMoved(ax, ay, az);

  accelgyro.setSleepEnabled(1);


  //ENS160 measuring
  ENS160.setTempAndHum(/*temperature=*/temperatureInF, /*humidity=*/humidityInF);

  delay(200);

  while (ENS160.getENS160Status() != 0) {
    delay(200);
  }

  TVOC = ENS160.getTVOC();

  ECO2 = ENS160.getECO2();

  ENS160.setPWRMode(ENS160_IDLE_MODE);





  //Print of values
  Serial.print(F("Humidity Out [%]: "));
  Serial.print(humidityOut);
  Serial.print(F(" Temperature Out [C]: "));
  Serial.print(temperatureOut);

  Serial.print(F(" Humidity In [%]: "));
  Serial.print(humidityIn);
  Serial.print(F(" Temperature In [C]: "));
  Serial.print(temperatureIn);
  Serial.println();


  Serial.print(F("Peak frequency [Hz]: "));
  Serial.print(peakFrequency);
  Serial.println();


  Serial.print(F("Weight [kg]: "));
  Serial.print(weight / 100);
  Serial.println();

  Serial.print(F("TVOC [ppb]: "));
  Serial.print(TVOC);
  Serial.print(F(" ECO2 [ppm]: "));
  Serial.print(ECO2);
  Serial.println();

  Serial.print(F("Is hive moved?: "));
  if (movedHive) Serial.println(F("YES"));
  else Serial.println(F("NO"));
  Serial.println();


  //first false means we don't cache, second false can also be true (indicates which time is used for caching)
  String message = messageConvert(temperatureIn, humidityIn, temperatureOut, humidityOut, movedHive, peakFrequency, weight, TVOC, ECO2, false, false);

  //nepotrebne odstranit ked bude sa vracat kod 200
  //Serial.println(message);
  //Serial.print(F("message length: "));
 // Serial.println(message.length());

  //sending message
  delay(20000);  //delay for waking of 4g module

  Serial.println(F("Zapiname CFUN!!"));
  sendCommand("AT+CFUN=1", 5000);
  readResponse();

  if (sendMessage(message, temperatureIn, humidityIn, temperatureOut, humidityOut, movedHive, peakFrequency, weight, TVOC, ECO2)) Serial.println(F("S Message send."));
  else Serial.println(F("F Message sending failed."));


  digitalWrite(fourG_PIN, LOW);
  Serial.println(F("4G module POWER OFF"));


  Serial.println(F("-------------------------Sleeping Start-------------------------"));

  delay(100);

  enterSleep();

  Serial.println(F("-------------------------Sleeping End-------------------------"));
}
