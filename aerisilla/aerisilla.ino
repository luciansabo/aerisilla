/**
 * Aerisilla - IoT Device for Air Quality Monitoring
 * (c) 2019 Lucian SABO <luciansabo@gmail.com>
 * 
 * Main parts:
 * 
 *  • ESP8266 based MCU like NodeMCU
 *  • CCS811 VOC Sensfile:///home/luci/projects/electronics/aerisilla/aerisilla/BlynkLogger.h
or from Sparkfun (Adafruit sells one too and should be compatible with the code)
 *  • Sharp GP2Y1014AU0F Dust Sensor
 *  • DHT22 Temperature and Humidity Sensor
 *  • 128x32 SSD1306 OLED screen
 *  
 * Features:

    • Measures Total Volatile Organic Compounds  (TVOCs), provides 1 minute average of equivalent carbon dioxide (eCO2)
    • Measures dust level from large house dust to microscopic particles of 0.5 microns like bacteria, polen, mold and cigar smoke.
      Provides 1 minute average dust density .
    • Measures Temperature in Celsius and Relative Humidity
    • Shows information on the 128x32 pixels monochrome OLED display
    • Connects to internet by WiFI and sends real-time data to the cloud using Blynk.
      The Blynk mobile app project shows a dashboard with nice history graphs
    • Big multi-function push button (control display, switch lights using a relay, etc).
    • Red status led alerting different abnormal conditions
    • Two 3.3V logical outputs on the back which can be used to control anything from lights to air conditioning or fans using relays,
      transistors, mosfets or IR leds.
      
MIT License

Copyright (c) 2019 Lucian Sabo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
#include <lwip/netif.h>
#include <lwip/etharp.h>

#include <SparkFunCCS811.h>
#include <GP2YDustSensor.h>
#include <DHTesp.h>

// OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

#include <map>

// Utils
#include "BlynkLogger.h"

#include "Config.h"

#define CCS811_ADDR 0x5B // Default I2C Address
//#define CCS811_ADDR 0x5A //Alternate I2C Address
#define SSD1306_I2C_ADDRESS 0x3C

CCS811 vocSensor(CCS811_ADDR);
GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1014AU0F, SHARP_LED_PIN, SHARP_VO_PIN, dustRunningAverageCount);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial
#define BLYNK_HEARTBEAT      30 // heartbeat every 30s
#define BLYNK_MAX_SENDBYTES 1200

enum statuses {
  STATUS_OK,
  STATUS_NOTICE,
  STATUS_WARNING,
  STATUS_DANGER
} envStatus;

char logBuffer[512];
char displayBuffer[100];

char vocStatusBuffer[125];
char dustStatusBuffer[125];

// vars
DHTesp dht;
float temperature;
float humidity;
float heatIndex;

int16_t co2RunningAverageBuffer[co2RunningAverageCount];
uint16_t nextCO2RunningAverageCounter = 0;
uint16_t dustAverage, co2Average;
bool isVOCSensorReady = false;

// the timer object
BlynkTimer blynkTimer;
WidgetTerminal terminal(V7);
BlynkLogger blynkLogger(&terminal);

uint16_t wifiErrors = 0;
uint16_t blynkErrors = 0;
uint16_t blynkRestoreFailedAttempts = 0;

// --- debounce
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
// ---

void logoScreen()
{
  display.setFont(&FreeSansBold12pt7b);
  display.setTextSize(1);
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0, 24);             // Start at top-left corner
  display.print("AERISILLA");
  display.display();
  display.setFont(&FreeSans9pt7b);
}

void indoorTemperatureScreen()
{
  display.setCursor(10, 15);
  sprintf(displayBuffer, "%.1f *C   %.1f%%", temperature, humidity);
  display.print(displayBuffer);
  display.display();
}

void vocScreen()
{
  sprintf(displayBuffer, "eCO2 %d ppm", co2Average);
  
  display.setCursor(0, 15);
  display.print(displayBuffer);
  display.display();
}

void dustScreen()
{
  sprintf(displayBuffer, "Dust %d ug/m3", dustAverage);
  
  display.setCursor(10, 15);
  display.print(displayBuffer);
  display.display();
}

// screens
typedef void (*ScreenFnPtr)();
std::map<uint8_t, ScreenFnPtr> screenMap;

uint8_t currentScreenIndex = 0;

// ---------------------------------------------------

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  blynkLogger.debug("setup(): IndoorMonitor started.");

  char errorBuffer[255] = "";
  CCS811Core::status returnCode = vocSensor.begin();
  
  if (returnCode != CCS811Core::SENSOR_SUCCESS) {
    sprintf(errorBuffer, "vocSensor.begin() returned with an error: %d", returnCode);
  } else {
    // set drive mode
    // mode 1: 1s (default), mode2: 10s mode3: 60s
    vocSensor.setDriveMode(1);
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) { // Address 0x3C for 128x32
    strcpy(errorBuffer, "SSD1306 OLED allocation failed");
  }

  display.clearDisplay();
  logoScreen();
  
  screenMap[0] = vocScreen;
  screenMap[1] = dustScreen;
  screenMap[2] = indoorTemperatureScreen;
  screenMap[3] = logoScreen;

  for (uint16_t i = 0; i < co2RunningAverageCount; i++) {
    co2RunningAverageBuffer[i] = -1;
  }

  pinMode(INDICATOR_LED_PIN, OUTPUT);
  digitalWrite(INDICATOR_LED_PIN, HIGH);
  pinMode(DHT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // Initialize temperature sensor
  dht.setup(DHT_PIN, DHTesp::DHT22);
  dustSensor.begin();
  
  wifiSetup();
  blynkSetup();
  
  blynkTimer.setInterval(VOC_READING_INTERVAL, readVoc);
  blynkTimer.setInterval(60000, minuteCron);
  blynkTimer.setInterval(BLYNK_CHECK_INTERVAL, blynkWatchdog);   // check connection to blynk server 
  blynkTimer.setInterval(WIFI_CHECK_INTERVAL, wifiWatchdog);   // check wifi connection 
  blynkTimer.setInterval(TEMP_READING_INTERVAL, readTempAndHumidity);
  blynkTimer.setInterval(DUST_READING_TNTERVAL, readDustDensity);
  int displayHandlerTimerId = blynkTimer.setInterval(DISPLAY_CHANGE_INTERVAL, displayHandler);

  if (strcmp(errorBuffer, "")) {
    blynkLogger.fatal(errorBuffer);
  }
    
  blynkLogger.info("setup(): IndoorMonitor ready.");
  digitalWrite(INDICATOR_LED_PIN, LOW);
}

void displayHandler()
{
  display.clearDisplay();
  screenMap[currentScreenIndex]();
  
  currentScreenIndex++;
  if (currentScreenIndex >= screenMap.size()) {
    currentScreenIndex = 0;
  }
}



void readDustDensity()
{
  uint16_t dustDensity = dustSensor.getDustDensity();
  
  sendGratuitousARP();
  if (Blynk.connected()) {
    Blynk.virtualWrite(BLYNK_REALTIME_DUST_PIN, dustDensity);
  }
  
  sprintf(logBuffer, "Dust Density: %d ug/m3", dustDensity);
  blynkLogger.debug(logBuffer);
}

void loop()
{
  Blynk.run();
  blynkTimer.run();
  
  bool buttonPressed = !digitalRead(BUTTON_PIN);
  if (buttonPressed != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (buttonPressed != buttonState) {
      buttonState = buttonPressed;

      // only toggle the LED if the new button state is HIGH
      if (buttonPressed) {
        displayHandler();
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = buttonPressed;
}

void wifiSetup()
{
  sprintf(logBuffer, "wifiSetup(): Connecting to %s", ssid);
  blynkLogger.debug(logBuffer);
  WiFi.mode(WIFI_STA);
  
  // Static IP address configuration
  WiFi.persistent(false);
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(DEVICE_NAME); 
  WiFi.config(staticIP, dns, gateway, subnet);
  WiFi.begin(ssid, pass);
  wifiWaitForConnection();
}

void wifiWaitForConnection()
{
  int startMillis = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(WIFI_WAIT_DELAY);
    if ((millis() - startMillis) > WIFI_CONNECTION_TIMEOUT) {
      blynkLogger.notice("wifiSetup(): Wifi connection timeout.");
      break;            
    }
  }

  Serial.println("");
  
  if (WiFi.status() != WL_CONNECTED) {
    blynkLogger.fatal("wifiSetup(): Could not connect to network");
  } else {
    blynkLogger.debug("wifiSetup(): Wifi connected.");
  }
}

void blynkSetup()
{
  Blynk.config(BLYNK_TOKEN);  // in place of Blynk.begin(auth, ssid, pass); 
  Blynk.connect(BLYNK_SERVER_TIMEOUT);
  yield;
}

void blynkWatchdog()
{
  blynkLogger.debug("blynkWatchdog(): awake.");

  if (WiFi.status() != 3) {
    blynkLogger.error("blynkWatchdog(): No WiFi.");
    return;
  }
  bool triedToRestoreConnection = false;

  if (!Blynk.connected()) {
    blynkLogger.error("blynkWatchdog(): Blynk not connected.");

    if (blynkErrors > MAX_BLYNK_ERRORS_UNTIL_WIFI_RESTART) {
      blynkLogger.error("wifiWatchdog(): Could not restore Blynk after many tries. Restarting WiFi...");
      wifiRestart();
    }
  
    Blynk.connect(BLYNK_SERVER_TIMEOUT);

    triedToRestoreConnection = true;
  }

  if (Blynk.connected()) {
     blynkErrors = blynkRestoreFailedAttempts = 0;
     if (triedToRestoreConnection) {
       blynkLogger.info("blynkWatchdog(): Blynk is back on.");
     } else {
       blynkLogger.debug("blynkWatchdog(): ok.");
     }
  } else {
    blynkLogger.notice("blynkWatchdog(): Blynk is still offline.");
    blynkErrors++;

    if (triedToRestoreConnection) {
      blynkRestoreFailedAttempts++;
    }

    IPAddress checkedIp;
    if (blynkRestoreFailedAttempts > MAX_BLYNK_ERRORS_UNTIL_DEVICE_RESTART && !WiFi.hostByName(WIFI_TEST_DOMAIN, checkedIp)) {
        blynkLogger.error("wifiWatchdog(): Could not restore Blynk because no internet access. Restarting device");
        ESP.restart();
        return;
    }

  }
}

void wifiRestart()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(1000);
  wifiSetup();
}

void wifiWatchdog()
{
  blynkLogger.debug("wifiWatchdog(): awake.");

  bool triedToRestoreConnection = false;
  if (WiFi.status() != WL_CONNECTED)  
  {
    WiFi.reconnect();
    wifiWaitForConnection();
    if (wifiErrors > MAX_WIFI_ERRORS_UNTIL_RESTART) {
      blynkLogger.error("wifiWatchdog(): Could not restore WiFi after many tries. Restarting WiFi...");
      wifiRestart();
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    wifiErrors++;
    blynkLogger.notice("wifiWatchdog(): WiFI is still disconnected.");
  }
}

void readTempAndHumidity()
{
  // Reading temperature and humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
  TempAndHumidity tempAndHumidity = dht.getTempAndHumidity();
  yield;

  temperature = tempAndHumidity.temperature * TEMP_CALIBRATION_FACTOR;
  humidity = tempAndHumidity.humidity * HUMIDITY_CALIBRATION_FACTOR;
  heatIndex = dht.computeHeatIndex(temperature, humidity);
  
  sprintf(
    logBuffer, 
    "T: %.1f °C H: %.1f%% I: %.1f °C",
    temperature,
    humidity,
    heatIndex
  );

  blynkLogger.debug(logBuffer);

  if (Blynk.connected()) {
    Blynk.virtualWrite(0, round(temperature * 10) / 10.0); // round to one decimal
    Blynk.virtualWrite(1, round(humidity * 10) / 10.0); // round to one decimal
    Blynk.virtualWrite(2, round(heatIndex * 10) / 10.0); // round to one decimal
  }
}

void updateCO2RunningAverage(int value)
{
  co2RunningAverageBuffer[nextCO2RunningAverageCounter] = value;
  
  nextCO2RunningAverageCounter++;
  if (nextCO2RunningAverageCounter >= co2RunningAverageCount) {
    nextCO2RunningAverageCounter = 0; 
  }
}


uint16_t getCO2RunningAverage()
{
  float runningAverage = 0;
  int sampleCount = 0;
  
  for (uint16_t i = 0; i < co2RunningAverageCount; ++i) {
    if (co2RunningAverageBuffer[i] != -1) {
      runningAverage += co2RunningAverageBuffer[i];
      sampleCount++;
    }
  }

  if (sampleCount == 0) {
    return 0;
  }
  
  runningAverage /= sampleCount;
  
  return round(runningAverage);
}

void readVoc()
{
  uint8_t error;
  if ((error = vocSensor.getErrorRegister()) != CCS811Core::SENSOR_SUCCESS) {
    blynkLogger.error("readVoc(): I2C error. Recovering...");
    if (clearI2CBus() == 0) {
      Wire.begin();
    }

    // only as a last resort
    //ESP.restart();
  }
  // return early if VOC sensor is not prepared
  // 20 min * 60s * 1000ms = 1200000ms
  if (millis() < 1200000) {
    if (Blynk.connected()) {
      Blynk.virtualWrite(BLYNK_REALTIME_VOC_PIN, 0);
      Blynk.virtualWrite(BLYNK_REALTIME_ECO2_PIN, 0);
    }
    return;
  } else if (!isVOCSensorReady) {
    blynkLogger.info("readVoc(): VOC Sensor ready");
    isVOCSensorReady = true;
  }

  vocSensor.setEnvironmentalData(humidity, temperature);
  
  if (vocSensor.dataAvailable()) {
    vocSensor.readAlgorithmResults();
    updateCO2RunningAverage(vocSensor.getCO2());

    sprintf(logBuffer, "tVOC(): %d; eCO2: %d", vocSensor.getTVOC(), vocSensor.getCO2());
    blynkLogger.debug(logBuffer);

    if (Blynk.connected()) {
      Blynk.virtualWrite(BLYNK_REALTIME_VOC_PIN, vocSensor.getTVOC());
      Blynk.virtualWrite(BLYNK_REALTIME_ECO2_PIN, vocSensor.getCO2());
    }
  }
}

void minuteCron()
{
  vocAverages();
  dustAverages();
  sendStatusToBlynk();
}

void vocAverages()
{ 
  strcpy(vocStatusBuffer, "Init VOC.");
  co2Average = getCO2RunningAverage();
  
  sprintf(logBuffer, "1m Avg eCO2 level: %d ppm", co2Average);
  blynkLogger.debug(logBuffer);

  if (co2Average > 5000) {
    strcpy(vocStatusBuffer, "VOC L3/3.");
  } else if (co2Average > 2000) {
    strcpy(vocStatusBuffer, "VOC L2/3.");
  } else if (co2Average > 1000) {
    strcpy(vocStatusBuffer, "VOC L1/3.");
  } else if (isVOCSensorReady) {
    strcpy(vocStatusBuffer, "");
  }

  if (Blynk.connected()) {
    Blynk.virtualWrite(BLYNK_ECO2_AVG_PIN, co2Average);
  }
}

void dustAverages()
{ 
  dustAverage = dustSensor.getRunningAverage();
  float newBaseline = dustSensor.getBaselineCandidate();
  dustSensor.setBaseline(newBaseline);
  
  sprintf(logBuffer, "1m Avg Dust Density: %d ug/m3; New baseline: %.4f", dustAverage, newBaseline);
  blynkLogger.debug(logBuffer);

  if (dustAverage > 200) {
    strcpy(dustStatusBuffer, "Dust L5/5.");
  } else if (dustAverage > 100) {
    strcpy(dustStatusBuffer, "Dust L4/5.");
  } else if (dustAverage > 50) {;
    strcpy(dustStatusBuffer, "Dust L3/5.");
  } else if (dustAverage > 25) {
    strcpy(dustStatusBuffer, "Dust L2/5.");
  } else if (dustAverage > 10) {
    strcpy(dustStatusBuffer, "Dust L1/5.");
  } else {
    strcpy(dustStatusBuffer, "");
  }

  if (Blynk.connected()) {
    Blynk.virtualWrite(BLYNK_DUST_1M_AVG_PIN, dustAverage);
  }
}

void sendStatusToBlynk()
{
  char statusBuffer[250] = "";
  
  if (strlen(vocStatusBuffer)) {
    strcat(statusBuffer, vocStatusBuffer);
  }

  if (strlen(dustStatusBuffer)) {
    strcat(statusBuffer, dustStatusBuffer);
  }

  if (!strlen(statusBuffer)) {
    strcpy(statusBuffer, "Clean air.");
  }

  if (Blynk.connected()) {
    Blynk.virtualWrite(BLYNK_STATUS_TEXT_PIN, statusBuffer);
  }
}

/**
 * Send data to Blynk (sync pins)
 */
/*BLYNK_READ(BLYNK_FAN_CONTROL_VPIN) //Function to set status of BLYNK widget on Android
{
  Blynk.virtualWrite(BLYNK_FAN_CONTROL_VPIN, autoFanControl);
}*/

/**
 * Receive data from Blynk
 */
/*BLYNK_WRITE(V5) {
  Log.verbose("BLYNK_WRITE(): Received blynk command %d."CR, param.asInt());

  switch (param.asInt())
  {
    case 1: {

        break;
      }
  }
}*/

/*void printTimestamp(Print* _logOutput)
{
  char c[12];
  int m = sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}*/

/**
 * Adapted from:
 * http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/I2C_ClearBus.ino.txt
 */
int clearI2CBus()
{
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif
  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  //delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    blynkLogger.error("I2C_ClearBus(): Could not clear. SCL clock line held low");
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
    // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      blynkLogger.error("I2C_ClearBus(): Could not clear. SCL clock line held low by slave clock stretch for >2sec");
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    blynkLogger.error("I2C_ClearBus(): Could not clear. SDA data line held low");
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);

  blynkLogger.info("I2C_ClearBus(): Success");
  
  return 0; // all ok
}


void sendGratuitousARP() {
    netif *n = netif_list;

    while (n) {
        etharp_gratuitous(n);
        n = n->next;
    }
}
