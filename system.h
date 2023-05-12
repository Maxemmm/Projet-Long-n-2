#ifndef SYSTEM_H
#define SYSTEM_H

// -------------- CUSTOM DEFINES --------------- //
#define ONLINE_STATUS 0 // ONLINE: 1, OFFLINE: 0

// -------------- DISPLAY DEFINES -------------- //
#define EPD_CS      4
#define EPD_DC      3
#define SRAM_CS     -1
#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)

#define DISPLAY_MODE 2 // display setup

// -------------- THRESHOLDS DEFINES -------------- //
#define CO2_THRESHOLD 1000
#define TEMP_THRESHOLD 30
#define HUM_THRESHOLD 60
#define NOISE_THRESHOLD 60
#define LUX_THRESHOLD 1000
#define BATT_THRESHOLD 20

// -------------- GLOBAL VARIABLES -------------- //
/* Display */
int WIDTH = 250;
int HEIGHT = 122;
uint16_t fontColorCO2 = EPD_BLACK;
uint16_t backgroundColorCO2 = EPD_WHITE;
uint16_t fontColorHeader = EPD_BLACK;
uint16_t backgroundColorHeader = EPD_WHITE;
uint16_t fontColorFooter = EPD_BLACK;
uint16_t backgroundColorFooter = EPD_WHITE;

/* Arduino Pins */
const int batteryPin = A1;
const int microphonePin = A2;
const int interruptPin = 1;

/* Unsorted variables */
static char errorMessage[128];
static int16_t error;
// bool isSleep = false;
const int SCD_ADDRESS = 0x62;
volatile bool elementSent = false;
volatile bool maxThresholdCO2 = false;


// -------------- STRUCTURES -------------- //
struct scd30Sensor {
  uint16_t co2Value;
  float temperatureValue,
        humidityValue;
};

struct otherSensor {
  double noiseValue = 0;
  int luxValue = 0,
      batteryValue;
};

struct systemConfigurationSettings {
  unsigned long measuringFrequency = 60000; // measuring every 1 minutes
  int co2Threshold = CO2_THRESHOLD, // (ppm)
      temperatureThreshold = TEMP_THRESHOLD, // (°C)
      humidityThreshold = HUM_THRESHOLD, // (%)
      noiseThreshold = NOISE_THRESHOLD, // (dB)
      luxThreshold = LUX_THRESHOLD, // (LUX)
      batteryThreshold = BATT_THRESHOLD; // (%)
};

// -------------- FUNCTION DECLARATIONS -------------- //

/* Setups */
void initializeLoRaWANGatewayConnexion();
void initializeGroveSensor();
void initializeLuxSensor();
void initializeSensors();
void initializeScreen();

/* Sensors datas acquirement */
double readNoiseLevel();
int readBatteryLevel();
void readSensorsData();

/* Alert sensor management */
void sensorAlertManager(uint8_t type, int value, int threshold);

/* Display overlay initilization */
void headerOverlay();
void co2ScreenOverlay();
void temperatureScreenOverlay();
void humidityScreenOverlay();
void decibelScreenOverlay();
void batteryScreenOverlay();
void setupScreenOverlay();
void displayConnexionLogo(int16_t x, int16_t y);
void displayBatteryLogo(int16_t x, int16_t y, int batteryLevel);
void setHumidityLogo(int x, int y, int r);
void setTemperatureLogo(int x, int y, int r, int h);

/* Display values */
void displayDatasToScreen(float co2Sensor, float tempSensor, float humSensor, double noiseSensor, int batteryLevel);
void displayDatasToTerminal(float co2Sensor, float tempSensor, float humSensor, double noiseSensor, int luxSensor, int batteryLevel);

/* LoRaWAN Tx & Rx */
#if ONLINE_STATUS == 1
  void dataTransmission(float co2Sensor, float tempSensor, float humSensor, double noiseSensor, int luxSens, int batteryLevel);
  void dataReception();
#endif

/* Button callback interruption */
void buttonISR();

/* Change sensor treshold */
// TODO

#endif