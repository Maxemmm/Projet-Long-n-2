#ifndef SYSTEM_H
#define SYSTEM_H

// -------------- CUSTOM DEFINES --------------- //
#define ONLINE_STATUS 1 // ONLINE: 1, OFFLINE: 0

// -------------- DISPLAY DEFINES -------------- //
#define EPD_CS      4
#define EPD_DC      3
#define SRAM_CS     -1
#define EPD_RESET   -1 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)

#define DISPLAY_MODE 1 // display setup

// -------------- THRESHOLDS DEFINES -------------- //
#define CO2_THRESHOLD 1700
#define TEMP_THRESHOLD 30
#define HUM_THRESHOLD 60
#define NOISE_THRESHOLD 60
#define LUX_THRESHOLD 1000
#define BATT_THRESHOLD 20

// -------------- GLOBAL VARIABLES -------------- //
/* Display */
int WIDTH = 250;
int HEIGHT = 122;

/* Arduino Pins */
const int batteryPin = A1;
const int microphonePin = A2;
const int interruptPin = 1;

/* LoRaWAN ID */
// #if ONLINE_STATUS == 1
//   String appEui = "0000000000000000";
//   String appKey = "E6A69E487ABFDBF5CB20A3A96A9BE266";
// #endif
// String appEui = "0000000000000000";
// String appKey = "E6A69E487ABFDBF5CB20A3A96A9BE266";

/* Temporary variables */
bool isSleep = false;
const int SCD_ADDRESS = 0x62;

// -------------- STRUCTURES -------------- //
struct scd4xSensor {
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
void co2ScreenOverlay();
void temperatureScreenOverlay();
void humidityScreenOverlay();
void decibelScreenOverlay();
void batteryScreenOverlay();
void setupScreenOverlay();

/* Display values */
void displayDatasToScreen(uint16_t co2Sensor, float tempSensor, float humSensor, double noiseSensor, int batteryLevel);
void displayDatasToTerminal(uint16_t co2Sensor, float tempSensor, float humSensor, double noiseSensor, int luxSensor, int batteryLevel);

/* LoRaWAN Tx & Rx */
#if ONLINE_STATUS == 1
  void dataTransmission();
  void dataReception();
#endif

/* Button callback interruption */
void buttonISR();

/* Change sensor treshold */
// TODO

#endif