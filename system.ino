#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include "Adafruit_VEML7700.h"
#include <Wire.h>
#include "Adafruit_ThinkInk.h"
#include <avr/dtostrf.h>
#include <stdlib.h>
#include <MKRWAN.h>
#include "ArduinoLowPower.h"
#include "system.h"

LoRaModem modem;
String appEui = "0000000000000000";
String appKey = "E6A69E487ABFDBF5CB20A3A96A9BE266";
Adafruit_VEML7700 veml = Adafruit_VEML7700();
SensirionI2CScd4x scd4x;

scd4xSensor data1;
otherSensor data2;
systemConfigurationSettings device;

ThinkInk_213_Mono_BN display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

// int WIDTH = display.width();
// int HEIGHT = display.height();

// ---------------------------- DISPLAY MODE ---------------------------- //
# if DISPLAY_MODE == 1
  GFXcanvas1 co2Canvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 tempCanvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 humCanvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 soundCanvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 batteryCanvas((WIDTH/5), (HEIGHT/5));
# elif DISPLAY_MODE == 2
  GFXcanvas1 co2Canvas((WIDTH/2), (HEIGHT/2));
  GFXcanvas1 tempCanvas((WIDTH/2), (HEIGHT/2));
  GFXcanvas1 humCanvas((WIDTH/2), (HEIGHT/2));
  GFXcanvas1 soundCanvas((WIDTH/2), (HEIGHT/2));
  GFXcanvas1 batteryCanvas((WIDTH/2), (HEIGHT/2));
# endif
// ------------------------ END OF DISPLAY MODE ------------------------- //


// ------------------------------- SETUPS ------------------------------- //
void initializeLoRaWANGatewayConnexion() {
  Serial.println("[INFO] Starting LoRaWAN connexion...");
  if (!modem.begin(EU868)) {
    Serial.println("[ERROR] Failed to start LoRaWAN module.");
    while (1) {}
  };  

  Serial.print("[INFO] LoRaWAN module version is: ");
  Serial.println(modem.version());
  Serial.print("[INFO] Device EUI is: ");
  Serial.println(modem.deviceEUI());
  Serial.println("[INFO] Trying to join TTN ...");
  int connected = modem.joinOTAA(appEui, appKey);

  if (!connected) {
    Serial.println("[ERROR] Something went wrong; are you indoor? Move near a window and retry");
    while (1) {}
  } modem.minPollInterval(60);
  // NOTE: independently by this setting the modem will
  // not allow to send more than one message every 2 minutes,
  // this is enforced by firmware and can not be changed.
}

void initializeGroveSensor() {
  Wire.begin();
  scd4x.begin(Wire); // enable connection
  scd4x.reinit();

  scd4x.setAmbientPressure(1013); // 1013 hPa
  scd4x.setSensorAltitude(35); // 35 m
  // scd4x.setAutomaticSelfCalibration(1); // enable self calibration

  uint16_t error;
  char errorMessage[256];
  error = scd4x.stopPeriodicMeasurement(); // stop potentially previously started measurement
  if (error) {
    Serial.print("[ERROR] Could not execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  error = scd4x.startLowPowerPeriodicMeasurement(); // get first measurement
  if (error) {
    Serial.print("[ERROR] Could not execute startLowPowerPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }  
}

void initializeLuxSensor() {
  if (!veml.begin()) {
    Serial.println("[ERROR] Lux sensor not found");
    while (1);
  }
  Serial.println("[INFO] Lux sensor found");

  Serial.print(F("[INFO] Lux sensor gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("[INFO] Lux sensor integration time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }
}

void initializeSensors() {
  initializeLuxSensor();
  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
  Serial.println("[ALERT] Lux sensor ready.");

  initializeGroveSensor();  
  Serial.println("[ALERT] Grove sensor ready.");
}

void initializeScreen() {
  Serial.println("[INFO] Adafruit eInk display setting up...");
  display.begin(THINKINK_MONO);
  display.clearBuffer();
  Serial.println("[ALERT] Buffer has been cleared.");

  #if DISPLAY_MODE == 1
    Serial.println("[INFO] Display mode 1 chosen.");
  #elif DISPLAY_MODE == 2
    Serial.println("[INFO] Display mode 2 chosen.");
  #endif

  setupScreenOverlay();
  // canvas.setTextWrap(true);
  // display.display();

  Serial.println("[INFO] Display ready for use.");  
}
// --------------------------- END OF SETUP ----------------------------- //


// ------------------------------- SENSORS ------------------------------ //
double readNoiseLevel() {
  double pa_reference = 0.00002; // Référence pour le calcul de l'amplitude (Pa/LSB)
  double sensitivity = 66; // Gain de l'amplificateur du microphone
  double VCC = 5.0; // Tension d'alimentation du microphone (V)
  unsigned long startMillis = millis();  // Heure de début de l'échantillonnage
  unsigned int peakToPeak = 0;   // Mesure d'amplitude
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  int sampleWindow = 200; // La taille de la fenêtre d'échantillonnage (en millisecondes)
  unsigned int sample; // Valeur de l'échantillon
  
  // Acquisition de l'échantillon de son
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(microphonePin);
    if (sample < 1024) {
      if (sample > signalMax) {
        signalMax = sample;
      } else if (sample < signalMin) {
        signalMin = sample;
      }
    }
  }
  peakToPeak = (signalMax - signalMin);
  
  // Conversion RMS en dB
  double volts = (peakToPeak * VCC) / 1024;
  double vp = volts / 2; // Tension crête-à-crête
  double v_rms = vp / sqrt(2); // Tension efficace (RMS)
  double pa_rms = v_rms / sensitivity; // Pression acoustique efficace (RMS)
  return (20 * log10(pa_rms / pa_reference) + 20);
}

int readBatteryLevel() {
  int batteryLevel = 0;

  return batteryLevel;
}

void readSensorsData() {
  // -------------------- SCD4X SENSOR -------------------- //
  if (isSleep) {
    isSleep = false;
    scd4x.wakeUp();
  }
  uint16_t error;
  char errorMessage[256];
  error = scd4x.readMeasurement(data1.co2Value, data1.temperatureValue, data1.humidityValue);
  if (error) {
    Serial.print("[ERROR] Couldn't execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (data1.co2Value == 0) {
    Serial.println("[ERROR] Invalid co2 sample detected.");
  }
  scd4x.powerDown();
  isSleep = true;  
  // ------------------------------------------------------ //

  // -------------------- SOUND SENSOR -------------------- //
  data2.noiseValue = readNoiseLevel();
  if (data2.noiseValue == 0) {
    Serial.print("[ERROR] Couldn't execute readNoiseLevel(): ");
    Serial.println("invalid value.");
  }
  // ------------------------------------------------------ //

  // --------------------- LUX SENSOR --------------------- //
  // Serial.print("raw ALS: "); Serial.println(veml.readALS());
  // Serial.print("raw white: "); Serial.println(veml.readWhite());
  data2.luxValue = veml.readLux();  
  // uint16_t irq = veml.interruptStatus();
  // if (irq & VEML7700_INTERRUPT_LOW) {
  //   Serial.println("[WARNING] Lux sensor low threshold");
  // }
  // if (irq & VEML7700_INTERRUPT_HIGH) {
  //   Serial.println("[WARNING] Lux sensor high threshold");
  // }
  // ------------------------------------------------------ //

  data2.batteryValue = readBatteryLevel();
}
// --------------------------- END OF SENSORS --------------------------- //


// ------------------------------- ALERTS ------------------------------- //
void sensorAlertManager(uint8_t type, int value, int threshold) {
  switch(type) {
    case 0: // CO2
      if (value > threshold) {
        Serial.println("[ALERT] CO2 level over the threshold!");
        // print to E-Ink display
      } break;

    case 1: // Temperature
      if (value > threshold) {
        Serial.println("[ALERT] Temperature level over the threshold!");
        // print to E-Ink display
      } break;

    case 2: // Humidity
      if (value > threshold) {
        Serial.println("[ALERT] Humidity level over the threshold!");
        // print to E-Ink display
      } break;

    case 3: // Sound
      if (value > threshold) {
        Serial.println("[ALERT] Sound level over the threshold!");
        // print to E-Ink display
      } break;
    
    case 4: // Luminosity
      if (value > threshold) {
        Serial.println("[ALERT] Luminosity level over the threshold!");
        // print to E-Ink display
      } break;
    
    case 5: // Battery
      if (value < threshold) {
        Serial.println("[ALERT] Battery level under the threshold!");
        // ...
      }

    default:
      break;      
  }
}
// --------------------------- END OF ALERTS ---------------------------- //


// ----------------------------- OVERLAYS ----------------------------- //
void co2ScreenOverlay() {
  Serial.println("[INFO] Building co2 overlay...");

  #if DISPLAY_MODE == 1
    display.fillRect(2, 2, (WIDTH/3 - 4), (HEIGHT - 4), EPD_BLACK);
    display.fillRect(4, 4, (WIDTH/3 - 8), (HEIGHT - 8), EPD_WHITE);

    display.setTextSize(2);
    display.setTextColor(EPD_BLACK);
    display.setCursor(5, 5);
    display.print("CO2");

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((WIDTH/4 - 4), (HEIGHT - 15));
    display.print("ppm");
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  Serial.println("[INFO] CO2 overlay has been pushed.");
}

void temperatureScreenOverlay() {
  Serial.println("[INFO] Building temperature overlay...");

  #if DISPLAY_MODE == 1
    display.fillRect((WIDTH/3 + 2), 2, (WIDTH/3 - 4), (HEIGHT/2 - 4), EPD_BLACK);
    display.fillRect((WIDTH/3 + 4), 4, (WIDTH/3 - 8), (HEIGHT/2 - 8), EPD_WHITE);

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((WIDTH/3 + 5), 5);
    display.print("Temperature");

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((2*WIDTH/3 - 14), (HEIGHT/2 - 15));
    display.print("C");
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  Serial.println("[INFO] Temperature overlay has been pushed.");
}

void humidityScreenOverlay() {
  Serial.println("[INFO] Building humidity overlay...");

  #if DISPLAY_MODE == 1
    display.fillRect((WIDTH/3 + 2), (HEIGHT/2 + 2), (WIDTH/3 - 4), (HEIGHT/2 - 4), EPD_BLACK);
    display.fillRect((WIDTH/3 + 4), (HEIGHT/2 + 4), (WIDTH/3 - 8), (HEIGHT/2 - 8), EPD_WHITE);

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((WIDTH/3 + 5), (HEIGHT/2 + 5));
    display.print("Humidity");

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((2*WIDTH/3 - 14), (HEIGHT - 15));
    display.print("%");
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  Serial.println("[INFO] Humidity overlay has been pushed.");
}

void decibelScreenOverlay() {
  Serial.println("[INFO] Building decibel overlay...");

  #if DISPLAY_MODE == 1
    display.fillRect((2*WIDTH/3 + 2), 2, (WIDTH/3 - 4), (HEIGHT/2 - 4), EPD_BLACK);
    display.fillRect((2*WIDTH/3 + 4), 4, (WIDTH/3 - 8), (HEIGHT/2 - 8), EPD_WHITE);

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((2*WIDTH/3 + 5), 5);  
    display.print("Decibel");

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((WIDTH - 20), (HEIGHT/2 - 15));
    display.print("dB");
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  Serial.println("[INFO] Decibel overlay has been pushed.");
}

void batteryScreenOverlay() {
  Serial.println("[INFO] Building battery overlay...");
  
  #if DISPLAY_MODE == 1
    display.fillRect((2*WIDTH/3 + 2), (HEIGHT/2 + 2), (WIDTH/3 - 4), (HEIGHT/2 - 4), EPD_BLACK);
    display.fillRect((2*WIDTH/3 + 4), (HEIGHT/2 + 4), (WIDTH/3 - 8), (HEIGHT/2 - 8), EPD_WHITE);

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((2*WIDTH/3 + 5), (HEIGHT/2 + 5));
    display.print("Battery");

    display.setTextSize(1);
    display.setTextColor(EPD_BLACK);
    display.setCursor((WIDTH - 15), (HEIGHT - 15));
    display.print("%");
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  Serial.println("[INFO] Battery overlay has been pushed.");
}

void setupScreenOverlay() {
  co2ScreenOverlay();
  temperatureScreenOverlay();
  humidityScreenOverlay();
  decibelScreenOverlay();
  batteryScreenOverlay();
  display.display();
}
// -------------------------- END OF OVERLAYS ------------------------- //


// ----------------------------- DISPLAYS ----------------------------- //
void displayDatasToScreen(uint16_t co2Sensor, float tempSensor, float humSensor, double noiseSensor, int batteryLevel) {
  char co2Buffer[10];
  utoa(co2Sensor, co2Buffer, 10);
  
  #if DISPLAY_MODE == 1
    co2Canvas.setTextSize(2); // size 3
    co2Canvas.fillScreen(0);
    co2Canvas.setCursor(0, 0);
    co2Canvas.print(co2Buffer);
    display.drawBitmap((WIDTH/12 - 10), (HEIGHT/2 - 8), co2Canvas.getBuffer(), co2Canvas.width(), co2Canvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  char tempBuffer[10];
  dtostrf(tempSensor, 4, 1, tempBuffer);
  
  #if DISPLAY_MODE == 1
    tempCanvas.setTextSize(2);
    tempCanvas.fillScreen(0);
    tempCanvas.setCursor(0, 0);
    tempCanvas.print(tempBuffer);
    display.drawBitmap((WIDTH/3 + 10), (HEIGHT/3 - 12), tempCanvas.getBuffer(), tempCanvas.width(), tempCanvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  char humBuffer[10];
  dtostrf(humSensor, 4, 1, humBuffer);
  
  #if DISPLAY_MODE == 1
    humCanvas.setTextSize(2);
    humCanvas.fillScreen(0);
    humCanvas.setCursor(0, 0);
    humCanvas.print(humBuffer);
    display.drawBitmap((WIDTH/3 + 10), (2*HEIGHT/3 + 8), humCanvas.getBuffer(), humCanvas.width(), humCanvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  char noiseBuffer[10];
  dtostrf(noiseSensor, 4, 1, noiseBuffer);

  #if DISPLAY_MODE == 1
    soundCanvas.setTextSize(2);
    soundCanvas.fillScreen(0);
    soundCanvas.setCursor(0, 0);
    soundCanvas.print(noiseBuffer);
    display.drawBitmap((2*WIDTH/3 + 10), (HEIGHT/3 - 12), soundCanvas.getBuffer(), soundCanvas.width(), soundCanvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  char batteryBuffer[10];
  itoa(batteryLevel, batteryBuffer, 10);
  
  #if DISPLAY_MODE == 1
    batteryCanvas.setTextSize(2);
    batteryCanvas.fillScreen(0);
    batteryCanvas.setCursor(0, 0);
    batteryCanvas.print(batteryBuffer);
    display.drawBitmap((2*WIDTH/3 + 10), (2*HEIGHT/3 + 8), batteryCanvas.getBuffer(), batteryCanvas.width(), batteryCanvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    // TODO
  #endif

  Serial.println("[ALERT] Sensor's datas are being pushed.");
}

void displayDatasToTerminal(unsigned short co2Sensor, float tempSensor, float humSensor, double noiseSensor, int luxSens, int batteryLevel) {
  Serial.print("[INFO] Co2: ");
  Serial.print(co2Sensor);
  Serial.println(" ppm");
  
  Serial.print("[INFO] Temperature: ");
  Serial.print(tempSensor);
  Serial.println(" °C");
  
  Serial.print("[INFO] Humidity: ");
  Serial.print(humSensor);
  Serial.println(" %");
  
  Serial.print("[INFO] Sound: ");
  Serial.print(noiseSensor);
  Serial.println(" dB");

  Serial.print("[INFO] Lux: ");
  Serial.print(luxSens);  
  Serial.println(" lux");

  Serial.print("[INFO] Battery: ");
  Serial.print(batteryLevel);
  Serial.println(" %");
}
// -------------------------- END OF DISPLAYS ------------------------- //


// ----------------------------- TX & RX ------------------------------ //
void dataTransmission(unsigned short co2Sensor, float tempSensor, float humSensor, double noiseSensor, int luxSens, int batteryLevel) {
  uint16_t co2 = co2Sensor;
  short co2_2 = co2 >> 8;
  short co2_1 = (co2 << 8) >> 8;
  short temp = (short)tempSensor;
  short hum = (short)humSensor;
  short sound = (short)noiseSensor;
  short lux = (short)luxSens;
  short batt = (short)batteryLevel;
  short err;

  char bufferLoRaWAN[7] = {co2_1, co2_2, temp, hum, sound, lux, batt};

  modem.beginPacket();
  modem.write(bufferLoRaWAN, 7);
  err = modem.endPacket(true);

  if (err > 0) {
    Serial.println("[INFO] Message sent correctly!");
    // TODO: add an icon to the screen to indicate that the message has been sent
  } else {
    Serial.println("[ERROR] Couldn't send the message.");
    // TODO: add an icon to the screen to indicate that the message couldn't be sent
  }  
}

void dataReception() {
  delay(1000);
  if (!modem.available()) {
    Serial.println("[INFO] No downlink message received at this time.");
  }

  char rcv[258];
  int i = 0;
  while (modem.available()) {
    rcv[i++] = (char)modem.read();
  }
  
  Serial.println("[ALERT] Data received: ");
  
  for (unsigned int j = 0; j < i; j++) {
    Serial.print(rcv[j] >> 4, HEX);
    Serial.print(rcv[j] & 0xF, HEX);
    Serial.println();
  }
}
// -------------------------- END OF TX & RX -------------------------- //


// --------------------------- INTERRUPTION --------------------------- //
void buttonISR() {
  Serial.println("[INFO] Interruption triggered.");

  if (isSleep) {
    isSleep = false;
    scd4x.wakeUp();
  }
  
  uint16_t error;
  char errorMessage[256];
  
  float co2, temperature, humidity;
  uint8_t data[12], counter;

  bool isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
    Serial.print("Error trying to execute getDataReadyFlag(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    return;
  }
  
  if (!isDataReady) {
    Serial.println("[WARNING] Elapsed time between previous measure is too short.");
    return;
  } else {
    // send read data command
    Wire.beginTransmission(SCD_ADDRESS);
    Wire.write(0xEC);
    Wire.write(0x05);
    Wire.endTransmission();
    
    // read measurement data: 2 bytes co2, 1 byte CRC,
    // 2 bytes T, 1 byte CRC, 2 bytes RH, 1 byte CRC,
    // 2 bytes sensor status, 1 byte CRC
    // stop reading after 12 bytes (not used)
    // other data like  ASC not included
    Wire.requestFrom(SCD_ADDRESS, 12);
    counter = 0;
    while (Wire.available()) {
      data[counter++] = Wire.read();
    }
    
    // floating point conversion according to datasheet
    co2 = (float)((uint16_t)data[0] << 8 | data[1]);
    // convert T in degC
    temperature = -45 + 175 * (float)((uint16_t)data[3] << 8 | data[4]) / 65536;
    // convert RH in %
    humidity = 100 * (float)((uint16_t)data[6] << 8 | data[7]) / 65536;

    scd4x.powerDown();
    isSleep = true;

    Serial.println("[ALERT] Pushing new values.");
    data1.co2Value = (uint16_t)co2;
    data1.temperatureValue = temperature;
    data1.humidityValue = humidity; 
    data2.luxValue = veml.readLux();
    data2.batteryValue = readBatteryLevel();
    
    displayDatasToTerminal(data1.co2Value, data1.temperatureValue, data1.humidityValue, data2.noiseValue, data2.luxValue, data2.batteryValue);
    displayDatasToScreen(data1.co2Value, data1.temperatureValue, data1.humidityValue, data2.noiseValue, data2.batteryValue);

    // sensorAlertManager(0, data1.co2Value, device.co2Threshold);
    // sensorAlertManager(1, data1.temperatureValue, device.temperatureThreshold);
    // sensorAlertManager(2, data1.humidityValue, device.humidityThreshold);
    // sensorAlertManager(3, data2.noiseValue, device.noiseThreshold); 
    // sensorAlertManager(4, data2.luxValue, device.luxThreshold);
    // sensorAlertManager(5, data2.batteryValue, device.batteryThreshold);

    display.display();
    Serial.println("[INFO] Displaying datas.");
  }
}
// ------------------------ END OF INTERRUPTION ----------------------- //


void setup() {
  // ------------------- PIN MODE SETUP ------------------- //
  pinMode(microphonePin, INPUT);
  Serial.println("[INFO] Setting up microphone.");

  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonISR, FALLING);
  Serial.println("[INFO] Setting up external push button.");

  pinMode(batteryPin, INPUT);
  Serial.println("[INFO] Setting up battery.");
  // ------------------------------------------------------ //

  // ------------------ SERIAL COM SETUP ------------------ //
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  } Serial.println("[INFO] Serial communication available.");
  // ------------------------------------------------------ //

  #if ONLINE_STATUS == 1
    initializeLoRaWANGatewayConnexion();
  #endif
  initializeSensors();
  initializeScreen();

  Serial.println("[INFO] Waiting for first measurement...(10sec)");
  delay(10000);
}

void loop() { 
  readSensorsData();
  displayDatasToTerminal(data1.co2Value, data1.temperatureValue, data1.humidityValue, data2.noiseValue, data2.luxValue, data2.batteryValue);
  displayDatasToScreen(data1.co2Value, data1.temperatureValue, data1.humidityValue, data2.noiseValue, data2.batteryValue);
  
  #if ONLINE_STATUS == 1
    dataTransmission(data1.co2Value, data1.temperatureValue, data1.humidityValue, data2.noiseValue, data2.luxValue, data2.batteryValue);
  #endif

  sensorAlertManager(0, data1.co2Value, device.co2Threshold);
  sensorAlertManager(1, data1.temperatureValue, device.temperatureThreshold);
  sensorAlertManager(2, data1.humidityValue, device.humidityThreshold);
  sensorAlertManager(3, data2.noiseValue, device.noiseThreshold); 
  sensorAlertManager(4, data2.luxValue, device.luxThreshold);
  sensorAlertManager(5, data2.batteryValue, device.batteryThreshold);

  display.display();
  Serial.println("[INFO] Displaying datas.");

  Serial.println("[ALERT] Standby mode until next measurement.\n");
  delay(device.measuringFrequency); // wait

  #if ONLINE_STATUS == 1
    dataReception();
  #endif
}