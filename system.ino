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

// ---------------------------- DISPLAY MODE ---------------------------- //
# if DISPLAY_MODE == 1
  GFXcanvas1 co2Canvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 tempCanvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 humCanvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 soundCanvas((WIDTH/5), (HEIGHT/5));
  GFXcanvas1 batteryCanvas((WIDTH/5), (HEIGHT/5));
# elif DISPLAY_MODE == 2
  GFXcanvas1 co2Canvas((WIDTH/2 - 5), (HEIGHT/3));
  GFXcanvas1 tempCanvas((WIDTH/4 + 12), (HEIGHT/5));
  GFXcanvas1 humCanvas((WIDTH/4 - 23), (HEIGHT/5));
  GFXcanvas1 batteryLogoCanvas(25, 12);
  GFXcanvas1 connexionLogoCanvas(20, 12);
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
    // while (1);
  } else {
    Serial.println("[INFO] Lux sensor found");    
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
  const float R1 = 680000; // resistance value of R1 in ohms
  const float R2 = 390000; // resistance value of R2 in ohms
  const float Vin = 3.35; // input voltage to voltage divider
  float batteryVoltage = 0; // Variable pour stocker la tension de la batterie

  // Tableau de correspondance de la tension de la batterie au pourcentage de charge
  const float batteryVoltageTable[] = {3.0, 3.3, 3.6, 3.7, 3.75, 3.79, 3.83, 3.87, 3.92, 3.97, 4.1, 4.2};
  const int batteryPercentageTable[] = {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
  const int batteryTableSize = sizeof(batteryVoltageTable) / sizeof(batteryVoltageTable[0]);

  int sensorValue = analogRead(batteryPin); // Lire la valeur analogique de la broche A0
  float voltage = sensorValue * (3.35 / 1023.0); // Convertir la valeur analogique en tension

  // Calculer la tension de la batterie en utilisant le pont diviseur de tension
  batteryVoltage = voltage / (R2 / (R1 + R2));

  int batteryPercentage = 0;
  for (int i = 0; i < batteryTableSize - 1; i++) {
    if (batteryVoltage >= batteryVoltageTable[i] && batteryVoltage < batteryVoltageTable[i + 1]) {
      float slope = (float)(batteryPercentageTable[i + 1] - batteryPercentageTable[i]) / (batteryVoltageTable[i + 1] - batteryVoltageTable[i]);
      float offset = batteryPercentageTable[i] - slope * batteryVoltageTable[i];
      batteryPercentage = (int)(slope * batteryVoltage + offset);
      break;
    }
  }
  return batteryPercentage;
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
  data2.luxValue = veml.readLux();  
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
void headerOverlay() { //TODO get current time
  display.setCursor(((WIDTH/2) - (WIDTH/4) + 35), 5);
  display.setTextSize(2);
  display.setTextColor(EPD_BLACK);
  display.print("22:58");

  display.drawRoundRect(2, 2, (WIDTH - 4), (HEIGHT/5 - 4), 4, EPD_BLACK);
  display.drawRoundRect(2, (HEIGHT/5 + 2), (WIDTH - 4), (4 * HEIGHT/5 - 2), 4, EPD_BLACK);
}
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
    display.setCursor(10, (HEIGHT/2 - 8));
    display.setTextSize(3);
    display.setTextColor(EPD_BLACK);
    display.print("CO2");

    display.setCursor((2*WIDTH/3 + 20), (HEIGHT/2 - 8));
    display.setTextSize(3);
    display.setTextColor(EPD_BLACK);
    display.print("ppm");
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
    setTemperatureLogo(17, 107, 5, 15);
    display.setCursor((WIDTH/2 - 19), 87);
    display.setTextSize(2);
    display.setTextColor(EPD_BLACK);
    display.print("o");
    display.setCursor((WIDTH/2 - 6), 92);
    display.setTextSize(3);
    display.setTextColor(EPD_BLACK);
    display.print("C");
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
    setHumidityLogo((WIDTH/2 + 45), 107, 5);

    display.setCursor((WIDTH/2 + 100), 92);
    display.setTextSize(3);
    display.setTextColor(EPD_BLACK);
    display.print("%");
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
    Serial.println("[WARNING] No decibel overlay for this display parameter.");
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
    Serial.println("[WARNING] No battery overlay in this configuration.");
  #endif

  Serial.println("[INFO] Battery overlay has been pushed.");
}

void setupScreenOverlay() {
  headerOverlay();
  co2ScreenOverlay();
  temperatureScreenOverlay();
  humidityScreenOverlay();
  decibelScreenOverlay();
  batteryScreenOverlay();
  display.display();
}

void displayConnexionLogo(int16_t x, int16_t y) {
  display.drawFastVLine(x, (y + 8), 2, EPD_BLACK);
  display.drawFastVLine((x + 4), (y + 6), 4, EPD_BLACK);
  display.drawFastVLine((x + 8), (y + 4), 6, EPD_BLACK);
  display.drawFastVLine((x + 12), (y + 2), 8, EPD_BLACK);
  if (!elementSent || ONLINE_STATUS == 0) {
    display.drawLine((x - 2), (y + 5), (x + 2), (y + 1), EPD_BLACK);
    display.drawLine((x - 2), (y + 1), (x + 2), (y + 5), EPD_BLACK);
  }
}

void displayBatteryLogo(int16_t x, int16_t y, int batteryLevel) {
  display.drawRoundRect(x, y, 20, 10, 1, EPD_BLACK);
  display.drawRoundRect((x + 19), (y + 2), 3, 6, 1, EPD_BLACK);
  int w = (batteryLevel * 20)/100;
  display.fillRoundRect(x, y, w, 10, 1, EPD_BLACK);
}

void setHumidityLogo(int x, int y, int r) {
  display.fillCircle(x, y, r, EPD_BLACK); 
  display.fillTriangle((x - r + 1), (y - 4), (x + r - 1), (y - 4), x, (y - (r * 2) - 4), EPD_BLACK);
}

void setTemperatureLogo(int x, int y, int r, int h) {
  display.fillCircle(x, (y), r, EPD_BLACK);
  display.fillRoundRect((x - (r/2)), (y - 16), r, h, 2, EPD_BLACK);
  display.drawFastVLine(x, (y - (h - 3)), (h - 3), EPD_WHITE);
  display.drawFastHLine((x + (r/2) + 2), (y - 15), 4, EPD_BLACK);
  display.drawFastHLine((x + (r/2) + 2), (y - 12), 2, EPD_BLACK);
  display.drawFastHLine((x + (r/2) + 2), (y - 9), 2, EPD_BLACK);
  display.drawFastHLine((x + (r/2) + 2), (y - 6), 4, EPD_BLACK);
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
    co2Canvas.setTextSize(5); // size 3
    co2Canvas.fillScreen(0);
    co2Canvas.setCursor(0, 0);
    co2Canvas.print(co2Buffer);
    display.drawBitmap((WIDTH/2 - 60), (HEIGHT/2 - 21), co2Canvas.getBuffer(), co2Canvas.width(), co2Canvas.height(), EPD_BLACK, EPD_WHITE);
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
    tempCanvas.setTextSize(3);
    tempCanvas.fillScreen(0);
    tempCanvas.setCursor(0, 0);
    tempCanvas.print(tempBuffer);
    display.drawBitmap(31, 92, tempCanvas.getBuffer(), tempCanvas.width(), tempCanvas.height(), EPD_BLACK, EPD_WHITE);
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
    humCanvas.setTextSize(3);
    humCanvas.fillScreen(0);
    humCanvas.setCursor(0, 0);
    humCanvas.print(humBuffer);
    display.drawBitmap((WIDTH/2 + 60), 92, humCanvas.getBuffer(), humCanvas.width(), humCanvas.height(), EPD_BLACK, EPD_WHITE);
  #endif

  #if DISPLAY_MODE == 1
    char noiseBuffer[10];
    dtostrf(noiseSensor, 4, 1, noiseBuffer);
    soundCanvas.setTextSize(2);
    soundCanvas.fillScreen(0);
    soundCanvas.setCursor(0, 0);
    soundCanvas.print(noiseBuffer);
    display.drawBitmap((2*WIDTH/3 + 10), (HEIGHT/3 - 12), soundCanvas.getBuffer(), soundCanvas.width(), soundCanvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    // TODO
    Serial.println("[WARNING] No decibel overlay for this display parameter.");
  #endif
  
  int batteryState = readBatteryLevel();

  #if DISPLAY_MODE == 1
    char batteryBuffer[10];
    itoa(batteryLevel, batteryBuffer, 10);
    batteryCanvas.setTextSize(2);
    batteryCanvas.fillScreen(0);
    batteryCanvas.setCursor(0, 0);
    batteryCanvas.print(batteryBuffer);
    display.drawBitmap((2*WIDTH/3 + 10), (2*HEIGHT/3 + 8), batteryCanvas.getBuffer(), batteryCanvas.width(), batteryCanvas.height(), EPD_BLACK, EPD_WHITE);
  #elif DISPLAY_MODE == 2
    batteryLogoCanvas.fillScreen(0);
    display.drawBitmap((WIDTH - (WIDTH/8)), 7, batteryLogoCanvas.getBuffer(), batteryLogoCanvas.width(), batteryLogoCanvas.height(), EPD_BLACK, EPD_WHITE);
    displayBatteryLogo((WIDTH - (WIDTH/8)), 7, batteryState);
  #endif

  #if DISPLAY_MODE == 2
    connexionLogoCanvas.fillScreen(0);
    display.drawBitmap(11, 7, connexionLogoCanvas.getBuffer(), connexionLogoCanvas.width(), connexionLogoCanvas.height(), EPD_BLACK, EPD_WHITE);    
    displayConnexionLogo(14, 7);
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
    elementSent = true;
  } else {
    Serial.println("[ERROR] Couldn't send the message.");
    // TODO: add an icon to the screen to indicate that the message couldn't be sent
    elementSent = false;
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

    display.display();
    Serial.println("[INFO] Displaying datas.");
  }
}
// ------------------------ END OF INTERRUPTION ----------------------- //


void setup() {
  // ------------------- PIN MODE SETUP ------------------- //
  pinMode(microphonePin, INPUT);
  Serial.println("[INFO] Setting up microphone.");

  pinMode(interruptPin, INPUT_PULLDOWN);
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