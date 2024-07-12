#include <Adafruit_DPS310.h>
#include <math.h>
#include <SensirionI2CSdp.h>
#include <Wire.h>
#include <SPI.h>
#include <TORICA_SD.h>
#include <Adafruit_NeoPixel.h>

#define SerialIN  Serial1
#define SerialOUT Serial1

SensirionI2CSdp sdp;
volatile uint16_t error;
char errorMessage[256];

int cs_SD = 28;
TORICA_SD sd(cs_SD, false);
char SD_BUF[256]; 

Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

volatile float dps_altitude_m = 0;
volatile float dps_pressure_hPa = 0;
volatile float dps_temperature_deg = 0;
volatile float sdp_airspeed_ms = 0;
volatile float sdp_differentialPressure_Pa = 0;
volatile float sdp_airspeed_mss = 0;
volatile float sdp_temperature_deg = 0;

int NEO_Power = 11;
int NEO_PIN  = 12;
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial1.setFIFOSize(1024);
  Serial.begin(460800);
  Serial1.begin(460800);

  pinMode(16, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(17, OUTPUT);

  delay(100);
  Wire.setClock(400000);
  Wire.begin();

  sdp.begin(Wire, SDP8XX_I2C_ADDRESS_0);
  uint32_t productNumber;
  uint8_t serialNumber[8];
  uint8_t serialNumberSize = 8;
  sdp.stopContinuousMeasurement();
  error = sdp.readProductIdentifier(productNumber, serialNumber, serialNumberSize);
  if (error)
  {
    Serial.print("Error trying to execute readProductIdentifier(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("ProductNumber:");
    Serial.print(productNumber);
    Serial.print("\t");
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      Serial.print(serialNumber[i], HEX);
    }
    Serial.println();
  }
  error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  if (error) {
    Serial.print(
      "Error trying to execute "
      "startContinuousMeasurementWithDiffPressureTCompAndAveraging(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }

  Serial.println("DPS310");
  while (! dps.begin_I2C()) {             // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    delay(100);
  }
  Serial.println("DPS OK!");
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);

  pinMode(NEO_Power, OUTPUT);
  digitalWrite(NEO_Power, HIGH);
  pixels.begin();

  sd.begin();
  
  while (SerialIN.available()) {
    SerialIN.read();
  }
}

const int readUART_BUF_SIZE = 256;
char readUART_BUF[256];
char sendUART_BUF[256];
void loop() {
  while (SerialIN.available()) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    int read_length = SerialIN.available();
    if (read_length >= readUART_BUF_SIZE - 1) {
      read_length = readUART_BUF_SIZE - 1;
    }
    SerialIN.readBytes(readUART_BUF, read_length);
    readUART_BUF[read_length] = '\0';
    sd.add_str(readUART_BUF);
    if (!SerialIN.available()) {
      delay(1);
    }
    pixels.clear();
    pixels.show();
  }

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
    dps.getEvents(&temp_event, &pressure_event);
    dps_pressure_hPa = pressure_event.pressure;
    dps_temperature_deg = temp_event.temperature;

    dps_altitude_m = (powf(1013.25 / dps_pressure_hPa, 1 / 5.257) - 1) * (dps_temperature_deg + 273.15) / 0.0065;

    float _sdp_differentialPressure_Pa = 0;
    float _sdp_temperature_deg = 0;
    error = sdp.readMeasurement(_sdp_differentialPressure_Pa, _sdp_temperature_deg);
    sdp_differentialPressure_Pa = _sdp_differentialPressure_Pa;
    sdp_temperature_deg = _sdp_temperature_deg;
    if (error)
    {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    }
    else
    {
      sdp_airspeed_ms = sqrt(abs(2.0 * sdp_differentialPressure_Pa / (0.0034837 * 101325.0 / (dps_temperature_deg + 273.5))));
    }
    sprintf(sendUART_BUF, "%.2f,%.2f,%.2f,%.2f,%.2f\n", dps_pressure_hPa, dps_temperature_deg, dps_altitude_m, sdp_differentialPressure_Pa, sdp_airspeed_ms);
    SerialOUT.print(sendUART_BUF);

    pixels.clear();
    pixels.show();
  }
  sd.flash();
  Serial.println(sdp_airspeed_ms);
}

/*void LEDwrite(int status) {
  digitalWrite(25, status);
  digitalWrite(17, status);
  digitalWrite(16, status);
}*/

void LEDtoggle(){
  digitalWrite(25, !digitalRead(25));
  digitalWrite(17, !digitalRead(25));
  digitalWrite(16, !digitalRead(25));
}
