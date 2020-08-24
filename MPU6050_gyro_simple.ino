#include <english.h>
#include <oled.h>
#include <resources.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"



#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiSpi.h"

// pin definitions
//int count = 0;
#define CS_PIN  A5
#define RST_PIN A4
#define DC_PIN  A3
#define BUFSIZE 128   // Size of the read buffer for incoming data
#define VERBOSE_MODE true // If set to 'true' enables debug output
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN -1 // Optional, set to -1 if unused

#ifdef Serial1    // this makes it not complain on compilation if there's no Serial1
#define BLUEFRUIT_HWSERIAL_NAME      Serial1
#endif

#define BLUEFRUIT_UART_MODE_PIN 12 // Set to -1 if unused
#define BLUEFRUIT_SPI_CS               A2
#define BLUEFRUIT_SPI_IRQ              0
#define BLUEFRUIT_SPI_RST A1 // Optional but recommended, set to -1 if unused

#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI 11
#define CS_PIN  A5
#define RST_PIN A4
#define DC_PIN  A3
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}
SSD1306AsciiSpi oled;
MPU6050 mpu;
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;
int32_t count;

void checkSettings()
{
  Serial.println();

  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Clock Source:      ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }

  Serial.print(" * Gyroscope:         ");
  switch (mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  }

  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());

  Serial.println();

}

void setup()
{
  Serial.begin(115200);
  oled.begin(&Adafruit128x64, CS_PIN, DC_PIN, RST_PIN);
  oled.setFont(Adafruit5x7);
  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);




    Serial.println();
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    mpu.setThreshold(3);

    // Check settings
    checkSettings();
  }


  while (!Serial); // required for Flora & Micro
  delay(500);

  boolean success;
,,,,,,,,,,,,,,,,,,,,,,,,,,,

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ) {
    error(F("Couldn't factory reset"));
  }

 
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();


  Serial.println(F("Setting device name to 'Bluefruit HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Bluefruit HRM")) ) {
    error(F("Could not set device name?"));
  }

  
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &count);
  if (! success) {
    error(F("Could not add HRM service"));
  }

 
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x1, MIN_LEN=1, VALUE=0"), &hrmMeasureCharId);
  if (! success) {
    error(F("Could not add HRM characteristic"));
  }

  
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=5"), &hrmLocationCharId);
  if (! success) {
    error(F("Could not add BSL characteristic"));
  }

  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

}


void loop()
{

  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();

  Serial.print(" Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);
  oled.setCursor(1, 1);
  oled.println("  Portakal spor");
  oled.print("Adim sayiniz: ");
  float metre;
  float cal;
  float boy = 1.80;
  float kilo = 75.0;


  if (normGyro.XAxis <= 100 && normGyro.YAxis >= 150)
  {
    count++;
    delay(200);
  }
  if ( normGyro.XAxis >= 90 && normGyro.YAxis <= 180)
  {
    count++;
    delay(200);
  }
  cal = count * 0.050;
  Serial.print("Adim sayisi= ");
  Serial.println(count);
  oled.println(count);
  oled.println("------");
  oled.print("yakilan kalori:");
  oled.println(cal);
  oled.println("------");
  Serial.println(cal);
  metre = count * 0.005;
  oled.print("km:");
  oled.print(metre);


  ble.print( F("AT+GATTCHAR=") );
  //ble.print( hrmMeasureCharId );
  ble.println(count);


  delay(200);


}
