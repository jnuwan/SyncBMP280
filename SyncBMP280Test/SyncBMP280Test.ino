#include "SyncBMP280.h"

SyncBMP280 bmp;

void setup() {
  Serial.begin(9600);
  bmp.begin();
  bmp.setSampling(SyncBMP280::MODE_NORMAL,     /* Operating Mode. */
                SyncBMP280::SAMPLING_X2,     /* Temp. oversampling */
                SyncBMP280::SAMPLING_X16,    /* Pressure oversampling */
                SyncBMP280::FILTER_X16,      /* Filtering. */
                SyncBMP280::STANDBY_MS_500); /* Standby time. */
	Serial.println("\n~~ Testing BMP280 ~~");
}

void loop() {
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());Serial.println(" *C");
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());Serial.println(" Pa");
  Serial.print(F("Altitude = "));
  Serial.print(bmp.readAltitude());Serial.println(" m");
	Serial.println();
  delay(2000);
}