#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
int paquete=0;

static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;

const int MQ_PIN = A0;      // Pin del sensor
const int RL_VALUE = 5;      // Resistencia RL del modulo en Kilo ohms
const int R0 = 10;          // Resistencia R0 del sensor en Kilo ohms

// Datos para lectura multiple
const int READ_SAMPLE_INTERVAL = 100;    // Tiempo entre muestras
const int READ_SAMPLE_TIMES = 5;       // Numero muestras

// Ajustar estos valores para vuestro sensor según el Datasheet
// (opcionalmente, según la calibración que hayáis realizado)
const float X0 = 200;
const float Y0 = 1.7;
const float X1 = 10000;
const float Y1 = 0.28;

// Puntos de la curva de concentración {X, Y}
const float punto0[] = { log10(X0), log10(Y0) };
const float punto1[] = { log10(X1), log10(Y1) };

// Calcular pendiente y coordenada abscisas
const float scope = (punto1[1] - punto0[1]) / (punto1[0] - punto0[0]);
const float coord = punto0[1] - punto0[0] * scope;

double latitude = 0;
double longitude = 0;
float concentration = 0;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
 
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();//bmp.begin(0x76)
  if (!status) {
	Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                 	"try a different address!"));
	Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  Serial.print("    	ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
	Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("    	ID of 0x60 represents a BME 280.\n");
    Serial.print("    	ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
              	Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
              	Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling*/
              	Adafruit_BMP280::FILTER_X16,   /* Filtering. */
              	Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

}

void loop()
{
  //GPS
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
	if (gps.encode(ss.read())){
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    } else {
      latitude = 0;
      longitude = 0;
    }
  }
  	

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
	Serial.println(F("No GPS detected: check wiring."));
	while(true);
  }

  // Concentracion
  float rs_med = readMQ(MQ_PIN);      // Obtener la Rs promedio
  concentration = getConcentration(rs_med/R0);   // Obtener la concentración
  
  // Mostrar el valor de la concentración por serial
  //Serial.print("Concentración: ");
  //Serial.println(concentration);
 
  //BMP280
  Serial.print(paquete);
	Serial.print(",");
	Serial.print(bmp.readTemperature());
	Serial.print(",");
	Serial.print(bmp.readPressure());
	Serial.print(",");
	Serial.print(bmp.readAltitude(985.5));
	Serial.print(",");
  Serial.print(concentration);
	Serial.print(",");
  Serial.print(latitude,6);
	Serial.print(",");
  Serial.print(longitude,6);
	Serial.print(",");
	Serial.println("VALDESPACE");
    
	paquete++;

    

	delay(1000);
}


// Obtener la resistencia promedio en N muestras
float readMQ(int mq_pin)
{
   float rs = 0;
   for (int i = 0;i<READ_SAMPLE_TIMES;i++) {
      rs += getMQResistance(analogRead(mq_pin));
      delay(READ_SAMPLE_INTERVAL);
   }
   return rs / READ_SAMPLE_TIMES;
}

// Obtener resistencia a partir de la lectura analogica
float getMQResistance(int raw_adc)
{
   return (((float)RL_VALUE / 1000.0*(1023 - raw_adc) / raw_adc));
}

// Obtener concentracion 10^(coord + scope * log (rs/r0)
float getConcentration(float rs_ro_ratio)
{
   return pow(10, coord + scope * log(rs_ro_ratio));
}
