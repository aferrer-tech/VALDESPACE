/* 
 * Código utilizado por equipo ValdeSpace para lanzamiento nacional de CanSat2023. 
 * Envía por radio paquete de datos cada medio segundo con Temperatura, Presión atmosférica, altitud (gracias a BMP)y coordenadas (gracias a GPS)  
 * Desactica un electroimán cuando el CanSat está cayendo para apertura de bote con zeolitas.
 * Activa un buzzer para facilitar la recuperación del CanSat. 
 */
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

int bmp_hay = 1;

#define SPEEDS_COUNT 5
#define SPEED_THRESHOLD -6
#define MINIMUM_TIME_TO_CHECK_CANSAT 60000
#define MAGNET_DEACTIVATION_TIME 5400000
float speedReadings[SPEEDS_COUNT];
int speedIndex = 0;
float lastAltitude = 0;
float avgSpeed = 0;
unsigned long lastAltitudeTime = 0; // Tiempo de la última lectura
bool fallingDetected = false;
bool isGateOpen = false;
double latitude = 0;
double longitude = 0;
static const int RXPin = 3, TXPin = 4;
static const uint32_t GPSBaud = 9600;
const int magnet_pin = 9;


uint8_t paquete = 0;
unsigned long elapsed_time;
unsigned long send_last = 0;
#define send_interval 500

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
	Serial.begin(9600);
	ss.begin(GPSBaud);
	
	while ( !Serial ) delay(100);   // wait for native usb
	unsigned status;
	status = bmp.begin();//bmp.begin(0x76)
	if (!status) {
		Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
						"try a different address!"));
		Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
		Serial.print("      ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
		Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
		Serial.print("      ID of 0x60 represents a BME 280.\n");
		Serial.print("      ID of 0x61 represents a BME 680.\n");

		bmp_hay = 0;
		
	}

	/* Default settings from datasheet. */
	if (bmp_hay){ 
		bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
					Adafruit_BMP280::SAMPLING_X2,  /* Temp. oversampling */
					Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling*/
					Adafruit_BMP280::FILTER_X16,   /* Filtering. */
					Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
	}
	pinMode(magnet_pin, OUTPUT);
	digitalWrite(magnet_pin, HIGH);

}

void loop() {
	elapsed_time = millis();
	
	if(elapsed_time > send_last + send_interval) {
		send_last = elapsed_time;
	
		fallingDetected = isFalling();
		if (elapsed_time > MINIMUM_TIME_TO_CHECK_CANSAT && paquete % 5 == 0 && isGateOpen){
			tone(8,523,500);
		}
		if ((fallingDetected || elapsed_time > MAGNET_DEACTIVATION_TIME) && !isGateOpen) {
			digitalWrite(magnet_pin, LOW); // desactivar el electroimán
			isGateOpen = true;
		}
	
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
		
	
		if (millis() > 5000 && gps.charsProcessed() < 10){
			Serial.println(F("#VALDE, No GPS detected: check wiring."));
			while(true);
		}
		//formato trama, campos separados cpor comas CSV, con delimitador inicio y fin y numero de secuencia 
		//delimitador inicio, #seq, temp, presion, altitud, velocidad, latitud, longitud, tiempo_transcurrido, puerta_abierta, delimitador fin
		Serial.print("#VALDE");   //delimitador de inicio de trama #...
		Serial.print(",");
		Serial.print(paquete);    //numero de secuencia
		Serial.print(",");
		if (bmp_hay){
			Serial.print(bmp.readTemperature());
			Serial.print(",");
			Serial.print(bmp.readPressure());
			Serial.print(",");
			Serial.print(bmp.readAltitude(985.5));
			Serial.print(",");
			Serial.print(avgSpeed);
			Serial.print(",");
		} else {
			Serial.print("0.0");
			Serial.print(",");
			Serial.print("0.0");
			Serial.print(",");
			Serial.print("0.0");
			Serial.print(",");
			Serial.print(avgSpeed);
			Serial.print(",");
		}
		Serial.print(latitude,6);
		Serial.print(",");
		Serial.print(longitude,6);
		Serial.print(",");
		Serial.print(elapsed_time);
		Serial.print(",");
		Serial.print(isGateOpen);
		Serial.print(",");
		Serial.println("$PACE"); //delimitador de fin ..$
		paquete++;
	}
}


bool isFalling() {
	bool result = false;

	float currentAltitude = bmp.readAltitude(985.5); // Obtener la altitud actual en metros
	unsigned long currentTime = millis(); // Obtener el tiempo actual en milisegundos

	// Calcular la velocidad promedio de la caída desde la última lectura
	float altitudeDiff = currentAltitude - lastAltitude;
	unsigned long timeDiff = currentTime - lastAltitudeTime;
	float currentSpeed = altitudeDiff / ((float) timeDiff / 1000.0);

	// Agregar la nueva lectura de altitud al array y actualizar el índice
	speedReadings[speedIndex] = currentSpeed;
	speedIndex = (speedIndex + 1) % SPEEDS_COUNT;

	// Calcular la velocidad promedio de la caída a partir del promedio móvil de las últimas 5 lecturas
	float speedSum = 0;
	for (int i = 0; i < SPEEDS_COUNT; i++) {
		speedSum += speedReadings[i];
	}
	// esperar MINIMUM_TIME_TO_CHECK_CANSAT para tener al menos 5 lecturas de la velocidad
	if (elapsed_time > MINIMUM_TIME_TO_CHECK_CANSAT) {
		avgSpeed = speedSum / (float) SPEEDS_COUNT;
	} 
  
	// Verificar si la velocidad de caída supera el umbral
	if (avgSpeed < SPEED_THRESHOLD && (elapsed_time > MINIMUM_TIME_TO_CHECK_CANSAT)) {
		result = true;
	}

	// Actualizar la última altitud y tiempo registrados
	lastAltitude = currentAltitude;
	lastAltitudeTime = currentTime;

	return result;
}
