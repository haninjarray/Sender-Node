#include <SPI.h>                                    
#include <LoRa.h>
#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> //comminucation with I2C devices

//define the pins used by the transceiver module
#define ss 10 //5 
#define rst 5 //14
#define dio0 2
#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 22  (AM2302), AM2321


// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);
int counter = 0;
String lora_message = "";
Adafruit_MPU6050 mpu;
unsigned long previousTime = 0;

void setup() {

  Serial.begin(115200);
  dht.begin();

  //initialize Serial Monitor
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  //Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  Serial.println("");
  delay(100);

  while (!Serial);
  Serial.println("LoRa Sender");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  
  while (!LoRa.begin(866E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
   // Wait a few seconds between measurements.
   //Serial.println("-------1---------");


  delay(2000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Calcul de l'accélération totale
  float accelerationTotale = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)/9.81;
  
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // Conversion en secondes
  previousTime = currentTime;
  
  // Intégrer l'accélération pour obtenir la vitesse (attention aux erreurs cumulatives)
  static float VX = 0.0;
  static float VY = 0.0;
  static float VZ = 0.0;

  VX = a.acceleration.x * 0.01; // Temps d'échantillonnage de 0.01 secondes (10 ms)
  VY = a.acceleration.y * 0.01;
  VZ = a.acceleration.z * 0.01;
  
  float VitesseTotale = 10.0*sqrt(VX * VX + VY * VY + VZ * VZ);

//Serial.println("--------2--------");
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }



  //Serial.println("--------3--------");
  lora_message = String(t) + "a" + String(h) + "b" + String(accelerationTotale) + "c" + String(VitesseTotale) + "d" + String(counter) ;
  Serial.print("Sending packet: ");
  Serial.println(counter);
  Serial.println(lora_message);
  Serial.print("Accélération totale: "); Serial.println(accelerationTotale);
  Serial.println("----------------");
  delay(100);
  


  //Serial.println("-------4---------");
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  LoRa.print(lora_message);
  LoRa.endPacket();
  counter++ ;
  delay(100);
  //Serial.println("-------5---------");
}
