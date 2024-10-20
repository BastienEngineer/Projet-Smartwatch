/// Configuration de Blynk Cloud (Template avec le device)
#define BLYNK_TEMPLATE_ID "XXXXXXXXXX"
#define BLYNK_TEMPLATE_NAME "XXXXXXXXX"
#define BLYNK_AUTH_TOKEN "XXXXXXXXXXXXXXXXXXXXXXXXXXXX"
#define BLYNK_PRINT Serial // Enables Serial Monitor

/// Libraries
#include <Wire.h>
#include <WiFi.h>;
#include <WiFiClient.h>;
#include <ThingSpeak.h>;
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include "MAX30105.h"
#include <Adafruit_MLX90614.h>
#include "spo2_algorithm.h"
#include "heartRate.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/// Configuration Adafruit
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "XXXXXXXXX"
#define AIO_KEY         "XXXXXXXXXXXXXXXXXXXXXXXXX" 

/// Define
#define MAX_BRIGHTNESS 255

/// Connexion Wi-Fi 
char WLAN_SSID[] = "XXXXXXXXXXX"; // Nom de notre Wi-Fi
char WLAN_PASS[]="XXXXXXXXXXX"; // Mot de passe Wi-Fi

/// Instance des classes  
BlynkTimer timer;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;
WiFiClient  client;
DHT dht(23, DHT11);
Adafruit_MPU6050 mpu;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish cal = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/calories");

/// Configuration ThingSpeak - sensor channel field
unsigned long myChannelNumber = 0; // Channel Number
const char * myWriteAPIKey = "XXXXXXXXXXX"; // Write API Key
unsigned int FieldNumber = 1; 

/// Variables globales
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute; // Battements par minute
int beatAvg; // Moyenne de bpm
int x,y,z; // Values of the accelerometer
float acceleration;
int lastButtonState;
int currentButtonState;
unsigned long t; // current time 
unsigned long tempsF; // final time
bool activite=false; // verification de l'activité
int cpt=0; // compteur pour une double vérification
float calories=0;
float coeff_met = 0; // coefficient de l'equation MET

/// Fonction pour envoyer les données à Blynk toutes les secondes
void increment() {
  Blynk.virtualWrite(V0, "Acceleration x = " + String(x) + " y = " + String(y) + " z = " + String(z));
  Blynk.virtualWrite(V1, mlx.readObjectTempC());
  Blynk.virtualWrite(V2, dht.readTemperature());
  Blynk.virtualWrite(V3, dht.readHumidity());
  Blynk.virtualWrite(V4, beatsPerMinute);
}

void MQTT_connect();

void setup() {
   Serial.begin(115200);
   Blynk.begin(BLYNK_AUTH_TOKEN, WLAN_SSID, WLAN_PASS); // Connexion entre Blynk et l'ESP32 en Wi-Fi
   timer.setInterval(1000L, increment); // la fonction increment() est lancée toutes les 1000 ms soit 1 seconde

  // Connect to WiFi access point
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Initialize DHT
  dht.begin();

  //Initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize MLX
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };

  Serial.print("Emissivity = "); Serial.println(mlx.readEmissivity());
  Serial.println("================================================");

  // Initialize MAX3012
  if (particleSensor.begin()==false) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  // Initialize Button
  pinMode(18, INPUT_PULLUP);
  currentButtonState = digitalRead(18);
}

void loop() {
  
  // Fonctions à éxecuter
  bpm();
  get_norm_accel();
  met(); 
  Blynk.run(); // lance Blynk 
  timer.run(); // lance le timer en continu  
  
//  adafruit_mqtt();

//  ThingSpeak.writeField(myChannelNumber, FieldNumber, dht.readTemperature(), myWriteAPIKey);
//  ThingSpeak.writeField(myChannelNumber, 2, dht.readHumidity(), myWriteAPIKey);
//  ThingSpeak.writeField(myChannelNumber, 3, x, myWriteAPIKey);
//  ThingSpeak.writeField(myChannelNumber, 4, y, myWriteAPIKey);
//  ThingSpeak.writeField(myChannelNumber, 5, z, myWriteAPIKey);
//  ThingSpeak.writeField(myChannelNumber, 6, int(mlx.readObjectTempC()), myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber, 8, calories, myWriteAPIKey);
 
}

/// Adafruit Send calories
void adafruit_mqtt()
{
  MQTT_connect();
  cal.publish(calories, sizeof(float));
  delay(2000);
}

/// Processing the acceleration & the MET coeff 
void get_norm_accel()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  x = a.acceleration.x;
  y = a.acceleration.y;
  z = a.acceleration.z;
  // The norm of the acceleration
  acceleration = sqrt(pow(a.acceleration.x,2)+pow(a.acceleration.y,2)+pow(a.acceleration.z,2));
  if(acceleration >= 10.5)
  {
    coeff_met = 3;
  }
  else
  {
    coeff_met = 1.5;
  }
}

/// Compute the calories with the time of the activity (button)
void met()
{
  lastButtonState = currentButtonState; // save the last state
  currentButtonState = digitalRead(18); // read new state
  // Presse sur le bouton pour commencer l'activité
  if(lastButtonState == HIGH && currentButtonState == LOW && !activite && cpt%2==0) {
    activite=true;
    cpt++;
    t = millis();
    Serial.println("Debut de l'activite");
  }
  // Presse sur le bouton pour terminer l'activité
  else if(lastButtonState == HIGH && currentButtonState == LOW && activite && cpt%2!=0)
  {
    activite=false;
    cpt++;
    tempsF = millis();
    // Durée de l'activité
    unsigned long duree = tempsF - t;
    Serial.println("Fin de l'activite");
    Serial.print("Duree de l'activite : ");
    Serial.print(duree/1000);
    Serial.println(" s");
    // Formule de l'equation de MET
    calories = calories + (coeff_met * 70 * duree / (3600*1000)) ;
    Serial.println(calories);
  }
}

/// Compute the bpm
void bpm()
{
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if(irValue < 50000)
  {
    Serial.print("No finger?");
  }
  Serial.println();
}

/// Connexion with Adafruit MQTT
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(F("Connecting to MQTT... "));

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println(F("Retrying MQTT connection in 5 seconds..."));
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println(F("MQTT Connected!"));
}
