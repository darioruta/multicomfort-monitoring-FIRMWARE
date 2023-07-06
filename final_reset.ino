
/*   
Hardware Setup: 
LED Pinout 

LED 
digital                 12              

RGB           
Red                      11
Blue                     9
Green                    10



Sensor Type         Arduino PIN

Wind                   
 GND                    GND
 +V                     5V
 RV                     A1    
 TMP                    A0    

Lux    
 G                      GND
 V                      5V
 S                      A2 

Humidity + Temp      
 G                      GND
 V                      5V
 out                    D8 

Sensirion CO2                  
 GND - Black          GND
 V+  - Red            5V
 SCL - Yellow         A5    
 SDA - Green          A4 

Sound Sensor                 
 G                    GND
+                    5V
AO                   A3   

Termocuple                  
GND                  GND
VCC                  5V
SCk                  D6
CS                   D5
SO                   D4

PM10                 
https://electropeak.com/learn/interfacing-gp2y1010au0f-optical-dust-sensor-module-with-arduino/
Connection on A6, D2
  
 */
#include <DHT.h>
#include <Arduino.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>
#include <WiFi.h>
#include "max6675.h"
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino.h>

#define DHTPIN 8   //Pin a cui è connesso il sensore
#define DHTTYPE DHT22   //Tipo di sensore che stiamo utilizzando (DHT22)
#define analogPinForTMP   0
#define analogPinForRV    1   // change to pins you the analog pins are using
#define LIGHTSENSORPIN    A2 //Ambient light sensor reading 



// to calibrate your sensor, put a glass over it, but the sensor should not be
// touching the desktop surface however.
// adjust the zeroWindAdjustment until your sensor reads about zero with the glass over it. 

//const float zeroWindAdjustment =  .2; // negative numbers yield smaller wind speeds and vice versa.
const float zeroWindAdjustment =  -0.2; // NANO
int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
int LED_int = 12;
int RGB_R = 11; 
int RGB_G = 10; 
int RGB_B = 9;
float WindSpeed_MPH;
int chk;
float hum;  //Variabile in cui verrà inserita la % di umidità
float temp; //Variabile in cui verrà inserita la temperatura
int thermoDO = 4;
int thermoCS = 5;
int thermoCLK = 6;
int measurePin = 6; //Connect dust sensor to Arduino A0 pin
int ledPower = 2;   //Connect 3 led driver pins of dust sensor to Arduino D2
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

int timeout = 4; 



// Informazioni per connettersi alla rete di casa              
char ssid[] = "Vodafone-A74217872"; 
char pass[] = "us2948vhpzy3expx"; 

// Mio Hotspot del telefono
//char ssid[] = "AndroidAP"; 
//char pass[] = "luho9810"; 


int status = WL_IDLE_STATUS;            





         
// Impostazione indirizzo IP del server MQTT e la porta
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
// Topic MQTT
//const char* topic_subscribe = "testing_nano_ip_project_LED_status";
const char* topic_publish = "testing_nano_ip_project_79139719_Politecnico_Torino";
const char* clientName = "arduinoClient";
// attesa tra un controllo e l'altro
unsigned long tempoPrecedente = 0;
int i = 0;
//str messaggio = ""



DHT dht(DHTPIN, DHTTYPE); //Inizializza oggetto chiamato "dht", parametri: pin a cui è connesso il sensore, tipo di dht 11/22
SensirionI2CScd4x scd4x;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
// Dichiarazione dell'oggetto WiFiClient per la connessione WiFi
WiFiClient wifiClient;
// Oggetto PubSubClient per connessione MQTT
PubSubClient mqttClient(wifiClient);

void printUint16Hex(uint16_t value) {
  
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
    
}

void color_assign (unsigned char rosso, unsigned char verde, unsigned char blu)
{
 analogWrite(RGB_R, rosso); 
 analogWrite(RGB_B, blu); 
 analogWrite(RGB_G, verde); 
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
   
    //Serial.print("Serial: 0x");
    //printUint16Hex(serial0);
    //printUint16Hex(serial1);
    //printUint16Hex(serial2);
    //Serial.println();
    
}

void reconnect() {
  // while (!mqttClient.connected()) {
    int i = 0; 
    while (i < 10) {
    
    
    if (mqttClient.connect("arduinoClient")) {
      mqttClient.subscribe(topic_publish);
    } else {
      Serial.println("RESET");
      NVIC_SystemReset();
    }
    i = i +1; 
  }
}


// Callback per messaggi ricevuti
void callback(char* topic, byte* payload, unsigned int length) {

  //Serial.println(topic);
  String mess = "";
  
  for (int i = 0; i < length; i++) {
    mess = mess + (char)payload[i];
  }
  int state = mess.toInt();
  if(state == 1){
color_assign(255, 0, 0); // red }
  }else if (state == 2){
    color_assign(237,109,0); // orange
  }else if (state == 3){
    color_assign(0,255,0); // green 
  }else{
    //NVIC_SystemReset();
  }
  //Serial.println();
}

void inviaMessaggio(float WindSpeed_MPH, float lux_value, float hum, float temp, int acoustic_value, uint16_t co2, float mrt, float dustDensity) {
  // Invia un messaggio al topic specificato
  String wind;
  wind = String(WindSpeed_MPH);
  String luce;
  luce = String(lux_value);
  String umi;
  umi = String(hum);
  String temperatura;
  temperatura = String(temp);
  String suono;
  suono = String(acoustic_value);
  String anidride;
  anidride = String(co2);
  String termocoppia;
  termocoppia = String(mrt);
  String polvere;
  polvere = String(dustDensity);


  
 // String messaggio = "{\"kitID\": -1,\"timestamp\" : \"2023-12-12 00:00:00\",\"data\": {\"e\":" + wind + ",\"g\":" + luce + ",\"b\":" + umi + ",\"a\":" + temperatura + ",\"f\":" + suono +",\"d\":" + anidride + ",\"h\":" + termocoppia + ",\"c\":" + polvere + "}}";
  
  String messaggio = "{\"kitID\": -1,\"timestamp\" : \"\",\"data\": {\"e\":" + wind + ",\"g\":" + luce + ",\"b\":" + umi + ",\"a\":" + temperatura + ",\"f\":" + suono +",\"d\":" + anidride + ",\"h\":" + termocoppia + ",\"c\":" + polvere + "}}";
  
  mqttClient.publish(topic_publish, messaggio.c_str(), 2);
  Serial.println("Messaggio inviato");
}





unsigned long previousMillis = 0;
const long interval = 30000;  

void setup() {
  // Da togliere 
  Serial.begin(9600);


  digitalWrite(LED_int, LOW);
  //Serial.println("--------------   Start Setup   --------------");
  pinMode(LIGHTSENSORPIN,  INPUT); 
  pinMode(ledPower,OUTPUT);  



  dht.begin();
  pinMode(LED_int, OUTPUT);
  pinMode(RGB_R, OUTPUT); 
  pinMode(RGB_G, OUTPUT); 
  pinMode(RGB_B, OUTPUT);


  // Connect Wi-Fi network:
  while (status != WL_CONNECTED) {
    digitalWrite(LED_int, HIGH);
    //Serial.print("Attempting to connect to network: ");
    //Serial.println(ssid);
   
    status = WiFi.begin(ssid, pass);
    digitalWrite(LED_int, HIGH);
    delay(2000);
  }
  
  Serial.println("You're connected to the network");
  //Serial.println("---------------------------------------");
  //Serial.println("Board Information:");
  
  IPAddress ip = WiFi.localIP();
  /*
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println();
  Serial.println("Network Information:");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.println("---------------------------------------");
  */

  digitalWrite(LED_int, LOW);
  // Connessione Server MQTT  

  mqttClient.setServer(mqtt_server, mqtt_port);
  //mqttClient.setCallback(callback);
  reconnectMQTT();

  // Accesnione LED di stato 
  color_assign(0,255,0); // green
  digitalWrite(LED_int, HIGH);


    
    Wire.begin();
    uint16_t error;
    char errorMessage[256];
    scd4x.begin(Wire);
    // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        // Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        //NVIC_SystemReset();
        //errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    }
    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
      //NVIC_SystemReset();
        //Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        //Serial.println(errorMessage);
    } else {
        // printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
      // NVIC_SystemReset();
        // Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        //errorToString(error, errorMessage, 256);
        // Serial.println(errorMessage);
    }

    // Serial.println("Waiting for first measurement... (5 sec)");
  Serial.println("Setup End ");

  delay(1000);
}


int reset_timeout = 15; 
void loop() {


  

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval){      // read every 
     previousMillis = currentMillis;


    if (reset_timeout <= 0){
      NVIC_SystemReset();
    }


    //reconnect();    
     if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    // Wind Sensor    
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  
    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39
    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  
    WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   
   
    //Serial.print("   WindSpeed MPH  = ");
    //Serial.println((float)WindSpeed_MPH);

  // Lux Sensor     
    float lux_value = analogRead(LIGHTSENSORPIN) * 12 ; 
    //Serial.print("Lux value  = ");
    //Serial.println(lux_value);   
  
  // Humidity and Temperature Sensor 
    float hum = dht.readHumidity();
    float temp= dht.readTemperature() - 1;
    
    //Serial.print("Umidità: ");
    //Serial.print(hum);
    //Serial.print(" %, Temp: ");
    //Serial.print(temp);
    //Serial.println(" Celsius");    
    
    uint16_t error;
    char errorMessage[256];

    // CO2 Sensor Measurment
    uint16_t co2;
    float temperature;
    float humidity;
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        //Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        co2 = 400;
        //Serial.println(errorMessage);
    } else if (co2 == 0) {

        //Serial.println("Invalid sample detected, skipping.");
    } else {
        /*
        Serial.print("Co2:");
        Serial.print(co2);
        Serial.print("\t");
        Serial.print("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.print("Humidity:");
        Serial.println(humidity);
        */
      
    }

   // Sound Sensor Measurment   
   int acoustic_value = (analogRead(A3)/10) - 20;
   //Serial.print("Acoustic value : ");
   //Serial.println(analogRead(A3));
 

   // Termocouple Measurment 
   float mrt = thermocouple.readCelsius() - 1;
   //Serial.print("Termocouple  = "); 
   //Serial.println(thermocouple.readCelsius());

   // PM10
   digitalWrite(ledPower,LOW); // power on the LED
   delayMicroseconds(samplingTime);
   voMeasured = analogRead(measurePin); // read the dust value
   delayMicroseconds(deltaTime);
   digitalWrite(ledPower,HIGH); // turn the LED off
   delayMicroseconds(sleepTime);
   calcVoltage = voMeasured * (5.0 / 1024.0);
   dustDensity = 170 * calcVoltage - 0.1;
   //Serial.print("Air Quality : ");
   //Serial.println(dustDensity); // unit: ug/m3
  

    // Posso provare a fare la connessione MQTT qua, se è connesso mando il messaggio
   inviaMessaggio(WindSpeed_MPH, lux_value, hum, temp, acoustic_value, co2, mrt, dustDensity);
   Serial.print("Message before reset : ");
    Serial.println(reset_timeout);
   reset_timeout = reset_timeout - 1; 
   //timeout = timeout -1;

    
    //delay(10000);
    //NVIC_SystemReset();


    //if (timeout <= 0) {
    //NVIC_SystemReset(); // Riavvia la scheda Arduino Nano 33 IoT
    //}


  } 
  mqttClient.loop();

}

 int timout = 4;
 void reconnectMQTT() {
  while (!mqttClient.connected()) {

    if(timeout <= 0){
      Serial.print("RESET"); 
      NVIC_SystemReset();

    }

    Serial.println("Connessione al broker MQTT...");
    if (mqttClient.connect(clientName)) {
      Serial.println("Connesso al broker MQTT nel reconnect MQTT");
    } else {
      Serial.print("MQTT Reconnection"); 
      timout = timeout -1;
      delay(5000);
    }
  }
 }



// void loop() {   
//   color_assign(255, 0, 0); // red 
//   color_assign(237,109,0); // orange
//  color_assign(0,255,0); // green
// }




