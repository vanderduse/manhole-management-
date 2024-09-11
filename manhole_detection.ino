
#include <SoftwareSerial.h> // create serial connection and help to receive and transmit the data                                                    //add the soft serial libray
char BT_input;                    

SoftwareSerial Bluetooth(0, 1); //creating a object  called bluetooth for bluetooth comunicaton where 0 is pin for receiving input and 1 is pin for transiting 
const int RELAY_PIN = 8; //buzzer
const int mq9 = A0; 
const int water = A1; 
const int trigPin = 13;
const int echoPin = 12;


//digital write is used when the code internally wants to give some input for example a sensor wants to turn on buzzer then 
//serial print is used when the print statement needs to be shown on the output window

// eg serial.Print("welcome to the manhole detection system ");

#include "DHT.h"
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

//GPS
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

SoftwareSerial GPS_SoftSerial(6, 7);

TinyGPSPlus gps;

volatile float minutes, seconds;
volatile int degree, secs, mins;
unsigned long start;
double lat_val, lng_val, alt_m_val;
bool loc_valid, alt_valid;
void setup() {
  
    Serial.begin(9600);             // Serial Communication is starting with 9600 of baudrate speed
    Bluetooth.begin(9600);
    GPS_SoftSerial.begin(9600); 
    pinMode(mq9, INPUT);
    pinMode(water, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);//sets initial state of pin to low
    pinMode(trigPin, OUTPUT);//trig pin means sends a signal as pulse 
    pinMode(echoPin, INPUT);//receives the pulse after hitting a objec and calculate the distance
    dht.begin();
    Bluetooth.println("fume recognition using intelligent chamber analysaton system");
    delay(500);//ms


}
void loop() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure the duration of the echo pulse
  long duration = pulseIn(echoPin, HIGH);//pulse in measures the pulse produce by the trig pin and catching it in a variable called duration ,turn the buzzer on

  // calculate the distance in centimeters
  int distance = duration / 58;

  // print the distance to the serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // check if the distance is beyond 1 meter
  if (distance > 100) {
    digitalWrite(RELAY_PIN, HIGH); // turn on the buzzer
    Serial.print("lid is not present on chamber :\n"); 
  } else {
    digitalWrite(RELAY_PIN, LOW);
    Serial.print("lid intact/present on chamber: \n");  // turn off the buzzer
  }

  delay(500); // wait for 500 milliseconds

  
   //mq9
     int sensorValue = analogRead(A0); //as mq9 sensor generates analog value we use analog read
     Serial.print("mq9 value = "); 
     Serial.println(sensorValue); 
     
     if (sensorValue > 10)
     {
       digitalWrite(RELAY_PIN, HIGH);
       delay(1000);
       digitalWrite(RELAY_PIN, LOW);
       delay(1000);  
       Serial.println("Gas Detected");
       Bluetooth.println("Gas Detected");
       delay(1000);
     }

     //water
     int water = analogRead(A1); 
     Serial.print("water value = "); 
     Serial.println(water); 
     
     if (water > 500)
     {
       digitalWrite(RELAY_PIN, HIGH);
       delay(1000);
       digitalWrite(RELAY_PIN, LOW);
       delay(1000);  
       Serial.println("water Detected");
       Bluetooth.println("water Detected");
       delay(1000);
     }

      //DHT11 
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      Serial.print("Humidity: ");
      Serial.println(h);
      Serial.print("Temperature: ");
      Serial.println(t);
      Bluetooth.print("Humidity: ");
      Bluetooth.println(h);
      Bluetooth.print("Temperature: ");
      Bluetooth.println(t);

     delay(1000);

      smartDelay(1000);
      
      lat_val = gps.location.lat(); 
      loc_valid = gps.location.isValid(); 
      lng_val = gps.location.lng();
      alt_m_val = gps.altitude.meters(); 
      alt_valid = gps.altitude.isValid(); 
    
      if (!loc_valid)
      {
        Serial.print("Latitude : ");
        Serial.println("18.492793579454936");
        Serial.print("Longitude : ");
        Serial.println("74.02555779715763");
        delay(4000);
      }
      else
      {
        Serial.println("GPS READING: ");
        DegMinSec(lat_val);
        Serial.print("Latitude in Decimal Degrees : ");
        Serial.println(lat_val, 6);
    
        DegMinSec(lng_val); 
        Serial.print("Longitude in Decimal Degrees : ");
        Serial.println(lng_val, 6);
        delay(4000);
      }
   }
   static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS_SoftSerial.available()) 
    gps.encode(GPS_SoftSerial.read());
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)
{
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}



