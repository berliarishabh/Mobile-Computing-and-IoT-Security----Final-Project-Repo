/*

Name : Photon.ino

Authors: Omkar Seelam Reddy and Rishabh Berlia

Descprition: This program is used to communicate with 4 sensors on the Photon and Transmit their data to the ThingsSpeak Cloud.

All the libraries used are existing ones and are owned by the Respective authors.

*/




#include "Adafruit_DHT/Adafruit_DHT.h"
#include "ThingSpeak/ThingSpeak.h"
#include "photon-thermistor/photon-thermistor.h"
#include <math.h>
#include "ThingSpeak/ThingSpeak.h"

TCPClient client;
// On Particle: 0 - 4095 maps to 0 - 3.3 volts as it has a 12 bit ADCC

#define DHTPIN 1            //DHT Sesnor (Humidity) connected to D1
#define DHTTYPE DHT11
#define pin 4               //D4 connected to Fire Sesnor o/p
int photocellPin = 3;       //A3 connected to Photo Cell o/p

DHT dht(DHTPIN, DHTTYPE);
unsigned long myChannelNumber = 105460;              // Thingspeak Channel ID
const char * myWriteAPIKey = "ZLC926VS9GJSCV9W";     // Thingspeak API Key

Thermistor *thermistor;

void setup()
{
ThingSpeak.begin(client);
thermistor = new Thermistor(A0, 10000, 4095, 10000, 25, 3950, 5, 20);
pinMode(pin,INPUT);
dht.begin();

}

/*
This is the main().
All the sensor data is gathered here and then pushed to the ThingSpeak cloud
/*

void loop()

{
delay(2000);                                            //2 secs delay before start
float tempF = thermistor->readTempF();                  //Read Thermistor Value
int tempC=(tempF+0.0001-32.01)*float(5.0000/9.0000);    //Covert Temperature to Deg C
// Particle.publish(String("tempf"), String(tempF));
float h = dht.getHumidity();                            //Getting humidity value
//Particle.publish(String("H"), String(h));
int val=digitalRead(pin);                               //Read the Fire Sensor o/p
int inten=analogRead(photocellPin);                     //Read the Light Sensor o/p

int intensity;
if (inten<1000)                                         //Convert Analog Value into ON(0)or OFF(1) state
{
intensity=0;
}
else intensity=1;

ThingSpeak.setField(1,tempC);                           //Push all the Data to Cloud
ThingSpeak.setField(2,h);
ThingSpeak.setField(3,val);
ThingSpeak.setField(4,intensity);
ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
Particle.publish("A",String(inten));
delay(17000);                                           //The ThingSpeak API for Photon required a delay >15 secs in between POST operations.


}
