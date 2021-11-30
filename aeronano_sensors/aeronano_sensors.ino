/*
--------------------------------------------------------------------------
Version: 0.1



*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT_U.h>
#include <DHT.h>


#define LOGICAL_ON 1
#define LOGICAL_OFF 0

#define WATERLEVEL_DIST_PIN 4
#define WATERLEVEL_ECHO_PIN 5

float waterLevelPercentage = 100;
float waterLevelDistance = 0;
long calcWaterLevelTimer = 0;
int calcWaterLevelInterval = 300;
uint8_t waterEmptyDistance = 24;
uint8_t waterFullDistance = 5;


#define PANEL_TEMP_PIN 6      // what pin we're connected to
#define PUMP_TEMP_PIN 7
#define ROOM_TEMP_PIN 8
#define WATER_TEMP_PIN 9

#define DHTTYPE DHT22 // DHT 22  (AM2302)

DHT dhtPanel(PANEL_TEMP_PIN, DHTTYPE);
DHT dhtPump(PUMP_TEMP_PIN, DHTTYPE);
DHT dhtRoom(ROOM_TEMP_PIN, DHTTYPE);


OneWire oneWire(WATER_TEMP_PIN);

DallasTemperature sensors(&oneWire);



const int numChars = 600;
char receivedChars[numChars];
boolean newCommand = false;
int receivedCount = 0;

long sensorOutputTimer = 0;
int sensorOutputInterval = 2000;

void setup()
{


   pinMode(WATERLEVEL_DIST_PIN, OUTPUT);
   pinMode(WATERLEVEL_ECHO_PIN, INPUT);

   dhtPanel.begin();
   dhtPump.begin();
   dhtRoom.begin();
   Serial.begin(115200);
                   
   sensorOutputTimer = millis();
   calcWaterLevelTimer = millis();

  sensors.begin();

}

void loop()
{

   receiveCommand();
   processCommand();

   if (millis() - calcWaterLevelTimer > calcWaterLevelInterval)
   {
      calcWaterLevel();
   }

   if (millis() - sensorOutputTimer > sensorOutputInterval)
   {
      printSensorData();
   }
}

void calcWaterLevel()
{
   digitalWrite(WATERLEVEL_DIST_PIN, LOW);
   delayMicroseconds(2);
   digitalWrite(WATERLEVEL_DIST_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(WATERLEVEL_DIST_PIN, LOW);

   float duration = pulseIn(WATERLEVEL_ECHO_PIN, HIGH);
   waterLevelDistance = (duration * .0343) / 2;

   waterLevelPercentage = map(waterLevelDistance, waterFullDistance, waterEmptyDistance, 100.0, 0.0);
   calcWaterLevelTimer = millis();
}

void printSensorData()
{
   Serial.print("{{");
   //Temperature & Humidity
   float panel_hum = dhtPanel.readHumidity();
   float panel_temp = dhtPanel.readTemperature();
   Serial.print("PT:");
   Serial.print(panel_temp);
   Serial.print(";PH:");
   Serial.print(panel_hum);
   Serial.print(";");

   float pump_hum = dhtPump.readHumidity();
   float pump_temp = dhtPump.readTemperature();
   Serial.print("UT:");
   Serial.print(pump_temp);
   Serial.print(";UH:");
   Serial.print(pump_hum);
   Serial.print(";");

   float room_hum = dhtRoom.readHumidity();
   float room_temp = dhtRoom.readTemperature();
   Serial.print("RT:");
   Serial.print(room_temp);
   Serial.print(";RH:");
   Serial.print(room_hum);
   Serial.print(";");
   
   //Water temp
   sensors.requestTemperatures(); 
   float Celcius=sensors.getTempCByIndex(0);
   Serial.print("WT:");
    Serial.print(Celcius);
   Serial.print(";");

   //Water level
   Serial.print("WLP:");
   Serial.print(waterLevelPercentage);
   Serial.print(";");
   Serial.print("WLD:");
   Serial.print(waterLevelDistance);
   
   Serial.println("}}");

   sensorOutputTimer = millis();
}



void receiveCommand()
{
   static boolean recvInProgress = false;
   static int ndx = 0;
   char startMarker = '<';
   char endMarker = '>';
   char rc;

   while (Serial.available() > 0 && newCommand == false)
   {
      rc = Serial.read();

      if (recvInProgress == true)
      {
         if (rc != endMarker)
         {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars)
            {
               ndx = numChars - 1;
            }
         }
         else if (rc == endMarker)
         {
            receivedChars[ndx] = '\0'; // terminate the string
            recvInProgress = false;
            receivedCount = ndx;
            ndx = 0;
            newCommand = true;
         }
      }

      else if (rc == startMarker)
      {
         recvInProgress = true;
      }
   }
}

void processCommand()
{
   if (newCommand == true)
   {
      Serial.print("This just in ... ");
      Serial.print(receivedCount);
      Serial.print(": ");
      Serial.println(receivedChars);

      char cmd = receivedChars[0];

      //----------------------------------------
      // VALID INCOMING CMDS

      // CWFxx - C(onfig) W(ater) F(ull), Set water level sensor full distance in cm
      // CWExx - C(onfig) W(ater) E(mpty), Set water level sensor empty disance in cm

      //-----------------------------------------------
      if (receivedChars[0] == 'C') // Config value command
      {

         if (receivedChars[1] == 'W') // WATER CONFIG
         {
            int tiotal = receivedChars[3] - '0';
            int ental = receivedChars[4] - '0';
            int newValue = tiotal * 10 + ental;
            if (receivedChars[2] == 'F')
            {
               waterFullDistance = newValue;
               calcWaterLevel();
               printSensorData();

               Serial.print("Setting waterFullDistance to: ");
               Serial.println(newValue);
            }
            else if (receivedChars[2] == 'E')
            {
               waterEmptyDistance = newValue;
               calcWaterLevel();
               printSensorData();
               Serial.print("Setting waterEmptyDistance to: ");
               Serial.println(newValue);
            }
         }
         
       
      }
      
      newCommand = false;
   }
}
