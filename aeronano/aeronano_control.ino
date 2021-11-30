/*
--------------------------------------------------------------------------
Version: 0.1



*/

#include <Bounce2.h>
#include <DHT_U.h>
#include <DHT.h>

#define RELAY_ON false
#define RELAY_OFF true
#define LOGICAL_ON 1
#define LOGICAL_OFF 0

#define PUMP_BUTTTOM_PIN 10
#define MISTING_BUTTON_PIN 11
#define BYPASS_BUTTON_PIN 12

#define PUMP_INDEX 0
#define MISTING_INDEX 1
#define BYPASS_INDEX 2

#define PUMP_RELAY_PIN 7
#define MISTING_RELAY_PIN 8
#define BYPASS_RELAY_PIN 9

#define WATERLEVEL_DIST_PIN 5
#define WATERLEVEL_ECHO_PIN 4

float waterLevelPercentage = 100;
float waterLevelDistance = 0;
long calcWaterLevelTimer = 0;
int calcWaterLevelInterval = 300;
uint8_t waterEmptyDistance = 24;
uint8_t waterFullDistance = 5;

uint8_t buttonPins[] = {PUMP_BUTTTOM_PIN, MISTING_BUTTON_PIN, BYPASS_BUTTON_PIN};
uint8_t relayPins[] = {PUMP_RELAY_PIN, MISTING_RELAY_PIN, BYPASS_RELAY_PIN};

bool relayStates[] = {RELAY_OFF, RELAY_OFF, RELAY_OFF};

// 0 - idle, 1 - priming (waiting for pressure), 2 - misting, 3 - pausing (waiting to mist), 4-mixing (bypass open, mist closed)
#define STATE_IDLE 0
#define STATE_PRIMING 1
#define STATE_MISTING 2
#define STATE_PAUSE 3
#define STATE_MIXING 4

uint8_t state = STATE_IDLE;
bool simulation = 1;     // 0 - not simulation, 1 - simulation (no relays set, pressure simulated)
uint8_t minPressure = 7; //psi
uint8_t maxPressure = 11;
uint8_t curr_pressure = 0;
bool pressurize = false; // if true, pressure should be increased to maxPressure
long mockedPressureTimer;

uint8_t mistingScheduleRunTime = 30;         //seconds to mist
unsigned int mistingSchedulePauseTime = 300; //seconds to pause
long state_start_time;
long state_run_time;
float state_time_left;

Bounce debouncers[] = {Bounce(), Bounce(), Bounce()};

#define DHTPIN 6      // what pin we're connected to
#define DHTTYPE DHT22 // DHT 22  (AM2302)

DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

const int numChars = 600;
char receivedChars[numChars];
boolean newCommand = false;
int receivedCount = 0;

long sensorOutputTimer = 0;
int sensorOutputInterval = 1000;

void setup()
{
   pinMode(BYPASS_RELAY_PIN, OUTPUT);
   pinMode(PUMP_RELAY_PIN, OUTPUT);
   pinMode(MISTING_RELAY_PIN, OUTPUT);

   pinMode(WATERLEVEL_DIST_PIN, OUTPUT);
   pinMode(WATERLEVEL_ECHO_PIN, INPUT);

   digitalWrite(PUMP_RELAY_PIN, RELAY_OFF);
   digitalWrite(MISTING_RELAY_PIN, RELAY_OFF);
   digitalWrite(BYPASS_RELAY_PIN, RELAY_OFF);

   dht.begin();
   Serial.begin(115200);

   debouncers[PUMP_INDEX].attach(PUMP_BUTTTOM_PIN, INPUT_PULLUP);      // Attach the debouncer to a pin with INPUT_PULLUP mode
   debouncers[PUMP_INDEX].interval(25);                                // Use a debounce interval of 25 milliseconds
   debouncers[MISTING_INDEX].attach(MISTING_BUTTON_PIN, INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
   debouncers[MISTING_INDEX].interval(25);                             // Use a debounce interval of 25 milliseconds
   debouncers[BYPASS_INDEX].attach(BYPASS_BUTTON_PIN, INPUT_PULLUP);   // Attach the debouncer to a pin with INPUT_PULLUP mode
   debouncers[BYPASS_INDEX].interval(25);                              // Use a debounce interval of 25 milliseconds

   sensorOutputTimer = millis();
   calcWaterLevelTimer = millis();

   if (simulation)
   {
      mockedPressureTimer = millis();
   }
}

void loop()
{
   checkButtonPin(PUMP_INDEX);
   checkButtonPin(MISTING_INDEX);
   checkButtonPin(BYPASS_INDEX);
   manageState();
   managePressure();
   if (simulation)
   {
      manageMockedPressure();
   }
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

void manageState()
{

   // manage automatic state changes

   if (state == STATE_PRIMING)
   {
      if (curr_pressure >= maxPressure)
      {
         Serial.println("Priming done, starting misting");
         state_start_time = millis();
         state_run_time = 0;
         state = STATE_MISTING;
         state_time_left = 100;
         setRelayStateValue(PUMP_INDEX, RELAY_OFF);
         setRelayStateValue(MISTING_INDEX, RELAY_ON);
      }
   }
   else if (state == STATE_MISTING)
   {
      state_run_time = millis() - state_start_time;
      if (state_run_time >= mistingScheduleRunTime * 1000)
      {
         Serial.println("Misting done, pausing");
         setRelayStateValue(MISTING_INDEX, RELAY_OFF);
         state = STATE_PAUSE;
         state_time_left = 100;
         state_start_time = millis();

         // start
         pressurize = true;
      }
      else
      {
         /* Serial.println("Calculating time left");
         Serial.println(state_run_time);
         Serial.println(mistingScheduleRunTime * 1000);
         Serial.println((float)state_run_time / (float)(mistingScheduleRunTime * 1000));
*/
         state_time_left = 100 * (1 - (float)state_run_time / (mistingScheduleRunTime * 1000));
         // state_time_left = 100 * (1 - (state_run_time / (mistingScheduleRunTime * 1000)));
         //       Serial.println(state_time_left);
      }
   }
   else if (state == STATE_PAUSE)
   {
      state_run_time = millis() - state_start_time;
      //Serial.print("Pause run time: ");
      //Serial.print(state_run_time);
      if (state_run_time >= mistingSchedulePauseTime * 1000)
      {
         Serial.println("Pause done, starting misting");
         state = STATE_MISTING;
         state_time_left = 100;
         state_start_time = millis();
         setRelayStateValue(MISTING_INDEX, RELAY_ON);
      }
      else
      {
         /*  Serial.println("Calculating time left");
         Serial.println(state_run_time);
         Serial.println(mistingSchedulePauseTime * 1000);
         Serial.println((float)state_run_time / (float)(mistingSchedulePauseTime * 1000));
*/
         state_time_left = 100 * (1 - (float)state_run_time / (mistingSchedulePauseTime * 1000));
         // state_time_left = 100 * (1 - (state_run_time / (mistingScheduleRunTime * 1000)));
         //       Serial.println(state_time_left);

         //state_time_left = 100 * (1 - (state_run_time / (mistingSchedulePauseTime * 1000)));
      }
   }
}

void managePressure()
{

   if (pressurize && curr_pressure >= maxPressure)
   {
      Serial.println("Pressurizing done");
      pressurize = false;
      setRelayStateValue(PUMP_INDEX, RELAY_OFF);
   }
   else if (pressurize)
   {
      //make sure pump is on
      if (relayStates[PUMP_INDEX] == RELAY_OFF)
      {
         Serial.println("Starting pump for pressurizing");
         setRelayStateValue(PUMP_INDEX, RELAY_ON);
      }
   }
   else if (relayStates[PUMP_INDEX] == RELAY_ON && curr_pressure >= maxPressure)
   {
      setRelayStateValue(PUMP_INDEX, RELAY_OFF);
      Serial.print("ERR: Pressure at max, shutting of pump");
   }
}

void manageMockedPressure()
{

   if (millis() - mockedPressureTimer > 500)
   {
      if (relayStates[MISTING_INDEX] == RELAY_ON)
      {
         if (curr_pressure >= 2)
         {
            curr_pressure -= 2;
         }
         else
         {
            curr_pressure = 0;
         }
      }

      if (relayStates[BYPASS_INDEX] == RELAY_ON)
      {
         if (curr_pressure >= 29)
         {
            curr_pressure -= 29;
         }
         else
         {
            curr_pressure = 0;
         }
      }
      if (relayStates[PUMP_INDEX] == RELAY_ON)
      {
         curr_pressure += 5;
      }

      mockedPressureTimer = millis();
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
   Serial.print("[[");
   //Temperature & Humidity
   float hum = dht.readHumidity();
   float air_temp = dht.readTemperature();
   Serial.print("AT:");
   Serial.print(air_temp);
   Serial.print(";HUM:");
   Serial.print(hum);
   Serial.print(";");

   //Water temp
   Serial.print("WT:");
   Serial.print(";");

   //Pump State
   Serial.print("PMP:");
   Serial.print(!relayStates[PUMP_INDEX]);
   Serial.print(";");

   //Valve state
   Serial.print("MST:");
   Serial.print(!relayStates[MISTING_INDEX]);
   Serial.print(";");

   //Bypass State
   Serial.print("BP:");
   Serial.print(!relayStates[BYPASS_INDEX]);
   Serial.print(";");

   //System pressure
   Serial.print("PR:");
   Serial.print(curr_pressure);
   Serial.print(";");

   //Water level
   Serial.print("WLP:");
   Serial.print(waterLevelPercentage);
   Serial.print(";");
   Serial.print("WLD:");
   Serial.print(waterLevelDistance);
   Serial.print(";");

   //State
   Serial.print("ST:");
   Serial.print(state);
   Serial.print(";SL:");
   Serial.print(state_time_left);
   Serial.print(";SIM:");
   Serial.print(simulation);

   Serial.println("]]");

   sensorOutputTimer = millis();
}

void checkButtonPin(int idx)
{
   debouncers[idx].update(); // Update the Bounce instance
   if (debouncers[idx].fell())
   {
      relayStates[idx] = !relayStates[idx];
      digitalWrite(relayPins[idx], relayStates[idx]);
   }
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

      // SPx - S(et) P(ump) , Set pump state to x (0 or 1)
      // SMx - S(et) V(alve) , Set misting valve state to x (0 or 1)
      // SBx - S(et) B(ypass) , Set bypass valve state to x (0 or 1)
      // SSx - S(et) S(imulation) - 0 or 1

      // CWFxx - C(onfig) W(ater) F(ull), Set water level sensor full distance in cm
      // CWExx - C(onfig) W(ater) E(mpty), Set water level sensor empty disance in cm

      // CSRxxxx - C(onfig) S(chedule) R(untime), Set schedule run time (misting) in seconds
      // CSPxxxx - C(onfig> S(chedule) P(ause time). Set schedule pause time in seconds

      // CPHxxx - C(onfig) P(ressure) H(igh). Set max pressure
      // CPLxxx - C(onfig) P(ressure) L(ow). Set min pressure

      // RG - R(un command) G(o) - Start schedule, go to PRIMING state
      // RS - R(un command) S(top) - Abort schedule, go to IDLE state
      // RM - R(un command) M(ix) - Mix nutrients

      //-----------------------------------------------
      if (receivedChars[0] == 'S') // Set value on or off
      {
         //char state_object = receivedChars[1];
         uint8_t state_object_value = receivedChars[2] - '0';
         //Serial.print("Setting ");
         //Serial.print(state_object);
         //Serial.print(" to ");
         //Serial.println(state_object_value);

         if (receivedChars[1] == 'P' && (state_object_value == 1 || state_object_value == 0)) // Pump command
         {
            setRelayStateValue(PUMP_INDEX, state_object_value == LOGICAL_ON ? RELAY_ON : RELAY_OFF);
            printSensorData();

            Serial.print("Setting pump to: ");
            Serial.println(state_object_value);
         }
         if (receivedChars[1] == 'M' && (state_object_value == 1 || state_object_value == 0)) // Valve command
         {
            setRelayStateValue(MISTING_INDEX, state_object_value == LOGICAL_ON ? RELAY_ON : RELAY_OFF);
            printSensorData();

            Serial.print("Setting misting valve to: ");
            Serial.println(state_object_value);
         }
         if (receivedChars[1] == 'B' && (state_object_value == 1 || state_object_value == 0)) // Bypass command
         {
            setRelayStateValue(BYPASS_INDEX, state_object_value == LOGICAL_ON ? RELAY_ON : RELAY_OFF);
            printSensorData();
            Serial.print("Setting bypass valve to: ");
            Serial.println(state_object_value);
         }
         else if (receivedChars[1] == 'S')
         {
            simulation = state_object_value;
            Serial.print("Setting simulation to: ");
            Serial.println(state_object_value);

            if (simulation)
            {
               mockedPressureTimer = millis();
            }
         }
      }
      else if (receivedChars[0] == 'C') // Config value command
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
         else if (receivedChars[1] == 'P') // PRESSURE CONFIG
         {
            int hundratal = receivedChars[3] - '0';
            int tiotal = receivedChars[4] - '0';
            int ental = receivedChars[5] - '0';
            int newValue = hundratal * 100 + tiotal * 10 + ental;
            if (receivedChars[2] == 'H')
            {
               Serial.print("Setting max pressure to: ");
               Serial.println(newValue);
               maxPressure = newValue;
               printSensorData();
            }
            else if (receivedChars[2] == 'L')
            {
               Serial.print("Setting min pressure to: ");
               Serial.println(newValue);
               minPressure = newValue;
               printSensorData();
            }
         }
         else if (receivedChars[1] == 'S') // SCHEDULE CONFIG
         {
            int tusental = receivedChars[3] - '0';
            int hundratal = receivedChars[4] - '0';
            int tiotal = receivedChars[5] - '0';
            int ental = receivedChars[6] - '0';
            int newValue = tusental * 1000 + hundratal * 100 + tiotal * 10 + ental;
            if (receivedChars[2] == 'R')
            {
               Serial.print("Setting schedule run time to: ");
               Serial.println(newValue);
               mistingScheduleRunTime = newValue;
               printSensorData();
            }
            else if (receivedChars[2] == 'P')
            {
               Serial.print("Setting schedule pause time to: ");
               Serial.println(newValue);
               mistingSchedulePauseTime = newValue;
               printSensorData();
            }
         }
      }
      else if (receivedChars[0] == 'R') // Run command
      {

         if (receivedChars[1] == 'G')
         {
            if (state == STATE_IDLE)
            {
               state = STATE_PRIMING;
               pressurize = true;
               printSensorData();
            }
            else
            {
               Serial.println("ERR: Can only start from idle state");
            }
         }
         else if (receivedChars[1] == 'S')
         {
            state = STATE_IDLE;
            pressurize = false;
            setRelayStateValue(PUMP_INDEX, RELAY_OFF);
            setRelayStateValue(MISTING_INDEX, RELAY_OFF);
            setRelayStateValue(BYPASS_INDEX, RELAY_OFF);
            printSensorData();
         }
         else if (receivedChars[1] == 'M')
         {
            if (state == STATE_IDLE)
            {
               state = STATE_MIXING;
               pressurize = false;
               setRelayStateValue(PUMP_INDEX, RELAY_ON);
               setRelayStateValue(MISTING_INDEX, RELAY_OFF);
               setRelayStateValue(BYPASS_INDEX, RELAY_ON);
               printSensorData();
            }
            else
            {
               Serial.println("ERR: Can only start mixing from idle state");
            }
         }
      }

      newCommand = false;
   }
}
void setRelayStateValue(int idx, bool value)
{

   relayStates[idx] = value;
   if (!simulation)
   {
      digitalWrite(relayPins[idx], relayStates[idx]);
   }
}
