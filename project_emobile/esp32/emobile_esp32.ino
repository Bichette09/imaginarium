#include <Wire.h>
#include <VL53L1X.h>
int     freq = 5000;
int     ThrotleChannel = 0;
int     SteeringChannel = 1;
String  VL53L1XStatus = "0";
int     resolution = 16;
const byte        ESinterruptPin = 23;            
const byte        OddometerPin = 19;
const unsigned long        SpeedRefreshPeriod_us = 100000; // 10Hz
const float       meterpertck = 0.01335176877; // 2 x pi x 0,034 / 16
#define THROTTLE_PIN  13
#define STEERING_PIN 12 
VL53L1X sensor;
bool  lidarisOK = false;
volatile bool     EnablePowertrain = false;
volatile uint64_t ESStartFall = 0;                     // First interrupt value
volatile uint64_t ESStartRise = 0;                     // First interrupt value
unsigned long     tck_cpt = 0;
unsigned long     LastUpdateSpeedStamp = 0;
unsigned long     Lasttck_cpt = 0;

void IRAM_ATTR EShandleInterrupt() 
{
unsigned int Timestamp = micros();
      if (digitalRead(ESinterruptPin) == HIGH){
        if ((ESStartFall > ESStartRise) && (ESStartFall < Timestamp)){
          EnablePowertrain = (ESStartFall - ESStartRise) > 1500;
          
        }
        ESStartRise = Timestamp;
      }
      else {
        ESStartFall = Timestamp; 
      }
}

void IRAM_ATTR OddometerInterrupt() 
{
++tck_cpt; // Compteur d'impulsion roue codeuse
}

void setup() {
  // put your setup code here, to run once:
  pinMode(ESinterruptPin, INPUT_PULLUP);                                            // sets pin high
  pinMode(OddometerPin, INPUT_PULLUP); 
  pinMode(13, OUTPUT); 
  // sets pin high
  attachInterrupt(digitalPinToInterrupt(ESinterruptPin), EShandleInterrupt, CHANGE); // attaches pin to interrupt on Falling Edge
  attachInterrupt(digitalPinToInterrupt(OddometerPin), OddometerInterrupt, RISING); // attaches pin to interrupt on Falling Edge
  /* Initialise le port série */
  Serial.begin(115200);
  //Serial.begin(2000000);
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L1x sensor!");
  }
  else{
   lidarisOK = true; 
    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
  
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor.startContinuous(50);
  }
  
  //Setup PWM
  ledcSetup(ThrotleChannel, freq, resolution);
  ledcSetup(SteeringChannel, freq, resolution);

  ledcAttachPin(THROTTLE_PIN, ThrotleChannel);
  ledcAttachPin(STEERING_PIN, SteeringChannel);

}


void loop() {
  // put your main code here, to run repeatedly:;
//  Serial.println(EnablePowertrain);
     
  if (EnablePowertrain == false) {
       ledcWrite(ThrotleChannel,1540); //Neutral speed
       digitalWrite(13,HIGH); // Drive led on pin    
  }
  else{
      digitalWrite(13,LOW); // Drive led on pin     
  }

  if (Serial.available() > 0) {

    //Expected format message "[T|S][0-9]+/n"
    String incomingString = Serial.readStringUntil('\n');
    //  Serial.println(incomingString.c_str());
    String command = incomingString.substring(0, 1);
    if (command == "T" && EnablePowertrain ){
      ledcWrite(ThrotleChannel, incomingString.substring(1).toInt());
      //Serial.println(incomingString);
      //Serial.println("T");
    }
    else if (command == "S") {
      ledcWrite(SteeringChannel, incomingString.substring(1).toInt());
      //Serial.println(incomingString);
      //Serial.println("S");
    }
  }
    const unsigned long TimeStamp = micros();
    const unsigned long DeltaTime = TimeStamp - LastUpdateSpeedStamp;
    
    if (DeltaTime > SpeedRefreshPeriod_us) {
      const unsigned long Deltatck_cpt = (tck_cpt - Lasttck_cpt);
      LastUpdateSpeedStamp = TimeStamp;
      Lasttck_cpt = tck_cpt;
      const float Oddometry = meterpertck * tck_cpt; // in meter
      Serial.print("O");
      Serial.print(Oddometry); // in meter
      Serial.println();

      //Speed in m/s
      const float Speed = (Deltatck_cpt * meterpertck)  / (DeltaTime *0.000001); //DeltaTime is in microseconds
      Serial.print("S");
      Serial.print(Speed);
      Serial.println();
    }  
    if (lidarisOK){
      sensor.read();
      VL53L1XStatus = VL53L1X::rangeStatusToString(sensor.ranging_data.range_status);
      if (VL53L1XStatus == "range valid") {
        Serial.print("L");
        Serial.print(sensor.ranging_data.range_mm);
        Serial.println();
      }
    }
    //else {
    //  Serial.print(VL53L1XStatus);
    //  Serial.println();
    //}
  
}
