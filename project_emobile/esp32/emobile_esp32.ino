#include <Wire.h>
#include <VL53L1X.h>
int     freq = 50;
int     ThrotleChannel = 0;
int     SteeringChannel = 1;
String  VL53L1XStatus = "0";
int     resolution = 16;
const byte                  ESinterruptPin = 23;            
const byte                  OddometerPin = 19;
const unsigned long         SpeedRefreshPeriod_us = 100000; // 10Hz
const float                 meterpertck = 0.02591813939; // 2 x pi x r / 8 // (2 x 3,14 x 3,1459 )/8
#define THROTTLE_PIN  33
#define STEERING_PIN 32
VL53L1X sensor;
bool  lidarisOK = false;
volatile bool     EnablePowertrain = false;
volatile uint64_t ESStartFall = 0;                     // First interrupt value
volatile uint64_t ESStartRise = 0;                     // First interrupt value
unsigned long     tck_cpt = 0;
unsigned long     LastUpdateSpeedStamp = 0;
unsigned long     Lasttck_cpt = 0;
float             Oddometry =0;
float             Speed=0;
int               LidarDist;

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
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(STEERING_PIN, OUTPUT);
  digitalWrite (STEERING_PIN , LOW);
  digitalWrite (THROTTLE_PIN, LOW);

  
  //pinMode(13, OUTPUT); //pin for LED
  // sets pin high
  attachInterrupt(digitalPinToInterrupt(ESinterruptPin), EShandleInterrupt, CHANGE); // attaches pin to interrupt on Falling Edge
  attachInterrupt(digitalPinToInterrupt(OddometerPin), OddometerInterrupt, RISING); // attaches pin to interrupt on Falling Edge
  /* Initialise le port série */
  Serial.begin(115200);
  Serial2.begin(115200);
  //Serial.begin(2000000);
  Wire.begin();
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L1x sensor!");
    Serial2.println("Failed to detect and initialize VL53L1x sensor!");
  }
  else{
   lidarisOK = true; 
    // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
    // You can change these settings to adjust the performance of the sensor, but
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    // medium and long distance modes. See the VL53L1X datasheet for more
    // information on range and timing limits.
    Serial.println("VL53L1x sensor init successful!");
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(20000);
  
    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor.startContinuous(20);
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
       ledcWrite(ThrotleChannel,4915); //Neutral speed
       //digitalWrite(13,HIGH); // Drive led on pin    
  }
  else{
      //digitalWrite(13,LOW); // Drive led on pin     
  }

  if (Serial2.available() > 0) {

    //Expected format message "[T|S][0-9]+/n"
    String incomingString = Serial2.readStringUntil('\n');
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
    else if (command == "R") {
      Serial2.print("O");
      Serial2.print(Oddometry); // in meter
      Serial2.println();//Serial.println(incomingString);
      
      Serial2.print("S");
      Serial2.print(Speed); //in m/s
      Serial2.println();

      Serial2.print("L");
      Serial2.print(LidarDist); //in mm
      Serial2.println();

      Serial.print(LidarDist); //in mm
      Serial.println();
    }
    
  }
    const unsigned long TimeStamp = micros();
    const unsigned long DeltaTime = TimeStamp - LastUpdateSpeedStamp;
    
    if (DeltaTime > SpeedRefreshPeriod_us) {
      const unsigned long Deltatck_cpt = (tck_cpt - Lasttck_cpt);
      LastUpdateSpeedStamp = TimeStamp;
      Lasttck_cpt = tck_cpt;
      Oddometry = meterpertck * tck_cpt; // in meter
      

      //Speed in m/s
      Speed = (Deltatck_cpt * meterpertck)  / (DeltaTime *0.000001); //DeltaTime is in microseconds
      
    }  
    if (lidarisOK){
      sensor.read(
        3);
      VL53L1XStatus = VL53L1X::rangeStatusToString(sensor.ranging_data.range_status);
      if (VL53L1XStatus == "range valid") {

        LidarDist=sensor.ranging_data.range_mm; 
      }
    }
    //else {
    //  Serial.print(VL53L1XStatus);
    //  Serial.println();
    //}
  
}
