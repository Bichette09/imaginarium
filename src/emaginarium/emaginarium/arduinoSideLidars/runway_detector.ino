
#include <Wire.h>
#include <VL53L1X.h>

int L1SHUTDOWN = 2;
int L2SHUTDOWN = 3;
int L3SHUTDOWN = 4;
int L4SHUTDOWN = 5;
VL53L1X L1;
VL53L1X L2;
VL53L1X L3;
VL53L1X L4;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  //Serial.println("Sensor init");
  delay(100);
  pinMode(L1SHUTDOWN, OUTPUT);
  pinMode(L2SHUTDOWN, OUTPUT);
  pinMode(L3SHUTDOWN, OUTPUT);
  pinMode(L4SHUTDOWN, OUTPUT);
  digitalWrite(L1SHUTDOWN, LOW);
  digitalWrite(L2SHUTDOWN, LOW);
  digitalWrite(L3SHUTDOWN, LOW);
  digitalWrite(L4SHUTDOWN, LOW);
  //Serial.println("Lidars set address");
  
  digitalWrite(L1SHUTDOWN, HIGH);
  L1.init();
  L1.setAddress(1);
  L1.setDistanceMode(VL53L1X::Short);
  L1.setMeasurementTimingBudget(20000);
  L1.startContinuous(20);
  delay(100);
  digitalWrite(L2SHUTDOWN, HIGH);
  L2.init();
  L2.setAddress(2);
  L2.setDistanceMode(VL53L1X::Short);
  L2.setMeasurementTimingBudget(20000);
  L2.startContinuous(20);
  delay(100);
  digitalWrite(L3SHUTDOWN, HIGH);
  L3.init();
  L3.setAddress(3);
  L3.setDistanceMode(VL53L1X::Short);
  L3.setMeasurementTimingBudget(20000);
  L3.startContinuous(20);
  delay(100);
  digitalWrite(L4SHUTDOWN, HIGH);
  L4.init();
  L4.setAddress(4);
  L4.setDistanceMode(VL53L1X::Short);
  L4.setMeasurementTimingBudget(20000);
  L4.startContinuous(20);
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    char c = (char)Serial.read();
    //Serial.println(c);
    if(c=='A'){
      L1.read();
      L2.read();
      L3.read();
      L4.read();
      Serial.print("{\"L1\":{\"D\":");
      Serial.print(L1.ranging_data.range_mm);
      Serial.print(",\"S\":");
      Serial.print(L1.ranging_data.range_status);
      Serial.print("},\"L2\":{\"D\":");
      Serial.print(L2.ranging_data.range_mm);
      Serial.print(",\"S\":");
      Serial.print(L2.ranging_data.range_status);
      Serial.print("},\"L3\":{\"D\":");
      Serial.print(L3.ranging_data.range_mm);
      Serial.print(",\"S\":");
      Serial.print(L3.ranging_data.range_status);
      Serial.print("},\"L4\":{\"D\":");
      Serial.print(L4.ranging_data.range_mm);
      Serial.print(",\"S\":");
      Serial.print(L4.ranging_data.range_status);
      Serial.print("}}\n");
    }
  }  
  delay(1);
}
