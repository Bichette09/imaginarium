/* Constantes pour les broches */
// LIDARS 
// CAPT 9
//const byte TRIGGER_PIN9 = 46; // Broche TRIGGER
const byte ECHO_PIN9 = 47;    // Broche ECHO
// CAPT 10
//const byte TRIGGER_PIN10 = 48; // Broche TRIGGER
const byte ECHO_PIN10 = 49;    // Broche ECHO 
// CAPT 2
//const byte TRIGGER_PIN2 = 32; // Broche TRIGGER
const byte ECHO_PIN2 = 33;    // Broche ECHO
// CAPT 1
//const byte TRIGGER_PIN1 = 34; // Broche TRIGGER
const byte ECHO_PIN1 = 35;    // Broche ECHO


// CAPT 8
const byte TRIGGER_PIN8 = 50; // Broche TRIGGER
const byte ECHO_PIN8 = 51;    // Broche ECHO
// CAPT 7
const byte TRIGGER_PIN7 = 52; // Broche TRIGGER
const byte ECHO_PIN7 = 53;    // Broche ECHO
// CAPT 6
const byte TRIGGER_PIN6 = 44; // Broche TRIGGER
const byte ECHO_PIN6 = 45;    // Broche ECHO
// CAPT 5
const byte TRIGGER_PIN5 = 42; // Broche TRIGGER
const byte ECHO_PIN5 = 43;    // Broche ECHO
// CAPT 4
const byte TRIGGER_PIN4 = 40; // Broche TRIGGER
const byte ECHO_PIN4 = 41;    // Broche ECHO
// CAPT 3
const byte TRIGGER_PIN3 = 36; // Broche TRIGGER
const byte ECHO_PIN3 = 37;    // Broche ECHO


// Constantes pour le timeout 162.5ms - 6.2Hz
const unsigned long MEASURE_TIMEOUT_4m0 = 25000UL; // 25.000ms = ~8.5m à 340m/s 2x
const unsigned long MEASURE_TIMEOUT_3m5 = 21875UL; // 21.875ms = ~7.4m à 340m/s
const unsigned long MEASURE_TIMEOUT_3m0 = 18750UL; // 18.750ms = ~6.4m à 340m/s 2x
const unsigned long MEASURE_TIMEOUT_2m5 = 15625UL; // 15.625ms = ~5.3m à 340m/s 2x
const unsigned long MEASURE_TIMEOUT_2m0 = 12500UL; // 12.500ms = ~4.3m à 340m/s 2x
const unsigned long MEASURE_TIMEOUT_1m5 = 9375UL;  // 09.375ms = ~3.2m à 340m/s 2x
const unsigned long MEASURE_TIMEOUT_1m0 = 6250UL;  // 06.250ms = ~2.1m à 340m/s
const unsigned long LIDAR_TIMEOUT       = 15000UL; // 15ms
/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;
const float LIGHT_SPEED = 171.5 / 1000;

void setup() {
  // put your setup code here, to run once:
/* Initialise le port série */
// Serial.begin(115200);
Serial.begin(2000000);
 
   
  /* Initialise les broches CAPT1*/
  pinMode(ECHO_PIN1, OUTPUT);
  digitalWrite(ECHO_PIN1, LOW); // La broche TRIGGER doit être à LOW au repos
  /* Initialise les broches CAPT2*/
  pinMode(ECHO_PIN2, OUTPUT);
  digitalWrite(ECHO_PIN2, LOW); // La broche TRIGGER doit être à LOW au repos
/* Initialise les broches CAPT9*/
  pinMode(ECHO_PIN9, OUTPUT);
  digitalWrite(ECHO_PIN9, LOW); // La broche TRIGGER doit être à LOW au repos
  /* Initialise les broches CAPT10*/
  pinMode(ECHO_PIN10, OUTPUT);
  digitalWrite(ECHO_PIN10, LOW); // La broche TRIGGER doit être à LOW au repos
  
  /* Initialise les broches CAPT3*/
  pinMode(TRIGGER_PIN3, OUTPUT);
  digitalWrite(TRIGGER_PIN3, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN3, INPUT);  
  /* Initialise les broches CAPT4*/
  pinMode(TRIGGER_PIN4, OUTPUT);
  digitalWrite(TRIGGER_PIN4, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN4, INPUT);
  /* Initialise les broches CAPT5*/
  pinMode(TRIGGER_PIN5, OUTPUT);
  digitalWrite(TRIGGER_PIN5, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN5, INPUT);
  /* Initialise les broches CAPT6*/
  pinMode(TRIGGER_PIN6, OUTPUT);
  digitalWrite(TRIGGER_PIN6, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN6, INPUT);
  /* Initialise les broches CAPT7*/
  pinMode(TRIGGER_PIN7, OUTPUT);
  digitalWrite(TRIGGER_PIN7, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN7, INPUT);
  /* Initialise les broches CAPT8*/
  pinMode(TRIGGER_PIN8, OUTPUT);
  digitalWrite(TRIGGER_PIN8, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN8, INPUT);
}

void loop() {
// put your main code here, to run repeatedly:
//1 2 3 4 5 6 7 8 9 10

// LIDARS
// CAPT1 : RANGE 2m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 5µs sur la broche ECHO */
  pinMode(ECHO_PIN1, OUTPUT);
  digitalWrite(ECHO_PIN1, LOW);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN1, HIGH);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN1, LOW);
/* 2. Passe la pin en entrée et Mesure la largeur du pulse (si il existe) */
  pinMode(ECHO_PIN1, INPUT);
  long measure1 = pulseIn(ECHO_PIN1, HIGH, LIDAR_TIMEOUT);

// CAPT2 : RANGE 2m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 5µs sur la broche ECHO */
  pinMode(ECHO_PIN2, OUTPUT);
  digitalWrite(ECHO_PIN2, LOW);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN2, HIGH);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN2, LOW);
/* 2. Passe la pin en entrée et Mesure la largeur du pulse (si il existe) */
  pinMode(ECHO_PIN2, INPUT);
  long measure2 = pulseIn(ECHO_PIN2, HIGH, LIDAR_TIMEOUT);

// CAPT9 :  RANGE 2m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 5µs sur la broche ECHO */
  pinMode(ECHO_PIN9, OUTPUT);
  digitalWrite(ECHO_PIN9, LOW);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN9, HIGH);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN9, LOW);
/* 2. Passe la pin en entrée et Mesure la largeur du pulse (si il existe) */
  pinMode(ECHO_PIN9, INPUT);
  long measure9 = pulseIn(ECHO_PIN9, HIGH, LIDAR_TIMEOUT);

  // CAPT10 : RANGE 2m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 5µs sur la broche ECHO */
  pinMode(ECHO_PIN10, OUTPUT);
  digitalWrite(ECHO_PIN10, LOW);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN10, HIGH);
  delayMicroseconds(5);
  digitalWrite(ECHO_PIN10, LOW);
/* 2. Passe la pin en entrée et Mesure la largeur du pulse (si il existe) */
  pinMode(ECHO_PIN10, INPUT);
  long measure10 = pulseIn(ECHO_PIN10, HIGH, LIDAR_TIMEOUT);

// ULTRASOUND
// CAPT3 : RANGE 1.5m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN3, LOW);
/* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure3 = pulseIn(ECHO_PIN3, HIGH, MEASURE_TIMEOUT_1m5);

// CAPT4 : RANGE 2m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN4, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN4, LOW);
/* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure4 = pulseIn(ECHO_PIN4, HIGH, MEASURE_TIMEOUT_2m0);

// CAPT5 : RANGE 2m5
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN5, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN5, LOW);
/* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure5 = pulseIn(ECHO_PIN5, HIGH, MEASURE_TIMEOUT_2m5);

// CAPT6 : RANGE 2m5
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN6, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN6, LOW);
/* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure6 = pulseIn(ECHO_PIN6, HIGH, MEASURE_TIMEOUT_2m5);

  // CAPT7 : RANGE 2m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN7, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN7, LOW);
/* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure7 = pulseIn(ECHO_PIN7, HIGH, MEASURE_TIMEOUT_2m0);
/* 3. Calcul la distance à partir du temps mesuré */

// CAPT8 : RANGE 1.5m
/* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN8, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN8, LOW);
/* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure8 = pulseIn(ECHO_PIN8, HIGH, MEASURE_TIMEOUT_1m5);

 //LIDARS
  float distance1_cm = (measure1 * LIGHT_SPEED) / 10;
  float distance2_cm = (measure2 * LIGHT_SPEED) / 10;
  float distance9_cm = (measure9 * LIGHT_SPEED) / 10;
  float distance10_cm = (measure10 * LIGHT_SPEED) / 10;

 //ULTRASOUND    
  float distance3_cm = (measure3 / 2.0 * SOUND_SPEED) / 10;
  float distance4_cm = (measure4 / 2.0 * SOUND_SPEED) / 10;
  float distance5_cm = (measure5 / 2.0 * SOUND_SPEED) / 10;
  float distance6_cm = (measure6 / 2.0 * SOUND_SPEED) / 10;
  float distance7_cm = (measure7 / 2.0 * SOUND_SPEED) / 10;
  float distance8_cm = (measure8 / 2.0 * SOUND_SPEED) / 10;
  /* Affiche les résultats en mm, cm et m */

Serial.print(F("["));   
Serial.print(distance1_cm, 2);
Serial.print(F(","));
Serial.print(distance2_cm, 2);
Serial.print(F(","));
Serial.print(distance3_cm, 2);
Serial.print(F(","));
Serial.print(distance4_cm, 2);
Serial.print(F(","));
Serial.print(distance5_cm, 2);
Serial.print(F(","));
Serial.print(distance6_cm, 2);
Serial.print(F(","));
Serial.print(distance7_cm, 2);
Serial.print(F(","));
Serial.print(distance8_cm, 2);
Serial.print(F(","));
Serial.print(distance9_cm, 2);
Serial.print(F(","));
Serial.print(distance10_cm, 2);
Serial.println(("]"));

/* Délai d'attente pour éviter d'afficher trop de résultats à la seconde */
//  delay(500);
}
