#include <Wire.h>
#include <Servo.h>
#include <Adafruit_VL53L0X.h>  

//for servos
#define SERVO_1 2
#define SERVO_2 4
#define SERVO_3 5

#define SERVO_1_UpLim 45
#define SERVO_2_UpLim 45
#define SERVO_3_UpLim 45

#define init_angle 0

Servo Servo_1;
Servo Servo_2;
Servo Servo_3;

int s1_ang = init_angle;
int s2_ang = init_angle;
int s3_ang = init_angle;

//for tof sensors
Adafruit_VL53L0X tof_1;
Adafruit_VL53L0X tof_2;
Adafruit_VL53L0X tof_3;

typedef struct {

  Adafruit_VL53L0X *psensor; // pointer to object
  TwoWire *pwire;
  int id;                                                     // I2C id number for the sensor
  int shutdown_pin;                                           // which pin for shutdown;
  int interrupt_pin;                                          // which pin to use for interrupts.
  Adafruit_VL53L0X::VL53L0X_Sense_config_t sensor_config;     // options for how to use the sensor
  uint16_t range;                                             // range value used in continuous mode stuff.
  uint8_t sensor_status;                                      // status from last ranging in continuous.
  
} tofObj;


tofObj tofArr[] = {

  {&tof_1, &Wire, 0x30, 4, 5, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&tof_2, &Wire, 0x31, 6, 7, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0},
  {&tof_3, &Wire, 0x32, 8, 9, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT, 0, 0}
  
};

const int COUNT_TOFS = sizeof(tofArr) / sizeof(tofArr[0]);

uint16_t dists[COUNT_TOFS];

void init_tofSensors() {
  
  bool sensorFound = false;
  
  // Set all shutdown pins low to shutdown sensors
  for (int i = 0; i < COUNT_TOFS; i++) {
    
    digitalWrite(tofArr[i].shutdown_pin, LOW);
    delay(10); //buffer
    
  }

  for (int i = 0; i < COUNT_TOFS; i++) {
    
    digitalWrite(tofArr[i].shutdown_pin, HIGH);
    delay(10); //buffer
    
    if (tofArr[i].psensor -> begin(tofArr[i].id, false, tofArr[i].pwire, tofArr[i].sensor_config)) {
      
      sensorFound = true;
      
    } else {

      // for debug purpose
      //Serial.print(i, DEC);
      //Serial.print(" have no sensor.");
      //Serial.println("");
      
    }
    
  }
  
  if (!sensorFound) {
    
    Serial.println("No valid sensors found");
    while (1) {};
    
  }
  
}

//for control system's target
uint16_t distInMm = 96;
uint16_t errInMm[COUNT_TOFS];
uint16_t tmpSumDist;
uint8_t aveCount = 50;
bool r1_aligned;
bool r2_aligned;
bool r3_aligned;
bool aligned;

void checkAlignment(int index) {

  if (index >= 0 && index <= 2) {

    tmpSumDist = 0;
  
    //average to get consistent result.
    for (int j = 0; j < aveCount; j++) {
  
      tmpSumDist += tofArr[index].psensor->readRange();
      delay(20);
        
    }
    dists[index] = tmpSumDist/aveCount;
    errInMm[index] = distInMm - dists[index];
    
  }
  else {

    for (int i = 0; i < COUNT_TOFS; i++) {
  
      tmpSumDist = 0;
  
      //average to get consistent result.
      for (int j = 0; j < aveCount; j++) {
  
        tmpSumDist += tofArr[i].psensor->readRange();
        delay(20);
        
      }
      dists[i] = tmpSumDist/aveCount;
      errInMm[i] = distInMm - dists[i];
        
    }
      
  }

  r1_aligned = (errInMm[0] < 0.5 && errInMm[0] > -0.5);
  r2_aligned = (errInMm[1] < 0.5 && errInMm[1] > -0.5);
  r3_aligned = (errInMm[2] < 0.5 && errInMm[2] > -0.5);
  aligned = ((r1_aligned && r2_aligned) && r3_aligned);
  
}


// for dev mode, uncomment to use com terminal, comment to save bytes.
//String command = "";

void setup() {
  // Enable Serial Comm
  Serial.begin(9600);
  while (!Serial && (millis() < 3000)) {};
  Serial.setTimeout(1000);

  // Enable I2C
  Wire.begin();

  // Init Servos
  Servo_1.attach(SERVO_1);
  Servo_2.attach(SERVO_2);
  Servo_3.attach(SERVO_3);

  Servo_1.write(s1_ang);
  Servo_2.write(s2_ang);
  Servo_3.write(s3_ang);

  // Init ToF Sensors
  for (int i = 0; i < COUNT_TOFS; i++) {
    
    pinMode(tofArr[i].shutdown_pin, OUTPUT);
    digitalWrite(tofArr[i].shutdown_pin, LOW);

    if (tofArr[i].interrupt_pin >= 0) {
      
      pinMode(tofArr[i].interrupt_pin, INPUT_PULLUP);
      
    }
      
  }
  init_tofSensors();

}

void loop() {
  checkAlignment(3);

  //control system
  while (!aligned) {

    Serial.println("Bed not leveled, leveling sequence invoked. \n");
    Serial.println("Current Servos Nominated Angle: ");
    Serial.print("Servo 1: ");
    Serial.println(s1_ang);
    Serial.print("Servo 2: ");
    Serial.println(s2_ang);\
    Serial.print("Servo 3: ");
    Serial.println(s3_ang);
    Serial.println("");
    delay(2000);

    Serial.println("Leveling R1.");

    while (!r1_aligned) {

      Serial.println(".");

      if (s1_ang < SERVO_1_UpLim) {
        
        s1_ang += 1;
        Servo_1.write(s1_ang);

        //for debug purpose
        Serial.print("Writing S1 Angle: ");
        Serial.println(s1_ang);
        
        delay(10);

      }
      else {

        //leveling halted, do something. prompt user to manual reset/fix
        while (1) {
          
          Serial.println("Leveling Halted, manual fix and reset required!");
          delay(2000);
          
        }
        
      }
      
      checkAlignment(0);
      
    }
    delay(800);

    Serial.println("Leveling R2.");

    while (!r2_aligned) {

      Serial.println(".");

      if (s2_ang < SERVO_2_UpLim) {
        
        s2_ang += 1;
        Servo_2.write(s2_ang);

        //for debug purpose
        Serial.print("Writing S2 Angle: ");
        Serial.println(s2_ang);
        
        delay(10);

      }
      else {

        //leveling halted, do something. prompt user to manual reset/fix
        while (1) {

          Serial.println("Leveling Halted, manual fix and reset required!");
          delay(2000);
          
        }
        
      }
      
      checkAlignment(1);
      
    }
    delay(800);

    Serial.println("Leveling R3.");

    while (!r3_aligned) {

      Serial.println(".");

      if (s3_ang < SERVO_3_UpLim) {
        
        s3_ang += 1;
        Servo_3.write(s3_ang);

        //for debug purpose
        Serial.print("Writing S3 Angle: ");
        Serial.println(s3_ang);
        
        delay(10);

      }
      else {

        //leveling halted, do something. prompt user to manual reset/fix
        while (1) {

          Serial.println("Leveling Halted, manual fix and reset required!");
          delay(2000);
          
        }
        
      }
      
      checkAlignment(2);
      
    }
    delay(800);
    
  }
  
  // below codes are for dev mode, uncomment to use com terminal, comment to save bytes.
  /*
  
  for (int i = 0; i < COUNT_TOFS; i++) {
      
    Serial.print("Sensor #");
    Serial.print(i, DEC);
      
    Serial.print(" at IIC address 0x");
    Serial.print(sensors[i].id, HEX);
    Serial.print(": ");
      
    Serial.print(dists[i], DEC);
    Serial.print(" mm.      ");
    
  }
  Serial.println();
   
  if (Serial.available()) {

    command = Serial.readString();
    //Serial.print("String: ");
    //Serial.println(command);

    s1_ang = command.substring(0,3).toInt();
    s2_ang = command.substring(4,7).toInt();
    s3_ang = command.substring(8,11).toInt();

    Serial.println("\n New Servo Angle: ");
    Serial.print("Servo 1: ");
    Serial.println(s1_ang);
    Serial.print("Servo 2: ");
    Serial.println(s2_ang);\
    Serial.print("Servo 3: ");
    Serial.println(s3_ang);

    // clear the string for new command input
    command = ""; 

    Servo_1.write(s1_ang);
    delay(500);
    Servo_2.write(s2_ang);
    delay(500);
    Servo_3.write(s3_ang);
    delay(500); 
    */
    
}
