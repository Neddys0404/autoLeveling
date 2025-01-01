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
#define TOF_0_ADDRESS 0x30
#define TOF_1_ADDRESS 0x31
#define TOF_2_ADDRESS 0x32

#define TOF_0_XSHT 4
#define TOF_1_XSHT 6
#define TOF_2_XSHT 8

#define COUNT_TOFS 3

Adafruit_VL53L0X tof_0 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_2 = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t tof_0_reading;
VL53L0X_RangingMeasurementData_t tof_1_reading;
VL53L0X_RangingMeasurementData_t tof_2_reading;

int dists[COUNT_TOFS];

void init_tofSensors() {

  // reset all tofs
  digitalWrite(TOF_0_XSHT, LOW);    
  digitalWrite(TOF_1_XSHT, LOW);
  digitalWrite(TOF_2_XSHT, LOW);
  delay(10);

  // enable all tofs
  digitalWrite(TOF_0_XSHT, HIGH);    
  digitalWrite(TOF_1_XSHT, HIGH);
  digitalWrite(TOF_2_XSHT, HIGH);
  delay(10);

  // configure all tofs one by one
  digitalWrite(TOF_0_XSHT, HIGH);
  digitalWrite(TOF_1_XSHT, LOW);
  digitalWrite(TOF_2_XSHT, LOW);

  if (!tof_0.begin(TOF_0_ADDRESS)) {

    while( 1) { Serial.println("Failed to boot first VL53L0X."); delay(500);};

  }
  delay(10);

  digitalWrite(TOF_1_XSHT, HIGH);
  digitalWrite(TOF_2_XSHT, LOW);
  
  if (!tof_1.begin(TOF_1_ADDRESS)) {

    while(1) { Serial.println("Failed to boot second VL53L0X."); delay(500);};

  }
  delay(10);

  digitalWrite(TOF_2_XSHT, HIGH);
  
  if (!tof_2.begin(TOF_2_ADDRESS)) {

    while(1) { Serial.println("Failed to boot third VL53L0X."); delay(500);};

  }
  
}

//read distance once for all tof
void read_distance() {

  //read first tof
  tof_0.rangingTest(&tof_0_reading, false); // pass in 'true' to get debug data printout!

  /*
  //for debug purpose
  Serial.print("ToF at address 0x");
  Serial.print(TOF_0_ADDRESS, HEX);
  Serial.print(": ");

  if (tof_0_reading.RangeStatus != 4) {

    Serial.print(tof_0_reading.RangeMilliMeter, DEC);
    Serial.print(" mm     ");

  }
  else {

    Serial.print("Out of Range");

  }
  */

  //read second tof
  tof_1.rangingTest(&tof_1_reading, false); // pass in 'true' to get debug data printout!

  /*
  //for debug purpose
  Serial.print("ToF at address 0x");
  Serial.print(TOF_1_ADDRESS, HEX);
  Serial.print(": ");

  if (tof_1_reading.RangeStatus != 4) {

    Serial.print(tof_1_reading.RangeMilliMeter, DEC);
    Serial.print(" mm     ");

  }
  else {

    Serial.print("Out of Range");

  }
  */

  //read third tof
  tof_2.rangingTest(&tof_2_reading, false); // pass in 'true' to get debug data printout!

  /*
  //for debug purpose
  Serial.print("ToF at address 0x");
  Serial.print(TOF_2_ADDRESS, HEX);
  Serial.print(": ");

  if (tof_2_reading.RangeStatus != 4) {

    Serial.print(tof_2_reading.RangeMilliMeter, DEC);
    Serial.print(" mm     ");

  }
  else {

    Serial.print("Out of Range");

  }

  Serial.println();
  */

}

//for control system's target
uint16_t distInMm = 69; //target, should be changed
uint16_t errInMm[COUNT_TOFS];
uint16_t tmpSumDist[COUNT_TOFS];
uint8_t aveCount = 50;
bool r1_aligned;
bool r2_aligned;
bool r3_aligned;
bool aligned;

void checkAlignment(bool isTest) {

  tmpSumDist[0] = 0;
  tmpSumDist[1] = 0;
  tmpSumDist[2] = 0;

  //average to get consistent result.
  for (int i = 0; i < aveCount; i++) {

    read_distance();
    delay(10);

    if (tof_0_reading.RangeStatus != 4) {

      tmpSumDist[0] += tof_0_reading.RangeMilliMeter;
      
    }
    else {
      /* Debug */
    }

    if (tof_1_reading.RangeStatus != 4) {

      tmpSumDist[1] += tof_1_reading.RangeMilliMeter;
      
    }
    else {
      /* Debug */
    }

    if (tof_2_reading.RangeStatus != 4) {

      tmpSumDist[2] += tof_2_reading.RangeMilliMeter;
      
    }
    else {
      /* Debug */
    }
        
  }

  for (int i = 0; i < COUNT_TOFS; i++) {

    dists[i] = tmpSumDist[i]/aveCount;
    errInMm[i] = distInMm - dists[i];

  }

  if (isTest) {

    Serial.print("Test measure invoked!");
    Serial.println();
    Serial.println();

    Serial.print("At address 0x");
    Serial.print(TOF_0_ADDRESS, HEX);
    Serial.print("Current Reading: ");
    Serial.print(dists[0], DEC);
    Serial.println();
    Serial.print("Error in mm: ");
    Serial.print(errInMm[0], DEC);
    Serial.println();

    Serial.print("At address 0x");
    Serial.print(TOF_1_ADDRESS, HEX);
    Serial.print("Current Reading: ");
    Serial.print(dists[1], DEC);
    Serial.println();
    Serial.print("Error in mm: ");
    Serial.print(errInMm[1], DEC);
    Serial.println();

    Serial.print("At address 0x");
    Serial.print(TOF_2_ADDRESS, HEX);
    Serial.print("Current Reading: ");
    Serial.print(dists[2], DEC);
    Serial.println();
    Serial.print("Error in mm: ");
    Serial.print(errInMm[2], DEC);
    Serial.println();

  }
  else {

    r1_aligned = (errInMm[0] < 0.5 && errInMm[0] > -0.5);
    r2_aligned = (errInMm[1] < 0.5 && errInMm[1] > -0.5);
    r3_aligned = (errInMm[2] < 0.5 && errInMm[2] > -0.5);
    aligned = ((r1_aligned && r2_aligned) && r3_aligned);

  }
  
}


// for dev mode, uncomment to use com terminal, comment to save bytes.
String command = "";

void setup() {
  // Enable Serial Comm
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.println("Serial succesfully opened.");
  delay(500);

  // Init Servos
  Servo_1.attach(SERVO_1);
  Servo_2.attach(SERVO_2);
  Servo_3.attach(SERVO_3);

  Servo_1.write(s1_ang);
  Servo_2.write(s2_ang);
  Servo_3.write(s3_ang);

  // Init ToF Sensors
  pinMode(TOF_0_XSHT, OUTPUT);
  pinMode(TOF_1_XSHT, OUTPUT);
  pinMode(TOF_2_XSHT, OUTPUT);
  digitalWrite(TOF_0_XSHT, LOW);
  digitalWrite(TOF_1_XSHT, LOW);
  digitalWrite(TOF_2_XSHT, LOW);

  init_tofSensors();

  Serial.println("Initilization Complete! Starting...");
  Serial.println();
  for (int i = 0; i < 10; i++) {

    Serial.println(".");
    delay(200);

  }

}

void loop() {
  //control system
  /*
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
  */

  // below codes are for dev mode, uncomment to use terminal.
  //periodically reads tof distance
   
  checkAlignment(true);
  delay(1000);

}
