//Libraries
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

//Initialize i2C address and size of LCD screen
LiquidCrystal_I2C lcd(0x27, 16, 2); //sets I2C pin to 0x27 and tells it 2 rows by 16 columns

Adafruit_LIS3MDL lis3mdl;
//SPI Definitions
// #define LIS3MDL_CLK 13
// #define LIS3MDL_MISO 12
// #define LIS3MDL_MOSI 11
// #define LIS3MDL_CS 10

// pins and Global variables
const int CalibrationButton = 3;
const int TestButton = 2;
const int AnalogVoltageOut = A2;
const int SVoltage = 21; //Starting voltage is 1 V after the amp circuit
const int Pot = 7; // Potentiometer/Volume control
const int LEDR = 4;
const int LEDG = 5;
const int LEDB = 6;
//LED color legend
//Green is waiting for next phase 
//Blue is in a phase (setup or calibration or testing)
//Red is testing or calibration failed
//NOTE: LED is inverted meaning the color you want to show up is the color grounded.

const float pi = 3.1415926535897932384626433832795;
//float Average = -1;

float hardOffsetX = -12.99;
float hardOffsetY = 17.07;
float hardOffsetZ = -33.40;

float phiAverage = -1000;
float thetaAverage = -1000;
float rAverage = -1000;

float test_phiAverage = -1000;
float test_thetaAverage = -1000;
float test_rAverage = -1000;



//when the failure boolean is set to true that means there was an error in testing
//when the failure boolean is set to false that means everything is working properly
bool TestFailure = false;
//states
volatile bool CalibrationState = 0;
volatile bool TestState = 0;

//Functions (defined at the bottom)
float Calibration();
bool Test(float Avg);
float Heading();
bool Alarm(bool mode);
void printStats(double r, double theta, double phi, double rAvg, double thetaAvg, double phiAvg, bool printHeader = false);
void cartesianToSpherical(double* r, double* theta, double* phi);
void newCalibration();
void lcdDisplay(int mode, float args[]);
void newTest();

enum SystemState{
  WAITING,
  CALIBRATING,
  TESTING,
  ERROR
};

SystemState state = WAITING;
bool prevCalButton = true;
bool prevTestButton = true;
unsigned long lastDisplayTime = 0;
unsigned long lastCompassUpdate = 0;
const unsigned long compassUpdateInterval = 1000; // ms

void setup() {
  Serial.begin(9600);
  delay(100);
  //Initialize LED
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  //Indicate setup is happening
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, HIGH);

  // Initialize pins
  // NOTE: input pullup means the pins are inverted and read True when grounded by button press
  pinMode(CalibrationButton, INPUT_PULLUP);
  pinMode(TestButton, INPUT_PULLUP);
  pinMode(AnalogVoltageOut, OUTPUT);
  pinMode(Pot, OUTPUT);

  //Analog Voltage output (AVO) Legend
  //AVO range = 0V to 5V
  //AVO = 230 * Deflection + 25
  //AVO = 0 means 0V out
  //AVO = 255 means 5V out
  //The Output increases linearly from 0V to 5V, 51.2 per Volt
  analogWrite(AnalogVoltageOut, SVoltage);
  //Initialized to 25 for approx 0.5 Volts

  // Initialize LCD screen
  lcd.init();
  lcd.backlight();
  //Serial.println("LCD Initialized");

  //test if magnetometer is initalized
  if (!lis3mdl.begin_I2C(0x1E) && !lis3mdl.begin_I2C(0x1C)){
    //Serial.println("Failed to find LIS3MDL chip");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, HIGH);
    lcdDisplay(1, nullptr);
    //while (1) { delay(10); }
    }
   //Initialization Options
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  //Implement LED waiting colors and display
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDG, LOW);

  Alarm(0);
  //lcdDisplay(2, nullptr);
}

void loop() {
  // Read buttons (with INPUT_PULLUP logic: LOW = pressed)
  bool calPressed = !digitalRead(CalibrationButton);
  bool testPressed = !digitalRead(TestButton);

  // Detect button press events (LOW transition)
  bool calJustPressed = (calPressed && prevCalButton == false);
  bool testJustPressed = (testPressed && prevTestButton == false);

  prevCalButton = !calPressed;  // Save inverted logic
  prevTestButton = !testPressed;

  switch (state) {

    case WAITING:
      if (calPressed) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Calibrating...");
        state = CALIBRATING;

        // Reset Averages
        phiAverage = -1000;
        thetaAverage = -1000;
        rAverage = -1000;

        test_phiAverage = -1000;
        test_thetaAverage = -1000;
        test_rAverage = -1000;

      } else if (testPressed && thetaAverage > -1000) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Testing...");
        state = TESTING;
      } else if (testPressed && thetaAverage == -1000) {
        state = ERROR;
        lastDisplayTime = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Calibrate First");
        Alarm(0);
      } else if (test_thetaAverage > -1000){ // Show Test Results
        if (millis() - lastCompassUpdate >= compassUpdateInterval) {
          lastCompassUpdate = millis();
          lcdDisplay(7, nullptr);
        }
      } else if (thetaAverage > -1000){ // Calibration Complete, Show Averages Computed
        // Only update LCD every 1000 ms
        if (millis() - lastCompassUpdate >= compassUpdateInterval) {
          lastCompassUpdate = millis();
          lcdDisplay(6, nullptr);
        }
      } else{ // Show current data from magnetometer
        // Only update LCD every 1000 ms
        if (millis() - lastCompassUpdate >= compassUpdateInterval) {
          lastCompassUpdate = millis();

          double theta, phi, r;
          cartesianToSpherical(&r, &theta, &phi);
          float args[] = {theta, phi, r};
          lcdDisplay(5, args);
        }
      }
      break;

    case CALIBRATING:
      newCalibration();  // Blocking call assumed
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrated!");
      delay(1000);  // Optional pause to show success
      state = WAITING;
      break;

    case TESTING:
      //TestFailure = Test(Average);
      NewTest();

      /*
      if (TestFailure) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Test Failed");
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDB, HIGH);
        digitalWrite(LEDG, HIGH);
      } else {
        double theta, phi, r;
        cartesianToSpherical(&r, &theta, &phi);
        float args[] = {theta, phi, r};
        lcdDisplay(4, args);
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDB, HIGH);
        digitalWrite(LEDG, LOW);
      }
      */
      delay(1500);  // Briefly show results
      state = WAITING;
      break;

    case ERROR:
      if (millis() - lastDisplayTime > 3000) {
        // Show standby info after brief error message
        double theta, phi, r;
        cartesianToSpherical(&r, &theta, &phi);
        float args[] = {theta, phi, r};
        lcdDisplay(5, args);
        state = WAITING;
      }
      break;
  }
}

void lcdDisplay(int mode, float* args) {
  lcd.clear();

  switch (mode){
    case 0:
      lcd.setCursor(0, 0);
      lcd.print("Theta:");
      lcd.print(args[0], 1);
      lcd.print("  ");

      lcd.setCursor(0, 1);
      lcd.print("Phi:");
      lcd.print(args[1], 1);
      lcd.print("  R:");
      lcd.print(args[2], 0);
    break;

    case 1:
      lcd.setCursor(0, 0);
      lcd.print("LIS3MDL not");
      lcd.setCursor(0, 1);
      lcd.print("found");
    break;

    case 2:
      lcd.setCursor(0, 0);
      lcd.print("Ready to");
      lcd.setCursor(0, 1);
      lcd.print("Calibrate");
    break;

    case 3:
      lcd.setCursor(0, 0);
      lcd.print("Calibrating");
    break;

    case 4:
      lcd.setCursor(0,0);
      lcd.print("Ready to test.");

      lcd.setCursor(0, 1);
      lcd.print("t:");
      lcd.print(args[0], 0);  // theta

      lcd.print(" p:");
      lcd.print(args[1], 0);  // phi

      lcd.print(" r:");
      lcd.print(args[2], 0);  // r
    break;

    case 5:
      lcd.setCursor(0,0);
      lcd.print("Calibrate Ready");

      lcd.setCursor(0, 1);
      lcd.print("t:");
      lcd.print(args[0], 0);  // theta

      lcd.print(" p:");
      lcd.print(args[1], 0);  // phi

      lcd.print(" r:");
      lcd.print(args[2], 0);  // r
    break;

    case 6:
      lcd.setCursor(0,0);
      lcd.print("Test Ready");

      lcd.setCursor(0, 1);
      lcd.print("t:");
      lcd.print(thetaAverage, 0);  // theta

      lcd.print(" p:");
      lcd.print(phiAverage, 0);  // phi

      lcd.print(" r:");
      lcd.print(rAverage, 0);  // r
    break;

    case 7:
      lcd.setCursor(0, 0);
      lcd.print("t0:");
      lcd.print(thetaAverage, 1);   // Always 1 decimal

      lcd.print(" p0:");
      lcd.print(phiAverage, 1);

      lcd.setCursor(0, 1);
      lcd.print("t1:");
      lcd.print(test_thetaAverage, 1);

      lcd.print(" p1:");
      lcd.print(test_phiAverage, 1);
    break;

    case 8:
      lcd.setCursor(0, 0);
      lcd.print("Test Complete!");
    break;

    default:
      lcd.setCursor(0, 0);
      lcd.print("Invalid mode");
    break;
  }
}

void printStats(double r, double theta, double phi, double rAvg, double thetaAvg, double phiAvg, bool printHeader = false){
  if(printHeader){
    Serial.println("Strength of field (r), Average strength of field (rAvg), Difference in strength of field (r - rAvg), Theta (theta), Average theta (thetaAvg), Difference in theta (theta - thetaAvg), Phi (phi), Average phi (phiAvg), Difference in phi (phi - phiAvg)");
    return;
  }
  
  Serial.print(r);
  Serial.print(", ");
  Serial.print(rAvg);
  Serial.print(", ");
  Serial.print(abs(r - rAvg));
  Serial.print(", ");

  Serial.print(theta);
  Serial.print(", ");
  Serial.print(thetaAvg);
  Serial.print(", ");
  Serial.print(abs(theta - thetaAvg));
  Serial.print(", ");

  Serial.print(phi);
  Serial.print(", ");
  Serial.print(phiAvg);
  Serial.print(", ");
  Serial.println(abs(phi - phiAvg));
}

void cartesianToSpherical(double* r, double* theta, double* phi){

  sensors_event_t event;
  lis3mdl.getEvent(&event);
  //Serial.print(event.magnetic.x);
  //Serial.print("| ");
  //Serial.print(event.magnetic.y);
  //Serial.print("| ");
  //Serial.println(event.magnetic.z);

  float x = event.magnetic.x - hardOffsetX;
  float y = event.magnetic.y - hardOffsetY;
  float z = event.magnetic.z - hardOffsetZ;  

  *r = sqrt(x * x + y * y + z * z); // Strength of Magnetic Field
  *theta = atan2(y, x) * RAD_TO_DEG;
  if (*theta < 0) *theta += 360; // Normalize to 0-360
  *phi = asin(z / *r) * (180 / pi);
}

void newCalibration(){
  double rSum = 0, thetaSum = 0, phiSum = 0;
  int loop = 100;
  double r, theta, phi, rAvg, thetaAvg, phiAvg;

  int badReading = 0;

  //Implement LED Calibration colors.
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, HIGH);
  //Reset the Average everytime this state is entered
  
  //Indicate Calibration State on LCD
  lcdDisplay(3, nullptr);
  delay(2000);

  printStats(r, theta, phi, rAvg, thetaAvg, phiAvg, true);

  for(int i = 0; i < 10; i++){
    cartesianToSpherical(&r, &theta, &phi); // Warm device up, throw away potential garbage values
  }

  for(int i = 0; i < loop; i++){
    cartesianToSpherical(&r, &theta, &phi);

    rSum += r;
    thetaSum += theta;
    phiSum += phi;

    rAvg = rSum / (i + 1);
    thetaAvg = thetaSum / (i + 1);
    phiAvg = phiSum / (i + 1);

    //if(i > 0 && abs(heading - average) >= 2){
    //  // Bad Reading
    //  ++badReading;
    //}

    printStats(r, theta, phi, rAvg, thetaAvg, phiAvg);
    float args[] = {thetaAvg, phiAvg, rAvg};
    lcdDisplay(0, args);
    delay(100);
  }
  thetaAverage = thetaAvg;
  phiAverage = phiAvg;
  rAverage = rAvg;
}

void NewTest(){
  int loop = 100;
  double rSum = 0, thetaSum = 0, phiSum = 0;
  double r, theta, phi;

  printStats(0, 0, 0, 0, 0, 0, true);

  for(int i = 0; i < 10; i++){
    cartesianToSpherical(&r, &theta, &phi); // Warm device up, throw away potential garbage values
  }

  for(int i = 0; i < loop; i++){
    cartesianToSpherical(&r, &theta, &phi);

    rSum += r;
    thetaSum += theta;
    phiSum += phi;

    test_rAverage = rSum / (i + 1);
    test_thetaAverage = thetaSum / (i + 1);
    test_phiAverage = phiSum / (i + 1);

    printStats(r, theta, phi, test_rAverage, test_thetaAverage, test_phiAverage);
    //float args[] = {test_thetaAverage, test_phiAverage, test_rAverage};
    lcdDisplay(7, nullptr);

    if(abs(test_thetaAverage - thetaAverage) >= 1 || abs(test_phiAverage - phiAverage) >= 1){
      lcdDisplay(8, nullptr);
      delay(2000);
      lcdDisplay(7, nullptr);
      break;
    }
    delay(100);
  }
}

bool Test(float Avg) {
  float deflection = 1;
  float heading = 0;
  float highest = 0;
  float lowest = 0;
  float testAvg = 0;
  int loop = 10;
  bool endFlag = true;
  delay(2000);

  heading = Heading();
  lowest = heading;
  highest = heading;

  //loop until flag gets cleared
  while(endFlag) {
    testAvg = 0;

    for(int i = 0; i < loop; ++i) {
      heading = Heading();
      delay(10);
      //prints every 5th reading to 2 decimal numbers
      if(i % 5 == 0){
        lcd.setCursor(0, 1);
        lcd.print(heading, 2);
      }
    }  
    testAvg = testAvg / loop;

    if(testAvg > highest) {
      highest = testAvg;
    }
    else if (lowest > testAvg) {
      lowest = testAvg;
    }

    if(highest - Avg > Avg - lowest) {
      analogWrite(AnalogVoltageOut, 188*(highest - testAvg) + SVoltage);
    }
    else {
      analogWrite(AnalogVoltageOut, 188*(testAvg - lowest) + SVoltage);
    }

    //If heading and avg differ by more than 1 clear flag and exit
    if(highest - Avg >= deflection || Avg - lowest >= deflection) {
      endFlag = false;
    }
  }
    // Update LED and LCD to indicate the end of the Testing Phase
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, HIGH);
  Alarm(1);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Reading Complete");
  delay(8000);
    //Indicate if the testing was ended by an outlier
  if(highest - testAvg > 2 || testAvg - lowest > 2) {
    return true;
  }
  else {
    return false;
  }
}
bool Alarm(bool mode){
  //starts the alarm. if mode = 0, the alarm beeps 3 times.
  if(mode == 0){
    for(int i = 0; i < 3; i++){
      digitalWrite(Pot, HIGH);
      delay(500);
      digitalWrite(Pot, LOW);
    }
  }
  else{
    digitalWrite(Pot, HIGH);
    delay(4000);
    digitalWrite(Pot, LOW);
  }
}