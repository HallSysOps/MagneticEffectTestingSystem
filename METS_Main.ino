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

float hardOffsetX = -6.97;
float hardOffsetY = 38.33;
float hardOffsetZ = -34.96;

float softIron[3][3] = {
  {1.001,  0.038, -0.021},
  {0.038,  0.991,  0.004},
  {-0.021, 0.001,  1.011}
};

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
void Calibration();
void Test();
void lcdDisplay(int mode, float args[]);
void cartesianToSpherical(double* r, double* theta, double* phi);
double angleDifference(double a, double b);
bool Alarm(bool mode);
void printStats(double r, double theta, double phi, double rAvg, double thetaAvg, double phiAvg, bool printHeader = false);

enum SystemState{
  WAITING,
  CALIBRATING,
  TESTING,
  ERROR,
  PYTHONAPP
};

SystemState state = WAITING;
bool prevCalButton = true;
bool prevTestButton = true;
unsigned long lastDisplayTime = 0;
unsigned long lastCompassUpdate = 0;
const unsigned long compassUpdateInterval = 1000; // ms

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1);
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
String input = "";

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
        state = PYTHONAPP;
        //lastDisplayTime = millis();
        //lcd.clear();
        //lcd.setCursor(0, 0);
        //lcd.print("Calibrate First");
        //Alarm(0);
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
      Calibration();  // Blocking call assumed
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Calibrated!");
      delay(1000);  // Optional pause to show success
      state = WAITING;
      break;

    case TESTING:
      Test();

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
    case PYTHONAPP:
      if(calPressed){
        state = WAITING;
      }
      if (millis() - lastCompassUpdate >= compassUpdateInterval) {
          lastCompassUpdate = millis();
          lcdDisplay(9, NULL);
        }
      sensors_event_t event;
      lis3mdl.getEvent(&event);
      Serial.print(event.magnetic.x); Serial.print(",");
      Serial.print(event.magnetic.y); Serial.print(",");
      Serial.println(event.magnetic.z);
      delay(10);
  }
}

void lcdDisplay(int mode, float* args) {
  lcd.clear();
  lcd.setCursor(0, 0);

  switch (mode){
    case 0:
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
      lcd.print("LIS3MDL not");
      lcd.setCursor(0, 1);
      lcd.print("found");
    break;

    case 2:
      lcd.print("Ready to");
      lcd.setCursor(0, 1);
      lcd.print("Calibrate");
    break;

    case 3:
      lcd.print("Calibrating");
    break;

    case 4:
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
      lcd.print("t0:");
      lcd.print(thetaAverage, 1);   // Always 1 decimal

      lcd.print(" p0:");
      lcd.print(phiAverage, 1);

      lcd.setCursor(0, 1);
      lcd.print("t1:");
      lcd.print(args[0], 1);

      lcd.print(" p1:");
      lcd.print(args[1], 1);
    break;

    case 8:
      lcd.print("Test Complete!");
    break;

    case 9:
      lcd.print("PYTHON APP MODE");

    default:
      lcd.print("Invalid mode");
    break;
  }
}

void printStats(double r, double theta, double phi, double rAvg, double thetaAvg, double phiAvg, bool printHeader = false){
  if(printHeader){
    Serial.println("(r), (rAvg), (r - rAvg), (theta), (thetaAvg), (theta - thetaAvg), (phi), (phiAvg), (phi - phiAvg)");
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
// Define these globally or as static inside the function
float xEMA = 0, yEMA = 0, zEMA = 0;  // Running smoothed values
float alpha = 0.01;  // Smoothing factor (adjust as needed)

void cartesianToSpherical(double* r, double* theta, double* phi){
  delay(10);
  sensors_event_t event;
  lis3mdl.getEvent(&event);

  // Apply hard iron correction
  float rawX = event.magnetic.x - hardOffsetX;
  float rawY = event.magnetic.y - hardOffsetY;
  float rawZ = event.magnetic.z - hardOffsetZ;

  // Apply soft iron matrix
  float xRawCorrected = softIron[0][0]*rawX + softIron[0][1]*rawY + softIron[0][2]*rawZ;
  float yRawCorrected = softIron[1][0]*rawX + softIron[1][1]*rawY + softIron[1][2]*rawZ;
  float zRawCorrected = softIron[2][0]*rawX + softIron[2][1]*rawY + softIron[2][2]*rawZ;

  if(xEMA == 0 || yEMA == 0 || zEMA == 0){
    xEMA = xRawCorrected;
    yEMA = yRawCorrected;
    zEMA = zRawCorrected;
  }

  // === Apply EMA filtering ===
  xEMA = alpha * xRawCorrected + (1 - alpha) * xEMA;
  yEMA = alpha * yRawCorrected + (1 - alpha) * yEMA;
  zEMA = alpha * zRawCorrected + (1 - alpha) * zEMA;

  // Use filtered values for heading calculation
  *r = sqrt(xEMA * xEMA + yEMA * yEMA + zEMA * zEMA);
  *theta = atan2(yEMA, xEMA) * RAD_TO_DEG;
  if (*theta < 0) *theta += 360;
  *phi = asin(zEMA / *r) * RAD_TO_DEG;
}

void Calibration(){
  double rSum = 0, thetaSum = 0, phiSum = 0;
  int loop = 100;
  double r, theta, phi, rAvg, thetaAvg, phiAvg;

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

    printStats(r, theta, phi, rAvg, thetaAvg, phiAvg);
    float args[] = {thetaAvg, phiAvg, rAvg};
    lcdDisplay(0, args);
  }
  thetaAverage = thetaAvg;
  phiAverage = phiAvg;
  rAverage = rAvg;
}

void Test(){
  int loop = 100;
  double rSum = 0, thetaSum = 0, phiSum = 0;
  double r, theta, phi;
  bool endFlag = false;

  printStats(0, 0, 0, 0, 0, 0, true);

  for(int i = 0; i < 10; i++){
    cartesianToSpherical(&r, &theta, &phi); // Warm device up, throw away potential garbage values
  }

  while(!endFlag){
    cartesianToSpherical(&r, &theta, &phi);

    printStats(r, theta, phi, rAverage, thetaAverage, phiAverage);
    float args[] = {theta, phi};
    lcdDisplay(7, args);

    double thetaDiff = angleDifference(theta, thetaAverage);
    double phiDiff = fabs(phi - phiAverage);

    if(thetaDiff >= 1 || phiDiff >= 1){
      endFlag = true;
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDB, LOW);
      digitalWrite(LEDG, HIGH);
      Alarm(1);
      lcdDisplay(8, nullptr);
      delay(2000);
      lcdDisplay(7, nullptr);
      break;
    }
  }
}
double angleDifference(double a, double b) {
  double diff = fmod(a - b + 540.0, 360.0) - 180.0;
  return fabs(diff);
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