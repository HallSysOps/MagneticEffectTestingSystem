//Libraries
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>

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
float Average = -1;

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

void setup() {
  Serial.begin(9600);

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
  if (!lis3mdl.begin_I2C(0x1E) || !lis3mdl.begin_I2C(0x1C)){
    //Serial.println("Failed to find LIS3MDL chip");
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, HIGH);
    lcd.setCursor(0, 0);
    lcd.print("LIS3MDL not");
    lcd.setCursor(0, 1);
    lcd.print("found");
    //while (1) { delay(10); }
    }
   //Initialization Options
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
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
  lcd.print("Ready to callibrate.");
}
void loop() {

  //Read the buttons
  CalibrationState = digitalRead(CalibrationButton);
  TestState = digitalRead(TestButton);

  if(!CalibrationState){
    //start callibration and get average reading
    Average = Calibration();
  }
  //Extra qualifier of Average being non-zero so you have to go into the calibration state before test state
  if(Average > 0 && !TestState) {

    //Update LED and LCD to indicate the Testing Phase
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, HIGH);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Testing");
    
    //Serial.println("Entered Test phase.");
    TestFailure = Test(Average);
    //check if testing failed
    if(TestFailure) {
      lcd.setCursor(0,1);
      lcd.print("Failed");
      digitalWrite(LEDR, LOW);
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDG, HIGH);
    }
    else {
      //if testing worked properly
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Ready to test.");

      digitalWrite(LEDR, HIGH);
      digitalWrite(LEDB, HIGH);
      digitalWrite(LEDG, LOW);
    }
  }

  else if (!TestState) { //Check to see if the button is being pressed before it should be
    analogWrite(AnalogVoltageOut, SVoltage);
    //An Error message if the button is pressed before the average is calculated
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Calibrate First");
    Alarm(0);
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);
    digitalWrite(LEDG, HIGH);
    //delay so the message appears for ample amount of time to be read
    delay(5000);
    //Resume normal message waiting for instructions
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ready to calibrate.");
    
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDG, LOW);

  }
}
float Heading(){
  lis3mdl.read();
  float heading = 0;
  if(lis3mdl.x == 0 && lis3mdl.y == 0){
    exit;
  }
  else{
    heading = -((atan2(lis3mdl.y,lis3mdl.x) * 180) / pi)+180;
  }
  //print readings to serial
  //Serial.print(heading); Serial.print(","); Serial.print(lis3mdl.x); Serial.print(","); Serial.print(lis3mdl.y); Serial.print(",");
  return heading;
}
float Calibration() {

  //initialize local variables
  float Sum = 0;
  int loop = 1000;
  float heading = 0;
  //Count the number of bad readings
  int BadReading = 0;
  Average = 0;

  //Implement LED Calibration colors.
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, HIGH);
  //Reset the Average everytime this state is entered
  
  //Indicate Calibration State on LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Calibrating");
  delay(2000);

  // loop is still being tested to see the optimal sample size to get a good average
  // loop still needs a way to throw away bad readings during calibration
  for(int i = 0; i < loop; ++i) {
    heading = Heading();
    //After the first loop if there is a reading that differs by more than one get rid of the reading
    if(i > 0 && (heading - (Sum/i) >= 2 || (Sum/i) - heading >= 2)) {
      //Keep track of the readings that are thrown away and throw away the reading
      ++BadReading;
      heading = 0;
    }
    Sum = Sum + heading;
    //Delay is needed to make sure the Magnetometer has enough time
    delay(10);
  }
  if(BadReading > loop/10) {
    //Notify that Calibration had too many outliers
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Calibration");
    lcd.setCursor(0,1);
    lcd.print("Failed");
    Alarm(1);
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDG, HIGH);
    return -1;
  }
  else {
    //Reset the LCD to show we have left calibration state
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ready");
      
    //Reset the LED to show we have left the calibration state
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, HIGH);
    digitalWrite(LEDG, LOW);
    return Sum / (loop - BadReading);
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