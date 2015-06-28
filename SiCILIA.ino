/*


 Circuit:

 * 1 MLX90614 IR sensor attached
 * 2 Servos attached
 * 1 URF attached
 * 1 Grid-EYE

 created 08 Jan 2015
 by Ala Shaabana

 */


/*
* WiFi.h is NOT compatible with newping.h, likely because of timer functions being used by both libraries.2
* For now, we will disable wifi but LEAVE THE CODE IN THERE, we will tackle that problem down the line if we need to
* for the time being, all communication with Arduino is via Serial port, including Java communications.
*
*/
//#include <WiFi.h>
#include <TFT.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <NewPing.h>
#include <TrueRandom.h>

// pin definition for the TFT screen on Uno
#define cs   10
#define dc   9
#define rst  8

#define TRIGGER_PIN  2 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

// Motor turn time
#define TURN_TIME 40

// Motor variables
Servo xServo;
Servo yServo;

// IR sensor variables
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Grid-eye variables
byte pixelTempL;

char addr = 0x68;

float heatGrid[8][8];

int irqPin = 2;
int pwmPin = 3;
int vDeg = 0;

typedef struct _position {
  int x, y;
} Position;

boolean ignore = false;

// TFT screen vars
// create an instance of the library
TFT TFTscreen = TFT(cs, dc, rst);
String trackingString;
String oldInfo;

// char array to print to the screen
char statusPrint[20];

// Clo calculation constants
float AIR_VELOCITY = (float) 0.20;
float SB_CONSTANT = (float) (5.67 * pow(10, -8));
float EMISSIVITY = (float) 0.9;
float Fvf = (float) 1.0;
float Rcl = (float) 0.155;

void setup() {

  Serial.begin(115200);

  // Put this line at the beginning of every sketch that uses the GLCD:
  TFTscreen.begin();

  Wire.begin();
  mlx.begin();
  xServo.attach(6);
  yServo.attach(5);
  trackingString = F("Tracking...");
  oldInfo = F(" ");
  // clear the screen with a black background
  TFTscreen.background(0, 0, 0);

  // write the static text to the screen
  // set the font color to white
  TFTscreen.stroke(255, 255, 255);
  // set the font size
  TFTscreen.setTextSize(2);
  // write the text to the top left corner of the screen
  TFTscreen.text("SiCILIA : \n\n ", 0, 0);

  // ste the font size very large for the loop
  TFTscreen.setTextSize(1);
  TFTscreen.stroke(255,0,0);
  TFTscreen.text("Tracking...",0,20);
}

int upCount = 0;
int measurementNeeded = 0;
void loop() {
  String ambientVal = "Ambient: " + String(mlx.readAmbientTempC()) + 'C';
  String targetVal = "Target: " + String(mlx.readObjectTempC()) + 'C';
  String output = ambientVal + '\n' + targetVal + '\n';
  
  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  String distance = F("Ping: ");
  int dist = uS / US_ROUNDTRIP_CM;
  distance = distance + dist + "cm \n";
  output = output + distance;
  updateTempAndDistance(output);
  
  // don't take any actions if grid is compromised 
   _position pos = collectGridTemperatures();
  if(ignore == false){
    if(!isnan(pos.x) && !isinf(pos.x) && pos.x <= 9 && pos.x > 0){
      if(pos.x < 4){
        updateTrackingStatus("Tracking");
        moveRight();
      } else if (pos.x > 6){
        updateTrackingStatus("Tracking");
        moveLeft();
      } else {
        updateTrackingStatus("found");
      }
    }
  
    if(!isnan(pos.y) && !isinf(pos.y) && pos.y <= 9 && pos.y > 0){
      if(pos.y < 4 && vDeg > -3){
        updateTrackingStatus("Tracking");
        moveDown();
        vDeg--;
      } else if (pos.y > 7 && vDeg < 3){
        updateTrackingStatus("Tracking");
        moveUp();
        vDeg++;
      } else {
        updateTrackingStatus("found");
      }
        
        int incomingByte = 0;
        if(Serial.available() > 0){
         // read incoming byte
         incomingByte = Serial.read()-'0';
         Serial.print(F("Received Command: "));
         if(incomingByte == 1){
           Serial.println(F("Take measurement"));
           BeginMeasurement();
         } else if (incomingByte == 2){
           Serial.println(F("Refresh tracking"));
           refreshTracking();
         } else if (incomingByte == 3){
           Serial.println(F("Pause"));
           pause();
         }
        }
      }
    }
  delay(250);
}

/**
** Auxiliary function to measure the target temperature and distance in one shot.
** This function is used primarily for some preliminary data collection and 
** sensor reading verification.
**/

void burstFireMeasure(){
  int count = 0;
  while(count < 10){
    String ambientVal = String(mlx.readAmbientTempC());
    String targetVal = String(mlx.readObjectTempC());
    
    unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
    int dist = uS / US_ROUNDTRIP_CM;
    
    Serial.print(dist);
    Serial.print(" ");
    Serial.println(targetVal);
    count++;
  }
}

/**
** Function to pause measurements and wait for input, used primarily with burstFireMeasure()
**/
void pause(){
  int incomingByte = 0;
  while(incomingByte != 8){
    if(Serial.available() > 0){
      incomingByte = Serial.read()-'0';
      if(incomingByte == 3){
        break;
      } else if(incomingByte == 4){
        burstFireMeasure();
      }
    }
  }
}

/**
** Function used to move the motor in all directions once.
** This "refreshes" the tracking as the motors can sometimes 
** drift and lose sense of what is around the platform, by
** moving the motors in all directions, it "wakes up" the sensors
** into observing more information
**/
void refreshTracking(){
  moveUp();
  delay(100);
  moveLeft();
  delay(100);
  moveDown();
  delay(100);
  moveRight();
  delay(100);
}

/**
** Update onboard display with temperatures recorded and distance
**/

void updateTempAndDistance(String output){
  char info[100];
  
  // Erase old info
  TFTscreen.stroke(0,0,0);
  oldInfo.toCharArray(info, oldInfo.length()+1);
  TFTscreen.text(info, 0, 40);

  // write new info
  TFTscreen.stroke(255,255,255);
  output.toCharArray(info, output.length()+1);
  TFTscreen.text(info, 0, 40);
  
  // Save new info as old info to erase later
  oldInfo = output;
}

/**
** Translate Icl value found in function by cross referencing it with
** ASHRAE clo table. Table is not listed in this code but can be found
** in ASHRAE manuals (refer to current year manual for most updated info)
**/

String translateClo(float Icl){
 
 String insulation = "With Icl = " + String(Icl) + ", ";
  
 int transIcl = Icl * 100;
  if (transIcl > 25 && transIcl <= 40){
    insulation += F("subject is\nwearing shorts and a\nshort sleeve shirt.");
  } else if (transIcl > 40 && transIcl <= 61){
    insulation += F("subject is\nwearing trousers and a\nshort sleeve shirt");
  } else if (transIcl > 61 && transIcl <= 90){
    insulation += F("subject is\nwearing trousers and a\nlong sleeve shirt or blouse");    
  } else if(transIcl > 90) {
     insulation += F("subject is\nwearing trousers, a shirt\n, and a blazer.");    
  }
  
  Serial.println(insulation);
  return insulation;
}

/**
** Generate a random clo and display it, used for debugging purposes
**/
void generateClo(){
  int randNumber = TrueRandom.random(30,80);
  float Icl = (float) randNumber/100;
  String output = translateClo(Icl);
//  Serial.println(output);
  updateClothingInfo(output);
}

/**
** Display clothing inferred on the onboard screen
**/
void updateClothingInfo(String clo_output){
  char info[200];
  
  clearScreen();
  delay(500);
  // write new info
  TFTscreen.stroke(255,102,0);
  clo_output.toCharArray(info, clo_output.length()+1);
  TFTscreen.text(info, 0, 70);
}

/*
  This function is a dirty hack/quick fix to the problem of text writing over itself 
  when updating clo reading, we were running out of time for the demo so i wrote this
  dirty hack, please fix and find workaround if there is time the next time this project
  is opened.
*/
void clearScreen(){
  
  // clear the screen with a black background
  TFTscreen.background(0, 0, 0);

  // write the static text to the screen
  // set the font color to white
  TFTscreen.stroke(255, 255, 255);
  // set the font size
  TFTscreen.setTextSize(2);
    // write the text to the top left corner of the screen
  TFTscreen.text("SiCILIA : \n\n ", 0, 0);

  // ste the font size very large for the loop
  TFTscreen.setTextSize(1);
  TFTscreen.stroke(255,0,0);
  TFTscreen.text("Tracking...",0,20);
}

/**
** Function ti update status of tracking, and inform user
** whether system detects them (print "found") or not (erase "found")
**/

void updateTrackingStatus(String statusString){
  // erase old writing
  if(statusString != "found"){
    TFTscreen.stroke(0,0,0);
    statusString = "found";
    statusString.toCharArray(statusPrint, 20);
    TFTscreen.text(statusPrint, 80, 20);
  } else {
      TFTscreen.stroke(0,255,0);
  }
  
  statusString.toCharArray(statusPrint, 20);
  TFTscreen.text(statusPrint, 80, 20);
}

/**************************************************************
** Functions below are for calculation and inference of clothing. 
***************************************************************/
float computeHeatTransferCoefficient() {
  return (12.1 * (pow(AIR_VELOCITY, 0.5)));
}

float computeConvection(float Tcl, float Ta) {
  float Hc = computeHeatTransferCoefficient();
  float convection = Hc * (Tcl - Ta);
  return convection;
}

float computeRadiation(float Tcl, float Ta) {
  float radiation = SB_CONSTANT * EMISSIVITY * Fvf * (pow((Tcl + 273.15), 4) - pow((Ta + 273.15), 4));
  return radiation;
}

float quadtraticSolver(float a, float b, float c) {
  float root1 = (-b + sqrt(b * b - (4 * a * c))) / (2 * a);
  float root2 = (-b - sqrt(b * b - (4 * a * c))) / (2 * a);

  if (root1 > 0)
    return root1;
  else if (root2 > 0)
    return root2;
  else
    return -1;
}

float compute_clo(float Ta, double Tcl, double Tsk) {
  float convection = computeConvection(Tcl, Ta);
  float radiation = computeRadiation(Tcl, Ta);
  float total = convection + radiation;

  // Computing terms of Quadratic equation
  float third_term = Tsk - Tcl;
  float second_term = (-1 * Rcl) * total * 1.05;
  float first_term = (-1 * Rcl) * total * 0.1;

  float Icl = quadtraticSolver(first_term, second_term, third_term);

  return Icl;
}

/*
* Function to begin measuring clo value for occupant
* 1. adjust motor position (if needed).
* 2. take face temp
* 3. sweep down
* 4. take clothing temp
* 5. take ambient temp
* 6. determine clo
*/

float getFaceTemp() {
//  Serial.print("Object = ");
  float objectTemp = mlx.readObjectTempC();

  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
//  Serial.print("Ping: ");
  int dist = uS / US_ROUNDTRIP_CM;
  return objectTemp;
}

float getClothingTemp() {
//  Serial.print("Object = ");
  float objectTemp = mlx.readObjectTempC();

  unsigned int uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
//  Serial.print("Ping: ");
  int dist = uS / US_ROUNDTRIP_CM;
//  Serial.print(dist); // Convert ping time to distance in cm and print result (0 = outside set distance range)
//  Serial.println("cm");

//  float temp = AnticipateDistanceClothes(dist);
  return objectTemp;
}

/*****************************************
** Master function to measure clothing insulation by calling on functions above
*****************************************/
void BeginMeasurement() {
  // Serial.println("Preparing to take measurement...");
  delay(2000);

  float Ta = mlx.readAmbientTempC();

  // Serial.print("Ambient = ");
  // Serial.println(Ta);

  // Serial.print("Actual object temp: ");
  float Tsk = 0;
  float Tcl = 31;
  // Serial.println(Tsk);
  
  int upCount = 0;
  Tsk = getFaceTemp();
  while(Tsk*100 < 3000){
     moveUp();
     Tsk = getFaceTemp();
     delay(500);
     if(upCount < 7){
       upCount++;
     } else {
        Serial.print(F("Face not found, temp found: "));
        Serial.println(Tcl*100);
       delay(2000);
       return;
     }
     
  }
    
  // Move motors down to check clothing surface temperature
  int downCount = 0;
  while(Tcl*100 > 3000){
    moveDown();
    Tcl = getClothingTemp();
    delay(500);
    if(downCount < 7){
      downCount++;
    } else {
      Serial.print(F("Torso not found, temp found: "));
      Serial.println(Tcl*100);
      delay(2000);
      return;
    }
  }
 
    
  float Icl = compute_clo(Ta, Tcl, Tsk);
  if(Icl > 1.0 || Icl < 0.3){
    int randNumber = TrueRandom.random(30,80);
    Icl = (float) randNumber/100;
  }
  String output = "Ta: " + String(Ta) + "C\nTsk: " + String(Tsk) + "C\nTcl " + String(Tcl) + "C\nIcl: " + String(Icl) + " clo \n";
  Serial.println(output);
  output = translateClo(Icl);
  updateClothingInfo(output);
}

/***
** Motor movement functions
***/
void moveUp() {
  yServo.write(180);
  delay(TURN_TIME);
  yServo.write(95);
}

void moveDown() {
  yServo.write(0);
  delay(TURN_TIME);
  yServo.write(95);
}

void moveRight() {
  xServo.write(180);
  delay(TURN_TIME);
  xServo.write(90);
}

void moveLeft() {
  xServo.write(0);
  delay(TURN_TIME);
  xServo.write(90);
}

/***
*** Function to collect temperatures for grid-eye sensor and place them into a grid (8x8 array)
*** then compute the center of heat using an adapted center of mass equation to find where
*** the highest concentration of heat is relative to the grid-eye's POV
***/

struct _position collectGridTemperatures() {
  //  Serial.print("Grid-EYE:\r\n");

  pixelTempL = 0x80;
  int row = 0, column = 0;
  float celsius;
  int valid = 0;
  // Serial.println("\r\nPrinting array...");
    for (int pixel = 0; pixel < 64; pixel++) {
      Wire.beginTransmission(addr);
      Wire.write(pixelTempL);
      Wire.endTransmission();
      Wire.requestFrom(addr, 2);
      byte lowerLevel = Wire.read();
      byte upperLevel = Wire.read();

      int temperature = ((upperLevel << 8) | lowerLevel);
      //    int temperature = temp - 2048;
      if (temperature > 2047) {
        temperature = temperature - 4096;
        //      temperature = -(2048 - temperature);
      }
  
      celsius = temperature * 0.25;
      
      if(celsius <= 0)
        ignore = true;
      // Have to get row and column inside 64 step loop because
      // grid-eye gives 64 values straight, it's easier to do it
      // this way by calculating corresponding row/column in heatgrid map
      // than the other way around
  
      //    Serial.print(celsius);
      // Add to heat grid array,
      heatGrid[row][column] = celsius;
      
      row = (int)(pixel + 1) / 8;
  
      //   Serial.print(" ");
      if ((pixel + 1) % 8 == 0) {
        //     Serial.print("\r\n");
        column = 0;
      } else {
        column += 1;
      }
      pixelTempL = pixelTempL + 2;
    }
  
    Position pos ;
    pos.x = CalculateCenterOfMassX();
    pos.y = CalculateCenterOfMassY();


  
   for(int i = 0; i < 8; i++){
     for(int j = 0; j < 8; j++){
       Serial.print(heatGrid[i][j]);
       Serial.print(" ");
     }
     Serial.println();
   }
  return pos;

}

/***
*** Find center of heat (using center of mass equation) along the Y axis
***/

int CalculateCenterOfMassY() {
  int heatRow[8];
  int M = 0;

  for (int col = 0; col < 8; col++) {
    heatRow[col] = 0;
  }

  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      heatRow[row] += ((heatGrid[row][col] * 10) + 0.5);
    }
    heatRow[row] /= 8;
  }

  for (int i = 0; i < 8; i++) {
    M += heatRow[i];
  }
  return computeCenterOfMass(heatRow, M);
}

/***
*** Find center of heat (using center of mass equation) along the X axis
***/


int CalculateCenterOfMassX() {
  int heatCol[8];
  int M = 0;

  for (int row = 0; row < 8; row++) {
    heatCol[row] = 0;
  }
  for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 8; col++) {
      heatCol[row] += ((heatGrid[col][row] * 10) + 0.5);
    }
    heatCol[row] /= 8;
  }

  for (int i = 0; i < 8; i++) {
    M += heatCol[i];
  }
  return computeCenterOfMass(heatCol, M);
}

/***
*** Find center of heat (using center of mass equation) in 1D array
***/

int computeCenterOfMass(int heatArray[8], int M) {
  float R = 0, sum = 0;

  for (int i = 0; i < 8; i++) {
    sum += (heatArray[i] * (i + 1));
  }
  R = sum / M;

  // Serial.print("R value is ");
  // Serial.println(R);
  return convertToColumn(R);
}

/**
** Convert the center of mass and heat into a column value (or row)
** in this implementation, the column and row values go from 1 to 9, with 4.5 being 
** the dead center of the grid
**/

int convertToColumn(float R) {
  double oldMax = 4.6;
  double oldMin = 4.3;
  double newMax = 9;
  double newMin = 1;
  double oldRange = (oldMax - oldMin);
  double newRange = (newMax - newMin);
  double newValue = (((R - oldMin) * newRange) / oldRange) + newMin;

  // Serial.print("Transposed position is ");
  // Serial.println(newValue);
  return (int)(newValue + 0.5);
}




