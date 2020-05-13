#include <Wire.h>
#include <SoftwareSerial.h>
#include "GloMath.h"

#define warmup_time_seconds 5

// AAU sø
#define coordinate 57.015078, 9.977644

// Bruger Serial3 forbindelse
GloMath gps(20);

//************************Radiokommunikation**********************************************
const int pinTxd = 18;
const int pinRxd = 19;

//**********************************Differens beregner**************************************
float course = 157;  //course to follow
byte x=0;
float PDOut=0;
bool waypoint_init = false;

  
SoftwareSerial radioSerial = SoftwareSerial(pinRxd, pinTxd);

void radioPinSetup(){
  const int setPin = 7;
  const int pinAux = 9;

  const int pinEnable = 10;
    
  radioSerial.begin(9600);

  pinMode(setPin, OUTPUT);
  pinMode(pinAux, OUTPUT);
  pinMode(pinTxd, OUTPUT);
  pinMode(pinRxd, INPUT);
  pinMode(pinEnable, OUTPUT);

  digitalWrite(setPin, HIGH);
  digitalWrite(pinAux, HIGH); //fordi det er en sender
  digitalWrite(pinEnable, HIGH);
}

float setFilter(float course, float directions) {
  //This funtion sets the dutycycle of the filterpin, depending on the wished course and the current direction.
  char FilterPin=4;
  float difference = directions - course;
  if (difference < -180) {        //Ajusting so diffrence is within -180 to 180 deg.
    difference += 360;
  }
  else if (difference > 180) {    //Ajusting so diffrence is within -180 to 180 deg.
    difference -= 360;
  }

  if (difference < -100) {        //If boat is more than 100 deg out of course, pretend that it is 100 deg out of course.
    difference = -100;
  }
  else if (difference > 100) {
    difference = 100;
  }

  difference = ((difference + 100) * 255) / 200;  //Converts diffeence value from -100 to 100 into 8bit (0 to 255)

  analogWrite(FilterPin, difference);                     //Set output to LP-filter and PD-controller
  return difference;
}

//******************************************Retningsbestemmer*******************************************
void compassSetup() {
  //This function sets up the compass
  Wire.begin();
  byte Magnetometer = 0x1E;  //I2C adress of compass

  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  //Selects countinous mesurement:
  Wire.beginTransmission(Magnetometer);
  Wire.write(0x02); // Select mode register, 00000010
  Wire.write(0x00); // Continuous measurement mode, 00000000
  Wire.endTransmission();

  //Lets compass take 8 mesurements and calculate avg.
  Wire.beginTransmission(Magnetometer);
  Wire.write(0x00); // Select Configuration register a, 00000000
  Wire.write(0x70); // sets data output to 8 samples per measurement, 01110000
  Wire.endTransmission();
}

int readFromMagnetometer(byte registerAddress) {
  byte magnetometer = 0x1E;
  Wire.beginTransmission(magnetometer); // transmit to device
  Wire.write(registerAddress);
  Wire.endTransmission();
  Wire.requestFrom(magnetometer, 1);
  int output;
  if (Wire.available() <= 1)
  {
    output = Wire.read();
  }
  return output;
}

float getCompassHeading() {
  //This function returns the direction the compass is heading
  byte Magnetometer = 0x1E;         //Compass I2C adress

  byte Magnetometer_mX0 = 0x03;     //Adresses of x and y axis.
  byte Magnetometer_mX1 = 0x04;
  byte Magnetometer_mY0 = 0x07;
  byte Magnetometer_mY1 = 0x08;

  int mX0, mX1, mX_out;             //Variabels to store data form x and y axis.
  int mY0, mY1, mY_out;

  double heading, headingDegrees, declination;

  mX0 = readFromMagnetometer(Magnetometer_mX1);
  mX1 = readFromMagnetometer(Magnetometer_mX0);
  readFromMagnetometer(0x05);                       //All data must be read to allow new mesurement
  readFromMagnetometer(0x06);
  mY0 = readFromMagnetometer(Magnetometer_mY1);
  mY1 = readFromMagnetometer(Magnetometer_mY0);


  //---- X-Axis
  mX1 = mX1 << 8;      //Push data 8 bits to the left.
  mX_out = mX0 + mX1;  //Combine data from LSB and MSB. rules of 2'nd compliment.

  //---- Y-Axis
  mY1 = mY1 << 8;
  mY_out = mY0 + mY1;

  // ==============================
  //Calculating Heading
  heading = atan2(mY_out, mX_out);      //Calculate heading of compass returned i rad.

  // Correcting the heading with the declination angle depending on your location
  // You can find your declination angle at: https://www.ngdc.noaa.gov/geomag-web/
  declination = -0.057;
  heading += declination;

  // Correcting when signs are reveresed
  if (heading < 0) heading += 2 * PI;

  // Correcting when heading is greater than 2PI.
  if (heading > 2 * PI)heading -= 2 * PI;

  heading -= polyfit(heading);    //Compass calibration. Comment out for raw data.
  
  if (heading > 2 * PI) heading -= 2 * PI;
  if (heading < 0) heading += 2 * PI;

  headingDegrees = heading * 180 / PI; // The heading in Degrees unit
  
  return headingDegrees;
}


double polyfit(double nonCalHeading2) {
  //Calculates the calibration.
  double expArray[9];

  expArray[8] = 0.000204998906657;
  expArray[7] = -0.005670887551281;
  expArray[6] = 0.064399538761207;
  expArray[5] = -0.382471429941165;
  expArray[4] = 1.243422156440348;
  expArray[3] = -2.097356823377456;
  expArray[2] = 1.628895204121524;
  expArray[1] =  -0.752722201417257;
  expArray[0] =   0.261995404036337;

  double hold = 0;
  for (int i = 9 - 1; i >= 0; i--) {
    hold += expArray[i] * pow(nonCalHeading2, i);
  }
  return (hold);
}

void setup() {
	Serial.begin(9600);
	gps.version(true);
	radioPinSetup();
	compassSetup();
}

void loop() {

	gps.getCourse();
  gps.printCoordinates();

	// Båden får et par sekunder til at varme op og få GPS lock
	if (millis() > warmup_time_seconds*1000 && gps.isReady() == true && waypoint_init == false){
		
		//gps.distToDest();
		gps.nextWaypoint(coordinate);
		//gps.getCourse();

		Serial.println("Setting next waypoint");
		waypoint_init = true;
	} else {Serial.println("GPS not ready");}

	// Program loop for systemtest
	while (waypoint_init == true){
		
		// I mens afstanden til destinationen er større end 2 skal kursen følges
		while (gps.destDist() > 2){
			course = float(gps.getCourse());
			processError(course);
		}

		//Waypoint sættes til at være lig startpositionen
		double temp[2] = {gps.nav.dest[0],gps.nav.dest[1]};
		gps.nextWaypoint(gps.nav.start[0],gps.nav.start[1]);
		gps.nav.start[0] = temp[0];
		gps.nav.start[1] = temp[1];

		// Sejl samme vej tilbage
		while (gps.destDist() > 1){
			course = float(gps.getCourse());
			processError(course);
		}

		while(true){Serial.println("Misson succesful"); delay(5000);}
	}

}

void processError(float course){

	//gps.printCourseInfo();
  Serial.println("\n***Planlægger***");
	gps.printCoordinates();

    Serial.println("\n***Kursinfo***");
  gps.printCourseInfo();

	float heading = getCompassHeading();
	float difference = setFilter(course, heading);

	radioSerial.print("\nCourse:	"); radioSerial.println(course);
	radioSerial.print("Heading:	"); radioSerial.println(heading);
	radioSerial.print("Difference: 	"); radioSerial.println(difference);
	radioSerial.print("Input A0[V]: ");radioSerial.println(PDOut);

  Serial.println("\n***Bestemmer + diff***");
	Serial.print("Course:		"); Serial.println(course);
	Serial.print("Heading:	"); Serial.println(heading);
	Serial.print("Difference: 	"); Serial.println(((difference-128)/256*200));
	Serial.print("Input A0[V]: 	");Serial.println(PDOut);

	delay(50);
}
