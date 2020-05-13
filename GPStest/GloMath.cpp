#include "Arduino.h"
#include "GloMath.h"
#include "math.h"

// Constructor
GloMath::GloMath(float metersAhead = 10, bool GPSdebug = false){
	this->GPSdebug = GPSdebug;
	// KM ahead
	this->metersAhead = metersAhead/1000;
  	Serial1.begin(9600);
}

// Ændrer afstanden frem til følgepunktet
void GloMath::setMeters(int metersAhead){
	// Change predefined meters ahead variable
	this->metersAhead = metersAhead;
}

// Trækker strings ud af seriel forbindelse til GPS
void GloMath::gpsPull(struct GloMath::gpsData *output) {
	int pos = 0;                    // Variable to hold position
	String tempMsg = Serial1.readStringUntil('\n'); // Temporary output for $GPRMC
	String place[12]; // String array to hold each entry
	int stringstart = 0;        // Variable to hold start location of string
	//Serial.println(tempMsg);	// Prints tempMsg to serial monitor
	Serial1.flush(); // Serial connection is flushed
	while (Serial1.available() > 0) { // While GPS is available
		Serial1.read(); // Read entire output
	}
  
	if (Serial1.find("$GPRMC,")) { // If Minimum Configuration is present...
		for (int i = 0; i < tempMsg.length(); i++) {  // For-loop to go through each character
			if (tempMsg.substring(i, i + 1) == ",") { // If the next character in $GPRMC is a comma
				place[pos] = tempMsg.substring(stringstart, i); // Substring is written to array
				stringstart = i + 1; // Comma is skipped by increasing i by 1
				pos++; // Switch to next position in array 
			}
			if (i == tempMsg.length() - 1) { // If end of message is reached
				place[pos] = tempMsg.substring(stringstart, i); // Substring is written to array
			}
		}
	}

	// Source : http://www.davidjwatts.com/youtube/GPS-software-serial.ino
	// Finally relevant strings are exported using pointers

	output -> latitude = place[2];   // latitude
	output -> lathem = place[3];     // lat hem
	output -> longitude = place[4];  // longitude
	output -> longhem = place[5];    // long hem

	if (place[1] == "A"){
		this->valid = true;
	} else {this->valid = false;}
}

bool GloMath::isValid(){
	return this->valid;
}

// Formatterer GPS koordinaterne til DD med fortegn
void GloMath::gpsFormat(){

	// Temporary strings
	String lati, longi, lathem, longhem, latdec, longdec;

	// Determine positive or negative hemisphere
    if (this->pulled.lathem == "N") lathem = "+"; 		// If north, number is 1
    else if (this->pulled.lathem == "S") lathem = "-"; 	// If south, number is 2
    if (this->pulled.longhem == "E") longhem = "+"; 	// If north, number is 1
    else if (this->pulled.longhem == "W") longhem = "-";// If south, number is 2

    // Remove the decimal seperator by seperating substrings
    lati = this->pulled.latitude.substring(0,4) + this->pulled.latitude.substring(5,10);
    longi = this->pulled.longitude.substring(0,5) + this->pulled.longitude.substring(6,11);

    // Convert decimals from DM to DD
	latdec = String((lati.substring(2,lati.length()+1).toDouble()/6000000),7).substring(2,9);
    longdec = String((longi.substring(3,longi.length()+1).toDouble()/6000000),7).substring(2,9);

    // Splice together previous strings and save them as a Double variable
    this->nav.current[0] = (lathem + this->pulled.latitude.substring(0,2) +"."+ latdec).toDouble();
   	this->nav.current[1] = (longhem + this->pulled.longitude.substring(0,3) +"."+ longdec).toDouble();
}

// "Nulstiller" koordinater omkring startpunktet
void GloMath::zeroCoordinates(){

	// Zero out destination in relation to start
	this->nav.dest[0] = (this->nav.dest[0]-this->nav.start[0])*this->lonFact;
	this->nav.dest[1] = (this->nav.dest[1]-this->nav.start[1])*this->latFact;

	// Zero out current position in relation to start
	this->nav.current[0] = (this->nav.current[0]-this->nav.start[0])*this->lonFact;
	this->nav.current[1] = (this->nav.current[1]-this->nav.start[1])*this->latFact;

	//printCoordinates();
}

// Rotationsmatice som gemmer i pointers
void GloMath::rot(double ang, double *lati, double *longi){
	// Temorary value for longitude
	double longitemp = *longi;

	// Rotation matrix
	*longi = (*longi*cos(ang))-(*lati*sin(ang));
	*lati = (longitemp*sin(ang))+(*lati*cos(ang));
}

// Rotationsmatice som returnerer værdier, ikke destruktiv
double GloMath::rot(double ang, double lati, double longi, bool returnLongi = true){

	// Uses previous rotation matrix
	rot(ang,&lati, &longi);

	// Return the selected coordinate 
	double returner;
	if (returnLongi == true){
		returner = longi;
	} else {returner = lati;}
	return returner;
}

// Angiv koordinat til næste navigationspunkt
void GloMath::nextWaypoint(double waypointlati, double waypointlong){

	// Redefines next waypoint from current position
	this->nav.start[0] = this->nav.current[0];
	this->nav.start[1] = this->nav.current[1];
	this->nav.dest[0] = waypointlati;
	this->nav.dest[1] = waypointlong;
}

// Udregner vinkel mellem sejlrute og x-aksens positive retning
double GloMath::getAngle(bool printing = false){
	
	// If destination is west do PI+atan
	if (this->nav.dest[1] < 0){
		this->rotAngle = (PI+atan(this->nav.dest[0]/this->nav.dest[1]));

	// Else it is east do atan
	} else {
		this->rotAngle = (atan(this->nav.dest[0]/this->nav.dest[1]));
	}

	// Print if requested
	if (printing == true){
		Serial.print("\nDegs from equ : "); Serial.println(this->rotAngle*180/PI);
		Serial.print("Rads from equ : "); Serial.println(this->rotAngle);
	}
	return this->rotAngle;
}

// Oprindelig måde at regne kurs. Bruges ikke aktuelt
double GloMath::getAngleAlt(bool printing = false){
	float Ang = atan(rot(-this->rotAngle,this->nav.current[0],this->nav.current[1],false)/this->metersAhead);

	if ((Ang-this->rotAngle) > (PI/2)) {
		Ang = (PI*2)+Ang;
	}

	if (printing == true){
		//Serial.print("      Ang  "); Serial.println(Ang*(180/PI));
		//Serial.print("Angle Alt  "); Serial.println(90+((Ang-this->rotAngle)*(180/PI)));

		Serial.print("      Ang  "); Serial.println(Ang*(180/PI));
		Serial.print("Angle Alt  "); Serial.println(90-((this->rotAngle-Ang)*(180/PI)));
	}
	return (90-((this->rotAngle-Ang)*(180/PI)));
}

// Meter/grad for den gældende længdegrad
double GloMath::latFactor(){

	// Calculates meters/degree og longitude for given latitude
	this->latFact = 40075/(1/cos((this->nav.current[0]/180)*PI)*360);
	Serial.println(this->latFact);
	return this->latFact;
}

// Udregn navigationspunktets koordinat i længdegrader/breddegrader
void GloMath::getNavpoint(){

	// Course error is defined as rotated y-coordinate for current position
	this->courseError = rot(-this->rotAngle,this->nav.current[0],this->nav.current[1],false)*1000;

	// Nav point is on the course, and therefor has y-value of 0
	this->nav.navpoint[0] = 0;
	// Nav point x-value is rotated x-coordinate for current position plus meters value
	this->nav.navpoint[1] = rot(-this->rotAngle,this->nav.current[0],this->nav.current[1]) + this->metersAhead;

	// Nav point is rotated back using same angle as before
	rot(this->rotAngle,&this->nav.navpoint[0],&this->nav.navpoint[1]);

	// navpoint is divided by lat/lon factors and added to start coordinate
	this->nav.navpoint[0] = this->nav.navpoint[0]/this->lonFact + this->nav.start[0];
	this->nav.navpoint[1] = this->nav.navpoint[1]/this->latFact + this->nav.start[1];
}

// Printer information om kursen
void GloMath::printCourseInfo(){

	// Prints relevant course info
	Serial.print("Course to follow #    "); Serial.print(this->course,2); Serial.println(" degrees");
	Serial.print("Error from course #   "); Serial.print(this->courseError,2); Serial.println(" meters");
	Serial.print("Dist to destination # "); Serial.print(this->metersToDest,2); Serial.println(" meters");
}

// Printer alle kendte koordinater
void GloMath::printCoordinates(){

	// If coordinates are valid
	if (this->GPSdebug == true || this->nav.current[0] != double(0.0) && this->nav.current[1] != double(0.0)){

		// Prints all current coordinates
		Serial.print("Navgation point : \t"); Serial.print(this->nav.navpoint[0],6);
		Serial.print(" , "); Serial.println(this->nav.navpoint[1],6);

		Serial.print("Current point : \t"); Serial.print(this->nav.current[0],6);
		Serial.print(" , "); Serial.println(this->nav.current[1],6);

		Serial.print("Reference point : \t"); Serial.print(this->nav.start[0],6);
		Serial.print(" , "); Serial.println(this->nav.start[1],6);

		Serial.print("Destination point : \t"); Serial.print(this->nav.dest[0],6);
		Serial.print(" , "); Serial.println(this->nav.dest[1],6);//Serial.println("");
	} else {
		// Inform use of lack of lock
		Serial.println("GPS does not have lock");
	}
}

// Udregner vinklen der skal sejles for at nå til det ønskede punkt
double GloMath::navAngle(){

	// Source : https://link.springer.com/article/10.1186/s40537-019-0214-3
	double dLong = this->nav.current[1]-this->nav.navpoint[1];
	double A = cos(this->nav.navpoint[0]*PI/180)*sin(dLong*PI/180)*10E6;
	double B = (cos(this->nav.current[0]*PI/180)*sin(this->nav.navpoint[0]*PI/180)-(sin(this->nav.current[0]*PI/180)*cos(this->nav.navpoint[0]*PI/180)*cos(dLong*PI/180)))*10E6;

	double course = -(atan2(A,B)*180/PI);

	// if course is negative, add negative value to 360
	if (course < 0) {
		course = 360+course;
	}

	// set value and return
	this->course = course;
	return course;
}


double GloMath::distToDest(){

	// Pythagorean theorem used to calculate distance to destination using scaled coordinates
	this->metersToDest = 1000*sqrt(((this->nav.current[0]-this->nav.dest[0])*(this->nav.current[0]-this->nav.dest[0]))+((this->nav.current[1]-this->nav.dest[1])*(this->nav.current[1]-this->nav.dest[1])));
	return this->metersToDest;
}

// Taktisk sammensætning af ovenstående funktioner
double GloMath::getCourse(){
	// use GPS when possible
	if (this->GPSdebug == false){
		Serial.println("READING GPS");
		gpsPull(&this->pulled);
		gpsFormat();
	} else {
		delay(1000);
	}
	
	// If debugging, or GPS values are valid
	if (this->GPSdebug == true || this->nav.current[0] != double(0.0) && this->nav.current[1] != double(0.0)){

		// Place all coordinates in temporary struct
		this->temp = this->nav;

		// Calculate lat factor for current coordinates
		latFactor();

		// Zero in coordinates
		zeroCoordinates();

		// Calculate distance to destination
		distToDest();

		// Calculate angle from the equator
		getAngle();

		// Calculate angle from north in cartesian coordinates (ALTERNATIVE)
		getAngleAlt();

		// Point of navigation is calculated
		getNavpoint();

		// All other coordinates are restored
		this->nav.current[0] = this->temp.current[0];
		this->nav.current[1] = this->temp.current[1];

		this->nav.start[0] = this->temp.start[0];
		this->nav.start[1] = this->temp.start[1];

		this->nav.dest[0] = this->temp.dest[0];
		this->nav.dest[1] = this->temp.dest[1];

		// Angle between north, current position and destination
		navAngle();
	} 

	// return value for user
	return this->course;
}

float GloMath::courseFloat(){
	return float(this->course);
}

// Printer versionen til brugeren
float GloMath::version(bool print = false){
	if (print == true){
		Serial.print("GloMath version : "); Serial.println(this->versionID,2);
	return this->versionID;
	}
}

bool GloMath::isReady(){
	if (this->GPSdebug == true || this->nav.current[0] != double(0.0) && this->nav.current[1] != double(0.0)){
		return true;
	} else {return false;}
}

double GloMath::destDist(){
	return this->metersToDest;
}