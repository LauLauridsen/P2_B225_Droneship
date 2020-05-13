#include "GloMath.h"

GloMath gps(20);

void setup() {
	Serial.begin(9600);

	gps.version(true);
	
							// Latitude							// Longitude
	//gps.nav.current[0] = 	57.023992; 	gps.nav.current[1] = 	9.951320;	
	gps.nav.dest[0] = 		57.019386; 	gps.nav.dest[1] = 		9.963343;
	gps.nav.start[0] = 		57.019355; 	gps.nav.start[1] = 		9.963990;
/*
	gps.nav.current[0] = 	57.027050; 	gps.nav.current[1] = 	9.949854;	
	gps.nav.start[0] = 		57.025085; 	gps.nav.start[1] = 		9.950966;
	gps.nav.dest[0] = 		57.028522; 	gps.nav.dest[1] = 		9.950809;
*/
	//gps.defineStart();
	//gps.nextWaypoint();
}

float coursetemp = 0;
double latitemp = 0, longitemp = 0;

void loop() {

	gps.getCourse();

	if (gps.isValid() == true){
		for(int i = 0 ; i < 10 ; i++){
			Serial.print(i+1); Serial.print(" .. ");

			gps.getCourse();

			Serial.println("\n***Kursinfo***");
			gps.printCourseInfo();
			Serial.println("\n***Koordinater***");
			gps.printCoordinates();

			coursetemp = coursetemp + gps.courseFloat();
			latitemp = latitemp + gps.nav.current[0];
			longitemp = longitemp + gps.nav.current[1];
		}

		Serial.println("\n****** GENNEMSNIT FUNDET ******\n");
		Serial.print("Course AVG : "); Serial.println(coursetemp/10,4);
		Serial.print("Coord  AVG : "); Serial.print(latitemp/10,6); Serial.print(" , "); Serial.println(longitemp/10,6);
		Serial.println("\n****** GENNEMSNIT FUNDET ******"); 

		//delay(2000);
		Serial.println("Gennemsnit nulstilles\n");
		coursetemp = 0;
		latitemp = 0;
		longitemp = 0;
	} else {
		Serial.println("GPS invalid");
		delay(2000);
	}

}
