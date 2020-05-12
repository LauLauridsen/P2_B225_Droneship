#include "GloMath.h"

// Add true as argument for testing
GloMath gps(500);

void setup() {
	Serial.begin(9600);

	gps.version(true);
	
					// Latitude							// Longitude
//	gps.nav.current[0] = 57.060901; 	gps.nav.current[1] = 9.936452;
//	gps.nav.start[0] = 57.054236; 		gps.nav.start[1] = 9.919875;	
//	gps.nav.dest[0] = 57.066447; 		gps.nav.dest[1] = 9.953335;

}

void loop() {

	gps.getCourse();
	gps.printCourseInfo();
	gps.printCoordinates();

	gps.nextWaypoint(57.024020, 9.948230);
}
