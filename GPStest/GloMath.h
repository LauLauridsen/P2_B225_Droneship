#ifndef GloMath_h
#define GloMath_h

#define lib_version 1.22

class GloMath{
	public:

		struct gpsData{
		    String latitude;
		    String lathem;
		    String longitude;
		    String longhem;
		} pulled;

		struct navSet{
			double current[2];
			double start[2];
			double dest[2];
			double navpoint[2];
		} nav;

		GloMath(float metersAhead, bool GPSdebug = false);
		void setMeters(int metersAhead);
		void nextWaypoint(double waypointlati, double waypointlong);
		double latFactor();
		double latFactor2();		
		void printCoordinates();
		void printCourseInfo();
		double getCourse();
		float courseFloat();
		double distToDest();
		float version(bool print = false);
		bool isReady();
		bool isValid();
		double destDist();


	private:
		//double course;

	protected:

	void gpsPull(struct GloMath::gpsData *output);
		void gpsFormat();
		void zeroCoordinates();
		void rot(double ang, double *lati, double *longi);
		double rot(double ang, double lati, double longi,bool returnLongi = true);
		double navAngle();
		void getNavpoint();
		double getAngle(bool printing = false);
		double getAngleAlt(bool printing = false);
		double course;
		navSet temp;
		double latFact = 1;
		double lonFact = 40075/360;
		bool valid;
		double rotAngle;
		double courseError;
		double metersToDest;
		bool GPSdebug;
		float metersAhead;
		const float versionID = lib_version;
};

#endif
