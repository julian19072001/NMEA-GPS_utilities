/*!
 *  \file    NMEA-GPSutils.c
 *  \author  Julian Della Guardia
 *  \date    28-09-2025
 *  \version 1.0
 *
 *  \brief   Library to get GPS data out of a NMEA stream along with nice to have conversion function
 * 
 *  \cite    Based on the "Autorouter" library written for the autonomous EVA project at the HvA by JDB
 *  \cite    Reference for all Haversine stuff: https://www.movable-type.co.uk/scripts/latlong.html
 */

#include "NMEA-GPSutils.h"
#include "../RPI-serial/RPIserial.h"

device gpsModule;


/* ----- BEGIN OF RMC UTILS ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
#define LINE_LEN 1024
#define RMCARRAY_GROWRATE 100

/*! \brief Check if the checksum at the end of the NMEA line is correct
 *  
 *  \param string location of NMEA string
 * 
 *  \return true if checksum is correct or absent
 */
bool validateNmeaChecksum(const char *string){
    // Check if there is a checksum present
    const char *endOfData = strchr(string, '*');
    // If there is no checksum return what is defined
    if (!endOfData) return true; 
    
    // Calculate checksum from received data
    unsigned char checkSum = 0;
    for (const char *c = string + 1; c < endOfData; ++c) checkSum ^= (unsigned char)(*c);
    
    // Compare calculated and received checksum
    unsigned int given = 0;
    if (sscanf(endOfData + 1, "%2x", &given) == 1){
        return checkSum == (unsigned char)given;
    }
    return false;
}/*validateNmeaChecksum*/


/*! \brief Split line into seprate fields
 *  
 *  \param string location of NMEA string
 * 
 *  \param fields location of where separate fields should be saved
 * 
 *  \return number of fields
 */
int splitFields(char *string, char **fields){
    int i = 0;
    char *pointer = string;
    while (i < MAX_FIELDS) {
        fields[i++] = pointer;
        char *comma = strchr(pointer, FIELD_TERMINATOR);
        if (!comma) break;
        *comma = '\0';
        pointer = comma + 1;
        /* if pointer stays on '\0' (for example being followed by ,), then pointer == "" and a empty field will be created */
    }
    return i;
}/*split Fields*/


/*! \brief convert NMEA gps notation to decimal notation
 *  
 *  \param nmeaGPS gps location in degrees + decimal minutes
 * 
 *  \param direction N/S or E/W depeding if longitude or latitude is being converted
 * 
 *  \return GPS location in decimal notation
 */
double convertNmeaToDeg(const char *nmeaGPS, char direction){
    if (!nmeaGPS || nmeaGPS[0] == '\0') return NAN;
    
    double v = atof(nmeaGPS);
    double deg = floor(v / 100.0);
    double minutes = v - (deg * 100.0);
    double dec = deg + (minutes / 60.0);
    
    if (direction == 'S' || direction == 'W') dec = -dec;
    return dec;
}/*convertNmeaToDeg*/


/*! \brief parse time into separate fields
 *  
 *  \param utc time in format hhmmss.ss
 * 
 *  \param hour location of where hours needs to written to
 * 
 *  \param min location of where minutes needs to written to
 * 
 *  \param sec location of where seconds needs to written to
 * 
 *  \param milliSec location of where milliseconds needs to written to
 */
void parseTime(const char *utc, int *hour, int *min, int *sec, int *milliSec){
    *hour = *min = *sec = *milliSec = 0;
    
    if (!utc || strlen(utc) < 6) return;
    char buf[32];
    strncpy(buf, utc, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    char hh[3] = {buf[0], buf[1], '\0'};
    char mm[3] = {buf[2], buf[3], '\0'};
    *hour = atoi(hh);
    *min = atoi(mm);
    
    double secd = atof(buf + 4); // includes fractional seconds
    *sec = (int)floor(secd);
    *milliSec = (int)round((secd - *sec) * 1000.0);
}/*parseTime*/


/*! \brief parse date into separate fields
 *  
 *  \param time date in format ddmmyy
 * 
 *  \param day location of where day needs to written to
 * 
 *  \param month location of where month needs to written to
 * 
 *  \param year location of where year needs to written to
 */
void parseDate(const char *time, int *day, int *month, int *year){
    *day = *month = *year = 0;
    
    if (!time || strlen(time) < 6) return;
    
    char dd[3] = {time[0], time[1], '\0'};
    char mm[3] = {time[2], time[3], '\0'};
    char yy[3] = {time[4], time[5], '\0'};
    
    *day = atoi(dd);
    *month = atoi(mm);
    int yyv = atoi(yy);
    *year = 2000 + yyv; // assume 2000+
}/*parseDate*/


/*! \brief parse RMC data from sting into a struct
 *  
 *  \param line RMC data line
 * 
 *  \param day Struct where the RMC dat needs to written to
 * 
 *  \retval false if parsing was not succesfull
 *  \retval true if parsing was succesfull
 */
bool parseRmcLine(char *line, RMC_t *out){
    char utc[32];

    if (!line || !out) return false;
    
    memset(out, 0, sizeof(*out));
    strncpy(utc, "", sizeof(utc));

    // Check if the NMEA line has a RMC header
    if (!(strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0)) {
        return false;
    }

    // Check if checksum is correct
    if (!validate_nmea_checksum(line)) return false;

    // Remove checksum before splitting the line into fields
    char *star = strchr(line, '*');
    if (star) *star = '\0';

    // Split all fields into a seprate variable
    char *fields[MAX_FIELDS];
    int nfields = split_nmea_fields(line, fields);
    // Clear fields if number of received fields is not correct
    const char *time_s = (nfields > 1) ? fields[1] : "";
    const char *status = (nfields > 2) ? fields[2] : "";
    const char *lat_s = (nfields > 3) ? fields[3] : "";
    const char *latNS = (nfields > 4) ? fields[4] : "";
    const char *lon_s = (nfields > 5) ? fields[5] : "";
    const char *lonEW = (nfields > 6) ? fields[6] : "";
    const char *speed_s = (nfields > 7) ? fields[7] : "";
    const char *course_s = (nfields > 8) ? fields[8] : "";
    const char *date_s = (nfields > 9) ? fields[9] : "";
    const char *magvar_s = (nfields > 10) ? fields[10] : "";
    const char *magvarEW_s = (nfields > 11) ? fields[11] : "";
    const char *mode_s = (nfields > 12) ? fields[12] : "";
    const char *navStatus_s = (nfields > 13) ? fields[13] : "";

    // status A=valid, V=invalid
    out->valid = (status && status[0] == 'A');

    // Parse time
    if (time_s) parse_time(time_s, &out->hour, &out->minute, &out->second, &out->msec);

    // Convert gps location into decimal notation
    out->latDeg = convert_nmea_to_deg(lat_s, (latNS && latNS[0]) ? latNS[0] : '\0');
    out->lonDeg = convert_nmea_to_deg(lon_s, (lonEW && lonEW[0]) ? lonEW[0] : '\0');

    // speed and course
    double speed_knots = (speed_s && speed_s[0]) ? atof(speed_s) : 0.0;
    out->speed = speed_knots * 1.852;
    out->courseDeg = (course_s && course_s[0]) ? atof(course_s) : NAN;

    // Parse date
    parse_date(date_s, &out->day, &out->month, &out->year);

    // magvar (can be empty)
    out->hasMagvar = (magvar_s && magvar_s[0]);
    if (out->hasMagvar) {
        out->magvar = atof(magvar_s);
        if ((magvarEW_s && magvarEW_s[0]) ? magvarEW_s[0] : '\0' == 'W') out->magvar = -out->magvar;
    }
    
    out->mode = (mode_s && mode_s[0]) ? mode_s[0] : '\0';
    out->navStatus = (navStatus_s && navStatus_s[0]) ? navStatus_s[0] : '\0';

    return true;
}/*parseRmcLine*/


/*! \brief parse file with RMC data to create array with waypoints (waypoint path)
 *  
 *  \param fileName filename of file containing multiple RMC lines
 * 
 *  \param numRead Location where to save how many RMC lines are withing the file
 * 
 *  \returns array of RMC data points
 */
RMC_t *parseRmcFile(const char *fileName, int *numRead){
	RMC_t *res = NULL, curRMC;
	int resCapacity = 0, lineNo = 0,validRMC;
	FILE *inFile = NULL;
	char buf[LINE_LEN];

	*numRead = 0;

    // Check if filename exists
	inFile = fopen(fileName, "r");
	if(inFile == NULL) {
		fprintf(stderr, "*** Opening RMC map path file");
		return NULL;
	}
    
    // Read all lines from file.
	while(fgets(buf, LINE_LEN, inFile) != NULL) {
		lineNo++;
        // Check if line is a valid RMC line
		validRMC = parseRmcLine(buf, &curRMC);

		if(validRMC) {
            // If the array is too small allocated more room
			if(resCapacity <= *numRead) {
				resCapacity += RMCARRAY_GROWRATE;
				res = (RMC_t *) realloc(res, resCapacity * sizeof(RMC_t));
				assert(res != NULL);
			}
			res[*numRead] = curRMC;
			(*numRead)++;			
		}
	}
	
	fprintf(stderr, "=== %d RMC waypoints read in %d lines\n", *numRead, lineNo);
	fclose(inFile);
  
	if(res == NULL) {
		fprintf(stderr, "Cant open file\n");
		exit(2);
	}
  
	if(*numRead < 2) {
		fprintf(stderr, "*** At least 2 valid RMC waypoints required (got %ls)\n", numRead);
		exit(3);
	}
  
	return res;
}/*parseRmcFile*/

/*! \brief Get a new RMC line
 *  
 *  \returns stuct with new RMC data
 * 
 *  \details Waits until a new RMC data line is received so make sure 
 *           that data is being send or program will get stuck
 */
RMC_t getNewRmcLine(void){
    char buf[LINE_LEN];
    int idx = 0;
    RMC_t rmcRes;

    while(1){    
        if (canReadByte(&gpsModule)) {
            uint8_t byte;
            int res = readByte(&gpsModule, &t);
            if (res == 1) {
                char character = (char)t;
                if (character == '\n') {
                    
                    buf[idx] = '\0';
                    
                    // strip trailing \r if present
                    if (idx > 0 && buf[idx-1] == '\r') buf[idx-1] = '\0';
                    
                    if(parseRmcLine(&buf, &rmcRes)) return rmcRes;
                    
                    idx = 0;
                } 
                
                // Check for buffer overflow - reset and skip
                if (idx < LINE_LEN - 1) buf[idx++] = character;
                else idx = 0;
            } 
        } else if (res < 0) {
            fprintf(stderr, "Error reading byte from serial\n");
            usleep(10000);
        } else return NULL;
    }

    return NULL;
}/*getNewRmcLine*/


/*! \brief Print data from RMC struct
 *  
 *  \param printLocation File where the data needs to printed
 * 
 *  \param gpsData RMC struct from which the data needs to be printed
 */
void printRmcData(FILE* printLocation, RMC_t gpsData){
    if(!printLocation) return;
    fprintf(printLocation, "--- GNRMC ---\n");
    fprintf(printLocation, "UTC time: %02d:%02d:%02d.%03d\n", gpsData.hour, gpsData.minute, gpsData.second, gpsData.msec);
    fprintf(printLocation, "Valid: %s\n", gpsData.valid ? "A (valid)" : "V (invalid)");
    if (!isnan(gpsData.latDeg) && !isnan(gpsData.lonDeg))
        fprintf(printLocation, "Lat/Lon: %.6f, %.6f\n", gpsData.latDeg, gpsData.lonDeg);
    else
        fprintf(printLocation, "Lat/Lon: (empty)\n");
    fprintf(printLocation, "Speed: %.2f km/h\n", gpsData.speed);
    if (!isnan(gpsData.courseDeg))
        fprintf(printLocation, "Course: %.1f°\n", gpsData.courseDeg);
    else
        fprintf(printLocation, "Course: (empty)\n");
    fprintf(printLocation, "Date: %04d-%02d-%02d\n", gpsData.year, gpsData.month, gpsData.day);
    if (gpsData.hasMagvar) fprintf(printLocation, "Magvar: %.2f\n", gpsData.magvar);
    else fprintf(printLocation, "Magvar: (null)\n");
    if (gpsData.mode) fprintf(printLocation, "Mode: %c\n", gpsData.mode);
    if (gpsData.navStatus) fprintf(printLocation, "Nav status: %c\n", gpsData.navStatus);
    fprintf(printLocation, "---------------\n");
}

/* ----- END OF RMC UTILS ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */
/* ----- BEGIN OF GPS UTILS ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
// For explanation for all the calculation look at: https://www.movable-type.co.uk/scripts/latlong.html
// Functions written by JDB in "autorouter.c"

#define EARTH_RADIUS 6371e3
#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

/*! \brief Get the distance in meters between two waypoints
 *  
 *  \param orig First waypoint
 * 
 *  \param dest Second waypoint
 */
double getDistance(const RMC_t *orig, const RMC_t *dest){
	double phi1 = DEG2RAD(orig->latDeg);
	double phi2 = DEG2RAD(dest->latDeg);
	double deltaphi = DEG2RAD(dest->latDeg - orig->latDeg);
	double deltalambda = DEG2RAD(dest->lonDeg - orig->lonDeg);

	double a = sin(deltaphi / 2.0) * sin(deltaphi / 2.0)
				+ cos(phi1) * cos(phi2)
				* sin(deltalambda / 2) * sin(deltalambda / 2);
	double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

	return EARTH_RADIUS * c;
}/*getDistance*/


/*! \brief Get the distance until the end of the waypoint path in meters through following the path
 *  
 *  \param path Array with waypoints 
 * 
 *  \param numberOfWaypoints numberOfWaypoints in path
 * 
 *  \param currentWaypoint number of the waypoint to closest location
 */
double DistanceToEnd(RMC_t *path, int numberOfWaypoints, int currentWaypoint){
  double distanceLeft = 0;
  for(int i = numberOfWaypoints - 1; currentWaypoint < i; i--){
        distanceLeft += Distance(path + i - 1, path + i);
  }
  return distanceLeft;
}/*getDistanceToEnd*/


/*! \brief Get the direction in degrees the destination is relative to the origin
 *  
 *  \param orig First waypoint
 * 
 *  \param dest Second waypoint
 */
double getBearing(const RMC_t *orig, const RMC_t *dest){
	double phi1 = DEG2RAD(orig->latDeg);
	double phi2 = DEG2RAD(dest->latDeg);
	double deltalambda = DEG2RAD(dest->lonDeg - orig->lonDeg);

	double y = sin(deltalambda) * cos(phi2);
	double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltalambda);

	double bearing = RAD2DEG(atan2(y, x));

	return (bearing >= 0.0) ? bearing : bearing + 360.0; 
} /*getBearing*/


/*! \brief Get the waypoint in the middle of two different waypoints
 *  
 *  \param orig First waypoint
 * 
 *  \param dest Second waypoint
 * 
 *  \param mid mid point between the waypoints
 */
void MidWaypoint(const RMC_t *orig, const RMC_t *dest, RMC_t *mid){
    double phi1 = DEG2RAD(orig->latDeg);
	double phi2 = DEG2RAD(dest->latDeg);
	double deltalambda = DEG2RAD(dest->lonDeg - orig->lonDeg);

    double x = cos(phi2) * cos(deltalambda);
    double y = cos(phi2) * sin(deltalambda);
    double phi3 = atan2(sin(phi1) + sin(phi2), sqrt((cos(phi1) + x) * (cos(phi1) + x) + y * y));
    double lambda3 =  orig->lonDeg + atan2(y, cos(phi1) + x);

    /* Normalize the longitude */
    lambda3 += 3.0 * M_PI;
    while(lambda3 > 2.0 * M_PI)
        lambda3 -= 2.0 * M_PI;
    lambda3 -= M_PI;

    mid->latDeg = RAD2DEG(phi3);
    mid->lonDeg = RAD2DEG(lambda3);
} /* MidWayPoint */


/*! \brief Create new waypoint relative to origin in the direction of bearing with the distance in meters form the origin
 *  
 *  \param orig First waypoint
 * 
 *  \param bearing direction relative to the origin waypoint in which the new waypoint should be created
 * 
 *  \param dist distance in meters from the origin waypoint that the new waypoint should be created at
 * 
 *  \param dest location where the new waypoint should be saved
 */
void ExtendPath(const RMC_t *orig, double bearing, double dist, RMC_t *dest){
	double phi1 = DEG2RAD(orig->latDeg);
	double lambda1 = DEG2RAD(orig->lonDeg);

	double phi2 = asin(sin(phi1) * cos(dist / EARTH_RADIUS)
					+ cos(phi1) * sin(dist / EARTH_RADIUS) * cos(DEG2RAD(bearing)));
	double lambda2 = lambda1 + atan2(sin(DEG2RAD(bearing)) * sin(dist / EARTH_RADIUS) * cos(phi1),
									cos(dist / EARTH_RADIUS) - sin(phi1) * sin(phi2));

	/* Normalize the longitude */
	lambda2 += 3.0 * M_PI;
	while(lambda2 > 2.0 * M_PI)
		lambda2 -= 2.0 * M_PI;
	lambda2 -= M_PI;

	assert(dest != NULL);

	dest->latDeg = RAD2DEG(phi2);
	dest->lonDeg = RAD2DEG(lambda2);
}/*ExtendPath*/

/* ----- END OF GPS UTILS ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ */

/*! \brief Establish serial connection with gps module
 * 
 *  \param port tty portname where the gps module is connected to
 * 
 *  \param baudrate select baudrate that the gps module is expecting
 */
void setupGpsDevice(const char* port, int baudrate){
    gpsModule.deviceport = port;
    gpsModule.baudrate= baudrate;
    setupDevice(&gpsModule)
}/*setupGpsDevice*/

/*! \brief Close serial connection with lidar
 */
void closeGpsDevice(void){
	closeDevice(&gpsModule, true);
}/*closeGpsDevice*/