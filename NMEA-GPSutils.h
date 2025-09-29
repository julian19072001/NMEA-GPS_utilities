#ifndef _NMEAGPS_H_
#define _NMEAGPS_H_
  #include <stdio.h>
  #include <stdint.h>
  #include <stdbool.h>
  #include <string.h>
  #include <stdlib.h>
  #include <assert.h>
  #include <math.h>

    #define ACCEPT_NO_CHECKSUM true     // Set if a line is valid or invalid when no checksum is received 
    #define MAX_FIELDS  15              // Maximum number of fields in the received NMEA line
    #define FIELD_TERMINATOR ','        // Character that is used to terminate a field in the NMEA line
    

    typedef struct {
        bool valid;                 // status A = valid
        double latDeg, lonDeg;      // Decimal degrees
        double speed;               // Speed in km/h
        double courseDeg;           // Course over ground (w.r.t. True North)     
        bool hasMagvar;             // has Magnetic variation
        double magvar;              // Magnetic variation
        char mode;                  // positioning mode 
        char navStatus;             // nav status 
        int hour, minute, second, msec;
        int day, month, year;
    } RMC_t;

    bool parseRmcLine(char *line, RMC_t *out);
    RMC_t *parseRmcFile(const char *fileName, int *numRead);
    RMC_t getNewRmcLine(void);
    void printRmcData(FILE* printLocation, RMC_t gpsData);

    double getDistance(const RMC_t *orig, const RMC_t *dest);
    double getDistanceToEnd(RMC_t *path, int numberOfWaypoints, int currentWaypoint);
    double getBearing(const RMC_t *orig, const RMC_t *dest);
    void getMidWaypoint(const RMC_t *orig, const RMC_t *dest, RMC_t *mid);
    void ExtendPath(const RMC_t *orig, double bearing, double dist, RMC_t *dest);
    int getClosestWaypointIdx(const RMC_t *curPosRMC, const RMC_t *path, int numWaypoints);

    void setupGpsDevice(const char* port, int baudrate);
    void closeGpsDevice(void);
#endif