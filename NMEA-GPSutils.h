#ifndef _NMEAGPS_H_
#define _NMEAGPS_H_
  #include <stdio.h>
  #include <stdint.h>
  #include <stdbool.h>
  #include <string.h>
  #include <stdlib.h>
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
        char magvarEW;              // 'E' or 'W'
        char mode;                  // positioning mode 
        char navStatus;             // nav status 
        int hour, minute, second, msec;
        int day, month, year;
    } RMC_t;

    bool parseRmcLine(char *line, RMC_t *out);
    void printRmcData(FILE* printLocation, RMC_t gpsData);
    
#endif