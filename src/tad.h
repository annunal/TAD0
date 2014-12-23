#include "tadParameters.h"
#include <stdio.h>
#include <linux/limits.h>

#ifndef TRUE
#define FALSE 0;
#define TRUE 1;
#endif

#define MAX_PROP 10
#define RETRY_BUFFER "retryStore.txt"
#define RETRY_BUFFER_GPS "retryStoreGPS.txt"
#define RETRY_BUFFER1 "retryStore1.txt"
#define RETRY_BUFFER_GPS1 "retryStoreGPS1.txt"

extern char *status[MAX_PROP];

struct sensorGrid {
double temperature;
double pressure;
double distance;
double batteryVoltage;
double panelVoltage;
double latitude;
double longitude;
double elevation;
char * gpstime;
char * gpsdate;
};

void initAnalysis ();
void initBarometer ();
void initSonar ();
void initDisplay ();
void initBigDisplay ();
void initSiren(int pin);
void initServo(int pin, FILE * inputFp);
void initVoltage();
void initSensors();

void startSensorsReading();
void startServo();
void bigDisplay (int linec, char *lines[]);
void siren(char *arg);

void *readBarometerThread(void *args);
void *readDistanceThread(void *args);
void *readVoltagesThread(void *args);
void *readSerialThread(void *args);
void *readGPSThread(void *args);
void addMeasure(struct sensorGrid *grid, double time0);

void writeStatus();
void printLog( char * line );
void printRetry(char *line) ;

typedef struct {
        char title[PATH_MAX];
        char location[PATH_MAX];
        char watchFolder[PATH_MAX];
        char watchFile[PATH_MAX];
        double positionX;
        double positionY;
        char SaveURL[4096];
        char AlertURL[4096];
        char gpsURL[4096];
        char IDdevice[20];
        int Interval;
        int n300;
        int n30;
        double threshold;
        double ratioRMS;
        double AddRMS;
        int backFactor;
        char commandSerial[PATH_MAX];
        char sensorSerial[PATH_MAX];
        int sensorRate;
        float sensorMultFac;
        float sensorAddFac;
        int batteryPin;
		float batteryMultiplier;
        int panelPin;
		float panelMultiplier;
        int voltageInterval;
        int gpsRate;
        int gpsDeltaTSave;
        char gpsSerial[PATH_MAX];
} configuration;
