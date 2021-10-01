#ifndef DATAEXTRACTORUTILITIES_H
#define DATAEXTRACTORUTILITIES_H

#include<cstdio>

/**
 * @brief This header contents a
 * set of function and procedures
 * to assist data extracting from
 * files and C string.
 */


/**
 * @brief This function drop all characters
 * of a file stream until a endline character.
 *
 * @param source
 */
void endLine(FILE *source);

/**
 * @brief This function remove all character
 * of a C string.
 *
 * @param str - C string
 * @param caracter - Character that will be deleted.
 */
void removeCharacter(char *str, char caracter);

/**
 * @brief Remove all quotation marks of a string.
 *
 * @param str
 */
void removeQuotationMarks(char *str);

int extractField(FILE *source, char *field, char split);

int extractField(FILE *source, char *field, char cStart, char cEnd);

int extractField(FILE *source, char *field, const char* cStart, const char * cEnd);

int sExtractField(char **source, char *field, char split);

int sExtractField(char **source, char *field, char cStart, char cEnd);

int sExtractField(char **source, char *field, const char* cStart, const char * cEnd);

void ignoreLine(FILE *source);

int readHeader(FILE *source);

bool isCommentLine(FILE *file);

void extractCordinate(FILE *source, double *x, double *y, double *z);
void extractCordinate(char *source, double *x, double *y, double *z);

void extractGPSMsg(char *str, char *latitude, char *longitude, char *altitude, char *time, char *accuracy);
void extractGPSMsgSensorLOG(char *str, char *latitude, char *longitude, char *altitude, char *time, char *accuracy);

void extractDataCSV(FILE *source,int *msgID, int *labelID, char *sensorName,double *accuracy, char*sensorValue, char *timeStamp);

int extractData(FILE *source,int *msgID, int *labelID, char *sensorName,double *accuracy, char*sensorValue, char *timeStamp);

int extractDataSensorLog(FILE *source, int *labelID, char *sensorName, char*sensorValue, char *timeStamp);

#endif // DATAEXTRACTORUTILITIES_H
