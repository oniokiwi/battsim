#ifndef CURL_DOT_H
#define CURL_DOT_H

void  curl_sendPowerToDeliver(uint16_t power);
void  curl_sendReadings(const char* readings, int length);
void *curl_handler( void *ptr );

#endif
