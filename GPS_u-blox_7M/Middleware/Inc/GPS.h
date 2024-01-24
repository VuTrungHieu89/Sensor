/*
 * GPS.h
 *
 *  Created on: Jan 9, 2024
 *      Author: Admin
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_


#include "main.h"
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_hal_uart.h"
#include "gpio.h"
#include "usart.h"
#include "c_library_v2-master/common/mavlink.h"
#include "c_library_v2-master/mavlink_helpers.h"

#define GPS_DEBUG	0
#define GPSBUFSIZE  128       // GPS buffer size

typedef struct{
    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;

    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;
    char mode;
    float PDOP;
    float VDOP;
    float HDOP;
    int fix_type;
    float cog; // course of ground
    int ellipsoid;
} GPS_t;


void GPS_Init();
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
void Transmit_mavlink_data_GPS(mavlink_gps_raw_int_t data);


#endif /* INC_GPS_H_ */
