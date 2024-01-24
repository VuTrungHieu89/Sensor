/*
 * GPS.c
 *
 *  Created on: Jan 9, 2024
 *      Author: Admin
 */


#include "GPS.h"


uint8_t rx_data = 0;
uint8_t rx_buffer[GPSBUFSIZE];
uint8_t rx_index = 0;
uint8_t tx_buffer[GPSBUFSIZE];
mavlink_gps_raw_int_t data_GPS = {0};
GPS_t GPS;

/*
static void UART_Transmit_raw_data(){
	for (int i = 0; i < sizeof(rx_buffer); i++){
		tx_buffer[i] = rx_buffer[i];
	}
	HAL_UART_Transmit(&huart1, tx_buffer, sizeof(tx_buffer), 100);
}
*/
void GPS_Init(){
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

void GPS_UART_CallBack(){
	if (rx_data != '\n' && rx_data != '\n' && rx_index < sizeof(rx_buffer)) {
			rx_buffer[rx_index++] = rx_data;
	}
	else {
		//UART_Transmit_raw_data();
		if(GPS_validate((char*) rx_buffer))
			GPS_parse((char*) rx_buffer);
		rx_index = 0;
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
	Transmit_mavlink_data_GPS(data_GPS);
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	if(huart->Instance == USART1) GPS_UART_CallBack();
}

int GPS_validate(char *nmeastr){
    char check[3];
    char checkcalcstr[3];
    int i;
    int calculated_check;

    i=0;
    calculated_check=0;

    // check to ensure that the string starts with a $
    if(nmeastr[i] == '$')
        i++;
    else
        return 0;

    //No NULL reached, 75 char largest possible NMEA message, no '*' reached
    while((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75)){
        calculated_check ^= nmeastr[i];// calculate the checksum
        i++;
    }

    if(i >= 75){
        return 0;// the string was too long so return an error
    }

    if (nmeastr[i] == '*'){
        check[0] = nmeastr[i+1];    //put hex chars in check string
        check[1] = nmeastr[i+2];
        check[2] = 0;
    }
    else
        return 0;// no checksum separator found there for invalid

    sprintf(checkcalcstr,"%02X",calculated_check);
    return((checkcalcstr[0] == check[0])
        && (checkcalcstr[1] == check[1])) ? 1 : 0 ;
}

void GPS_parse(char *GPSstrParse){
    if(!strncmp(GPSstrParse, "$GPGGA", 6)){
    	if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c,%d", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.lock, &GPS.satelites, &GPS.hdop, &GPS.msl_altitude, &GPS.msl_units, &GPS.ellipsoid) >= 1){
    		GPS.dec_latitude = GPS_nmea_to_dec(GPS.nmea_latitude, GPS.ns);
    		GPS.dec_longitude = GPS_nmea_to_dec(GPS.nmea_longitude, GPS.ew);
    		return;
    	}
    }
    else if (!strncmp(GPSstrParse, "$GPRMC", 6)){
    	if(sscanf(GPSstrParse, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &GPS.utc_time, &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.speed_k, &GPS.course_d, &GPS.date) >= 1)
    		return;

    }
    else if (!strncmp(GPSstrParse, "$GPGLL", 6)){
        if(sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &GPS.nmea_latitude, &GPS.ns, &GPS.nmea_longitude, &GPS.ew, &GPS.utc_time, &GPS.gll_status) >= 1)
            return;
    }
    else if (!strncmp(GPSstrParse, "$GPVTG", 6)){
        if(sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &GPS.course_t, &GPS.course_t_unit, &GPS.course_m, &GPS.course_m_unit, &GPS.speed_k, &GPS.speed_k_unit, &GPS.speed_km, &GPS.speed_km_unit) >= 1)
            return;
    }
    else if (!strncmp(GPSstrParse, "GPGSA", 6)){
    	int a[12];
        if(sscanf(GPSstrParse, "$GPGSA,%c,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f", &GPS.mode, &GPS.fix_type,&a[0],&a[1],&a[2],&a[3],&a[4],&a[5],&a[6],&a[7],&a[8],&a[9],&a[10],&a[11], &GPS.PDOP, &GPS.HDOP, &GPS.VDOP) >= 1)
            return;
    }
    else if (!strncmp(GPSstrParse, "$GPRMA", 6)){
        if(sscanf(GPSstrParse, "$GPRMA,%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,],%*[^,], %f", &GPS.cog) >= 1)
            return;
    }

}

float GPS_nmea_to_dec(float deg_coord, char nsew) {
    int degree = (int)(deg_coord/100);
    float minutes = deg_coord - degree*100;
    float dec_deg = minutes / 60;
    float decimal = degree + dec_deg;
    if (nsew == 'S' || nsew == 'W') { // return negative
        decimal *= -1;
    }
    return decimal;
}
void Transmit_mavlink_data_GPS(mavlink_gps_raw_int_t data){
	mavlink_message_t msg;
	uint8_t _buffer[100] = {0};
	data.time_usec = GPS.utc_time;
	data.alt = GPS.altitude_ft;
	data.alt_ellipsoid = GPS.ellipsoid;
	data.lat = GPS.dec_latitude;
	data.lon = GPS.dec_longitude;
	data.eph = GPS.HDOP * 100;
	data.epv = GPS.VDOP * 100;
	data.fix_type = GPS.fix_type;
	data.vel = GPS.speed_km * 0.277778;
	data.cog = GPS.cog;
	data.satellites_visible = GPS.satelites;
	data.alt_ellipsoid = 0;
	data.h_acc = 1;
	data.v_acc = 1;
	data.vel_acc = 1;
	data.hdg_acc = 1;
	data.yaw = 0;
	mavlink_msg_gps_raw_int_encode(0, 1, &msg, &data);
	uint8_t len = mavlink_msg_to_send_buffer(_buffer, &msg);
	HAL_UART_Transmit(&huart1, _buffer, len, 100);
}
