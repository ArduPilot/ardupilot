/*
 
 
 Version: 1.0
 
 Authors:
       Swaroop Hangal
       Bharat Tak
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*************************************************************************
//        XBee SERIAL SETUP
*************************************************************************/

#include <AP_Math.h>
#include <AP_HAL.h>
#include <GCS_MAVLink.h>
#include "APM_Xbee.h"

#define highbyte(w) ((w) >> 8) 
#define lowbyte(w) ((w) & 0xff) 
#define highword(w) ((w) >> 16) 
#define lowword(w) ((w) & 0xffff) 

#define makelong(hi, low) (((int32_t)hi)<<16 | (low))
#define makeshort(hi, low) (((int16_t)hi)<<8 | (low))

#define AIRCRAFT_COUNT 3
#define NUM_API_PKTS 20

extern const AP_HAL::HAL& hal;

uint8_t flw_addr_high[AIRCRAFT_COUNT];
uint8_t flw_addr_low[AIRCRAFT_COUNT];



	/* For every interaircraft packet sent, a simple packet crc is calculated and sent with it
	// (inside the API packet). It serves the purpose of differentiating between different types of 
	// interaircraft packets(which have different crcs). This function adds the appropriate crc.
	*/
void Calc_pkt_crc(uint16_t data_sum, uint8_t *crc, uint8_t type){
    switch(type) {
        case 0:
	    *crc = (0xFF - data_sum + FOLLOWER_WP_CRC) & 0x000000FF;
	    break;
	case 1:
	    *crc = (0xFF - data_sum + ORBIT_CENTER_CRC) & 0x000000FF;
	    break;
	default:
	    *crc = 0x00;
	}
}


	/*Creates xbee API packet frm the provided Loc and adds crc based on type.
	// type 0 for waypoint
	// type 1 for orbit center
	*/
		
	/* The function that extracts the location data from the location struct 'loc', creates
	// an array of bytes that forms an API packet for xbee, and sends it to hal.uartC to be 
	// transmitted to the aircraft choosed as the first argument. The third argument tells whether
	// the information in 'loc' is for waypoint or orbit cenetr or anything else(They have different crc)
	*/
void XBee_sendFollower(const uint8_t flwid, const struct Location *loc, const uint8_t type){
  uint8_t flw_pkt[NUM_API_PKTS+4] = { 0 };
  uint16_t xb_sum;uint8_t xb_chksum; uint8_t i;uint8_t j;
  xb_chksum = 0;uint8_t crc=0x00;uint16_t data_sum=0;
  xb_sum = 0;
  uint8_t data[4] = { 0 };
  int dataint[4] = { 0 };
  float dataadjust = 0;
  
  flw_addr_high[0]=0x33;
  flw_addr_high[1]=0x44;
  flw_addr_high[2]=0x55;
  
  flw_addr_low[0]=0x33;
  flw_addr_low[1]=0x44;
  flw_addr_low[2]=0x55;
  
  flw_pkt[0]=0x7E;
  flw_pkt[1]=0x00;
  flw_pkt[2]=NUM_API_PKTS;
  flw_pkt[3]=0x01;
  flw_pkt[4]=0x01;
  flw_pkt[5]=flw_addr_high[flwid];
  flw_pkt[6]=flw_addr_low[flwid];
  flw_pkt[7]=0x00;
  flw_pkt[8]=0xAF;   //Universal interaircraft start tag1
  flw_pkt[9]=0xFB;   //Universal interaircraft start tag2 

int_to_bytes(lowword(loc->lat), highword(loc->lat), &data[0]);
data_sum=data_sum+data[0]+data[1]+data[2]+data[3];
  flw_pkt[10]=data[0];
  flw_pkt[11]=data[1];
  flw_pkt[12]=data[2];
  flw_pkt[13]=data[3];
  
int_to_bytes(lowword(loc->lng), highword(loc->lng), &data[0]);
data_sum=data_sum+data[0]+data[1]+data[2]+data[3];
  flw_pkt[14]=data[0];
  flw_pkt[15]=data[1];
  flw_pkt[16]=data[2];
  flw_pkt[17]=data[3];

int_to_bytes(lowword(loc->alt), highword(loc->alt), &data[0]); 
data_sum=data_sum+data[0]+data[1]+data[2]+data[3];
  flw_pkt[18]=data[0];
  flw_pkt[19]=data[1];
  flw_pkt[20]=data[2];
  flw_pkt[21]=data[3];  

Calc_pkt_crc(data_sum, &crc, type);
  flw_pkt[22]=crc;  

for(j=3;j<NUM_API_PKTS+3;j++) {
  xb_sum+=flw_pkt[j];
  }

  xb_chksum=(0xFF-xb_sum) & 0x000000FF;
  flw_pkt[23]=xb_chksum;

for(i=0;i<NUM_API_PKTS+4;i++) {
  hal.uartC->write(flw_pkt[i]);
  }
}



//for debug purposes, debug xbee address 6666
void Follower_print_recieved(uint8_t *data){
  
 uint8_t flw_pkt[NUM_API_PKTS+4] = { 0 };
  uint16_t xb_sum;uint8_t xb_chksum;
  uint8_t i;uint8_t j;uint8_t type =3;
  xb_chksum = 0;
  xb_sum = 0;
  
  flw_pkt[0]=0x7E;
  flw_pkt[1]=0x00;
  flw_pkt[2]=NUM_API_PKTS;
  flw_pkt[3]=0x01;
  flw_pkt[4]=0x01;
  flw_pkt[5]=0x66;
  flw_pkt[6]=0x66;
  flw_pkt[7]=0x00;
  flw_pkt[8]=0xAF;   
  flw_pkt[9]=0xFB;   
  flw_pkt[10]=data[0];
  flw_pkt[11]=data[1];
  flw_pkt[12]=data[2];
  flw_pkt[13]=data[3];
  flw_pkt[14]=data[4];
  flw_pkt[15]=data[5];
  flw_pkt[16]=data[6];
  flw_pkt[17]=data[7];
  flw_pkt[18]=data[8];
  flw_pkt[19]=data[9];
  flw_pkt[20]=data[10];
  flw_pkt[21]=data[11];
  flw_pkt[22]=0x00;  

  
for(j=3;j<NUM_API_PKTS+3;j++) {
  xb_sum+=flw_pkt[j];
  }

  xb_chksum=(0xFF-xb_sum) & 0x000000FF;
  flw_pkt[23]=xb_chksum;
 
  for(i=0;i<NUM_API_PKTS+4;i++) {
  hal.uartC->write(flw_pkt[i]);
  }
} 




using namespace std;

void int_to_bytes(int loword, int hiword, unsigned char *hexdata){
    //unsigned short int hiword, loword;
    unsigned char hihibyte, hilobyte, lohibyte, lolobyte;
    lolobyte = lowbyte(loword);
    lohibyte = highbyte(loword);

    hilobyte = lowbyte(hiword);
    hihibyte = highbyte(hiword);

    hexdata[0] = lolobyte;
    hexdata[1] = lohibyte;
    hexdata[2] = hilobyte;
    hexdata[3] = hihibyte;	
}

long bytes_to_int(uint8_t *hexdata){
	uint16_t hiword1, loword1;
	int32_t longval1=0;

	loword1 = makeshort(hexdata[1], hexdata[0]);
	hiword1 = makeshort(hexdata[3], hexdata[2]);
	longval1 = makelong(hiword1, loword1);
	return longval1;
}


/*************************************************************************
//        TRANSMIT PACKET STRUCTURE
*************************************************************************/
// Source Unknown

// Function 's' 'n' 'p' PT N d1 ... dN     CHK
// Byte      1   2   3  4  5 6  ... N+5  N+6 N+7
// RX Packet Description
// 1 - 3 Each received packet must begin with the three-byte (character) sequence
// "snp" to signal the beginning of a new packet.
// 4 PT specifies the packet type.
// 5 N specifies the number of data bytes to expect.
// 6 - (N+5) d1 through dN contain the N data bytes in the packet.
// (N+6) - (N+7) CHK is a two-byte checksum.

/*************************************************************************
//        API TRANSMIT PACKET STRUCTURE
*************************************************************************/
// NAME                Byte Num    Byte
// Start Delimiter        0        0x7E
// Length              MSB 1       0xXX     Number of bytes between the length and the checksum
//                     LSB 2       0xXX
// Frame Type             3        0x10
// Frame ID               4        0xXX     Identifies the UART data frame for the host to correlate
//                                          with a subsequent ACK (acknowledgement). If set to 0,
//                                          no response is sent.
// 64-bit Address         5        0xXX
// of Destination      MSB 6       0xXX    0x000000000000FFFF = Broadcast
//                        7        0xXX
//                        8        0xXX
//                        9        0xXX
//                       10        0xXX
//                     LSB 11      0xXX
// Reserved              12        0xFF
//                       13        0xFE
// Broadcast Radius      14
// Transmit Options      15        0x0X     bit 0: Diasble ACK    bit 1: Don't atempt route Discovery
// RF Data               16        0xXX
//                       17        0xXX
//                       ..        ....
//                       ..        ....
//Checksum               XX        0xXX     Checksum = 0xFF - (the 8 bit sum of bytes from offset 3 to this byte)
/*************************************************************************/


/*************************************************************************
//        API RECEIVE PACKET STRUCTURE
*************************************************************************/
// NAME                Byte Num    Byte
// Start Delimiter        0        0x7E
// Length              MSB 1       0xXX     Number of bytes between the length and the checksum
//                     LSB 2       0xXX
// Frame Type             3        0x90
// Frame ID               4        0xXX     Identifies the UART data frame for the host to correlate
//                                          with a subsequent ACK (acknowledgement). If set to 0,
//                                          no response is sent.
// 64-bit Source          5        0xXX
// Address of sender   MSB 6       0xXX
//                        7        0xXX
//                        8        0xXX
//                        9        0xXX
//                       10        0xXX
//                     LSB 11      0xXX
// Reserved              12        0xFF
//                       13        0xFE
// Receive Options       14        0x0X     0x01 - Packet Acknowledged  0x02 - Packet was a broadcast packet
// Received Data         15        0xXX
//                       16        0xXX
//                       ..        ....
//                       ..        ....
//Checksum               XX        0xXX     Checksum = 0xFF - (the 8 bit sum of bytes from offset 3 to this byte)
/*************************************************************************/



