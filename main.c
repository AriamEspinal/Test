/*----------------------------------------------------------------------
* (c) 2008 Microstrain, Inc.
*----------------------------------------------------------------------
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
* CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
* FOR THEM TO SAVE TIME. AS A RESULT, MICROSTRAIN SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY 
* CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY 
* CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH 
* THEIR PRODUCTS.
*---------------------------------------------------------------------*/

/*----------------------------------------------------------------------
* Read Acceleration and Angular Rate
*
* 9/12/2008	fpm added this comment
*
* This command line application demonstrates how to use the 3DM-GX2 SDK to 
* retrieve an Acceleration and Angular Rate data sample from a MicroStrain 
* orientation sensor.  The only argument is the port number for the sensor.
* This can be obtained by using the Windows Device Manager. Under the 
* "Ports (COM & LPT)" listing, the MicroStrain device will be listed as
* either a "CP210x USB to UART Bridge Controller" or a "MicroStrain Virtual
* COM Port".  If no port argument is specified, then a scanport function is 
* called which will look for an attached sensor by scanning the serial ports.
*----------------------------------------------------------------------*/

#include <stdio.h>
#include "ms_basic_type.h"
#include "i3dmgx2_Errors.h"
#include "i3dmgx2_Serial.h"
#include "i3dmgx2_Cmd.h"
#include "i3dmgx2_Utils.h"
#include "win_scanports.h"

/*--------------------------------------------------------------------*/
int main(int argc, char **argv) {

	s16 portNum;
	s16 deviceNum = 0;
	s16 i;
	u16 value=0;
	s16 id_flag = 0;
	s16 errorCode;
	s16 tryPortNum = 1;
	unsigned char Record[79];				//record returned from device read where max size is 79
	C2Accel_AngRecord	Accel_AngRecord;

	printf("\n   3DM-GX2 Read Acceleration and Angular Rate\n");

	/*-------- If user specifies a port, then use it */
	if (argc > 1) {
		tryPortNum = atoi(argv[1]);
		if (tryPortNum < 1 || tryPortNum > 256) {
			printf("   usage:  i3dmgx2 <portNumber>\n");
			printf("        valid ports are 1..256, default is 1\n");
			exit(1);
		}
	}
	/*-------- If no port specified, then scan ports to see if we can find a connected device */
	else {
		printf("\n   No port specified. Scanning for device...\n");
		tryPortNum = scanports();
		if (tryPortNum < 1 || tryPortNum > 256) {
			printf("\n   No recognized devices attached!\n");
			goto Exit;
		}
		else {
			printf("\n   Found device on COM port #%d\n", tryPortNum);
		}
	}

	/*-------- open a port, map a device */
	portNum = i3dmgx2_openPort(tryPortNum, 115200, 8, 0, 1, 1024, 1024);
	if (portNum<0) {
		printf("   port open failed.\n");
		printf("   Comm error %d, %s: ", portNum, explainError(portNum));
		goto Exit;
	}
	printf("\n   Using COM Port #%d \n", portNum);

	/*-------- Set Comm Timeout values */
	errorCode = setCommTimeouts(portNum, 1, 1); /* Read & Write timeout values */
	if (errorCode!=I3DMGX2_COMM_OK) {
		printf("   setCommTimeouts failed on port:%d with errorcode:%d\n",portNum,errorCode);
		goto Exit;
	} 

	/*-------- Disclose the byte order of host */
	if( TestByteOrder() !=BIG_ENDIAN)
		printf("   (Local Host is in Little Endian format)\n");
	else
		printf("   (Local Host is in Big Endian format)\n");
	printf("\n");  

	/*-------- 0xC2 Accel and Ang rate Output --- Accel x y z and Ang x y z */
	printf("\n   0xC2  Accel and Ang Output  \n");
	errorCode = i3dmgx2_AccelAndAngRate(portNum, &Record[0]);
	if (errorCode < 0)
		printf("   Error Accel and AngRate - : %s\n", explainError(errorCode));
	else{
		for (i=0; i<3; i++) {
			Accel_AngRecord.Accel[i] = FloatFromBytes(&Record[1 + i*4]);	// extract float from byte array
			Accel_AngRecord.AngRt[i] = FloatFromBytes(&Record[13 + i*4]);	// extract float from byte array
		}
		printf("\n\tAccel X\t\tAccel Y\t\tAccel Z\n");
		printf("  \t%f\t%f\t%f\n", Accel_AngRecord.Accel[0], Accel_AngRecord.Accel[1], Accel_AngRecord.Accel[2]);
		printf("\n\t  Ang X\t\t Ang Y\t\t Ang Z\n");
		printf("  \t%f\t%f\t%f\n", Accel_AngRecord.AngRt[0], Accel_AngRecord.AngRt[1], Accel_AngRecord.AngRt[2]);

		Accel_AngRecord.timer = convert2ulong(&Record[25]);
		printf("\n   Time Stamp: %u\n", Accel_AngRecord.timer);
	}

Exit:
	/*-------- close device */
	if (portNum >= 0)
		i3dmgx2_closeDevice(portNum);

	/*-------- wait for user to respond before exiting */
	printf("\nHit return to exit...\n");
	while (getchar() == EOF);
	return(0);
}
