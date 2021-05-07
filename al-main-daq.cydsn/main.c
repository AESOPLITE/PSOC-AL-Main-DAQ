/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/


#include "project.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "errno.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
//#define WRAPINC(a,b) (((a)>=(b-1))?(0):(a + 1))
#define WRAPINC(a,b) ((a + 1) % (b))
#define WRAP3INC(a,b) ((a + 3) % (b))
#define WRAP(a,b) ((a) % (b)) //Macro to bring new calculated index a into the bounds of a circular buffer of size b

// From LROA103.ASM
//;The format for the serial command is:
//; S1234<sp>xyWS1234<sp>xyWS1234<sp>xyW<cr><lf>
//; where 1234 is an ASCII encoded 16 bit command, with the format:
//; Data Byte for boards:	1 = MSB high nibble, 2 = MSB low nibble
//; Address Byte for boards: 3 = LSB high nibble, 4 = LSB low nibble
//; S & W are literal format characters,<sp> = space, xy = CIP address (ignored).
//; 9 characters are repeated 3 times followed by a carriage return - line feed,
//; and all alpha characters must be capitalized, baud rate = 1200.
#define START_COMMAND	(uint8*)("S") //Start command string before the 4 command char 
#define START_COMMAND_SIZE	1u //Size of Start command string before the 4 command char 
#define END_COMMAND	(uint8*)(" 01W") //End command string after the 4 command char, CIP is 01 which is ignored
#define END_COMMAND_SIZE	 4u //Size of End command string after the 4 command char, CIP is 01 which is ignored
#define CR	(0x0Du) //Carriage return in hex
#define LF	(0x0Au) //Line feed in hex
#define DLE	(0x10u) //Data Link Escape Used as low rate packet header
#define ETX	(0x03u) //Data Link Escape Used as low rate packet trailer
#define CMD_ID	(0x14u) //ID byte for command in low rate packet
#define REQ_ID	(0x13u) //ID byte for request science data in low rate packet
#define SDATA_ID	(0x53u) //ID byte for science data in low rate packet
#define FILLBYTE (0xA3u) //SPI never transmits  so could be anything
//#define CMDBUFFSIZE 3
/* Project Defines */
#define FALSE  0
#define TRUE   1
//#define SPI_BUFFER_SIZE  (512u)
#define SPI_BUFFER_SIZE  (1024u)
#define EV_BUFFER_SIZE  (512u)
typedef uint16 SPIBufferIndex; //type of variable indexing the SPI buffer. should be uint8 or uint16 based on size
typedef uint16 EvBufferIndex; //type of variable indexing the Event buffer. should be uint16
//uint8 cmdBuff[CMDBUFFSIZE];
//uint8 iCmdBuff = CMDBUFFSIZE - 1;

#define USBFS_DEVICE	(0u)
/* The buffer size is equal to the maximum packet size of the IN and OUT bulk
* endpoints.
*/
#define USBUART_BUFFER_SIZE	(64u)
#define LINE_STR_LENGTH	(20u)

//#define NUM_SPI_DEV	(5u)
#define NUM_SPI_DEV	(1u)
uint8 iSPIDev = 0u;
//uint8 frameSPIDev = 0u;
#define POW_SEL		(0x01u)
#define PHA_SEL		(0x02u)
#define CTR1_SEL	(0x03u)
#define TKR_SEL		(0x0Bu)
#define CTR3_SEL	(0x0Cu)
//const uint8 tabSPISel[NUM_SPI_DEV] = {POW_SEL, PHA_SEL, CTR1_SEL, TKR_SEL, CTR3_SEL};
//const uint8 tabSPISel[NUM_SPI_DEV] = {0, 0, CTR1_SEL, 0, CTR3_SEL};
//const uint8 tabSPISel[NUM_SPI_DEV] = {0, 0, 0, 0, 0}; //DEBUG
const void (* tabSPISel[NUM_SPI_DEV])(uint8) = {
    Pin_Sel_Pwr_Write}; //debug
//    Pin_Sel_PHA_Write,
//    Pin_Sel_Ctr1_Write,
//    Pin_Sel_Tkr_Write,
//    Pin_Sel_Ctr3_Write}; //function pointers to write to the pins for diffent select lines
#define NULL_HEAD	(0xF9u)
#define POW_HEAD	(0xF6u)
#define PHA_HEAD	(0xF3u)
#define CTR1_HEAD	(0xF8u)
#define TKR_HEAD	(0xF4u)
#define CTR3_HEAD	(0xFAu)
#define EOR_HEAD	(0xFFu)
#define DUMP_HEAD	(0xF5u)
#define ENDDUMP_HEAD	(0xF7u)
const uint8 tabSPIHead[NUM_SPI_DEV] = {POW_HEAD}; //debug, PHA_HEAD, CTR1_HEAD, TKR_HEAD, CTR3_HEAD};
const uint8 frame00FF[2] = {0x00u, 0xFFu};
uint8 buffSPI[NUM_SPI_DEV][SPI_BUFFER_SIZE];
SPIBufferIndex buffSPIRead[NUM_SPI_DEV];
SPIBufferIndex buffSPIWrite[NUM_SPI_DEV];
SPIBufferIndex buffSPICurHead[NUM_SPI_DEV]; //Header of the current packet
SPIBufferIndex buffSPICompleteHead[NUM_SPI_DEV]; //Header of the latest complete packet
uint8 buffEv[SPI_BUFFER_SIZE];
EvBufferIndex buffEvRead;
EvBufferIndex buffEvWrite;

enum readStatus {CHECKDATA, READOUTDATA, EORFOUND, EORERROR};
enum commandStatus {WAIT_DLE, CHECK_ID, CHECK_LEN, READ_CMD, CHECK_ETX_CMD, CHECK_ETX_REQ};
#define COMMAND_SOURCES 3
enum commandStatus commandStatusC[COMMAND_SOURCES];
uint8 commandLenC[COMMAND_SOURCES];
uint8 cmdRxC[COMMAND_SOURCES][2];
#define COMMAND_CHARS	(4u)
uint8 curCmd[COMMAND_CHARS+1]; //one extra char for null
uint8 iCurCmd = 0u;
//volatile uint8 timeoutDrdy = FALSE;
//volatile uint8 lastDrdyCap = 0u;
//#define MIN_DRDY_CYCLES 4 //8 //might need  Fster clock since the master clock generates noise the noise on this line
 
const uint8 frameSync[2] = {0x55u, 0xABu};
uint32 frameCnt = 0u;
#define DATA_BYTES_FRAME 27

typedef struct PacketLocation {
	SPIBufferIndex index;
	SPIBufferIndex header;
	SPIBufferIndex EOR;
} PacketLocation;

#define PACKET_FIFO_SIZE	 (16u * NUM_SPI_DEV)
PacketLocation packetFIFO[PACKET_FIFO_SIZE];
uint8 packetFIFOHead = 0u;
uint8 packetFIFOTail = 0u;

uint8 buffUsbTx[USBUART_BUFFER_SIZE];
uint8 iBuffUsbTx = 0;
uint8 buffUsbTxDebug[USBUART_BUFFER_SIZE];
uint8 iBuffUsbTxDebug = 0;

#define FRAME_DATA_BYTES	(27u)
#define FRAME_BUFFER_SIZE	(64u)
uint8 buffFrameData[FRAME_BUFFER_SIZE][FRAME_DATA_BYTES];
uint8 buffFrameDataRead = 0;
uint8 buffFrameDataWrite = 0;


#define COUNTER_PACKET_BYTES	(45u)

/* Defines for DMA_LR_Cmd_1 */
#define DMA_LR_Cmd_1_BYTES_PER_BURST 1
#define DMA_LR_Cmd_1_REQUEST_PER_BURST 1
#define DMA_LR_Cmd_1_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_LR_Cmd_1_DST_BASE (CYDEV_SRAM_BASE)
#define DMA_LR_Cmd_1_BUFFER_SIZE 16
uint8 buffCmdRxC[COMMAND_SOURCES][DMA_LR_Cmd_1_BUFFER_SIZE];
reg16 * buffCmdRxCWritePtr[COMMAND_SOURCES];
uint8 buffCmdRxCRead[COMMAND_SOURCES];



//const uint8 continueReadFlags = (SPIM_BP_STS_SPI_IDLE | SPIM_BP_STS_TX_FIFO_EMPTY);
volatile uint8 continueRead = FALSE;


//;AESOPLite Initialization Commands
//HiVol	FDB	$A735  ;T1 1431.6 High Voltage
//	FDB	$DD36  ;T2 1860.7
//	FDB	$CA37  ;T3 1704.7
//	FDB	$B9B5  ;T4 1553.3
//	FDB	$CB74  ;G  1706.8
//DiscP	FDB	$0039  ;Dual PHA card 0, All PHA Discriminators set to 7.0
//	FDB	$073A  ;T1
//	FDB	$0039  ;Dual PHA card 0
//	FDB	$0778  ;T2
//	FDB	$0139  ;Dual PHA card 1
//	FDB	$073A  ;T3
//	FDB	$0139  ;Dual PHA card 1
//	FDB	$0778  ;T4
//	FDB	$0239  ;Dual PHA card 2
//	FDB	$073A  ;G	
//	FDB	$0239  ;Dual PHA card 2
//	FDB	$0778  ;No Input
//DiscL	FDB	$0039  ;Dual PHA card 0, All Logic Discriminators set to 7.0
//	FDB	$073B  ;T1
//	FDB	$0039  ;Dual PHA card 0
//	FDB	$0779  ;T2
//	FDB	$0139  ;Dual PHA card 1
//	FDB	$073B  ;T3
//	FDB	$0139  ;Dual PHA card 1
//	FDB	$0779  ;T4
//	FDB	$0239  ;Dual PHA card 2
//	FDB	$073B  ;G
//	FDB	$0239  ;Dual PHA card 2
//	FDB	$0779  ;No Input
//Coinc	FDB	$F838  ;T1 T2 T3 Coincidence
//	FDB	$0AB7  ;10sec counter R/O
//	FDB	$0AB6  ;10sec Power R/O

//AESOPLite Initialization Commands
#define NUMBER_INIT_CMDS	(32 + 39)
uint8 initCmd[NUMBER_INIT_CMDS][2] = {
	{0xA7, 0x35}, //T1 1431.6 High Voltage
	{0xDD, 0x36}, //T2 1860.7
	{0xCA, 0x37}, //T3 1704.7
	{0xB9, 0xB5}, //T4 1553.3
	{0xCB, 0x74}, //G  1706.8
	{0x00, 0x39}, //Dual PHA card 0, All PHA Discriminators set to 7.0
	{0x07, 0x3A}, //T1
	{0x00, 0x39}, //Dual PHA card 0
	{0x07, 0x78}, //T2
	{0x01, 0x39}, //Dual PHA card 1
	{0x07, 0x3A}, //T3
	{0x01, 0x39}, //Dual PHA card 1
	{0x07, 0x78}, //T4
	{0x02, 0x39}, //Dual PHA card 2
	{0x07, 0x3A}, //G	
	{0x02, 0x39}, //Dual PHA card 2
	{0x07, 0x78}, //No Input
	{0x00, 0x39}, //Dual PHA card 0, All Logic Discriminators set to 7.0
	{0x07, 0x3B}, //T1
	{0x00, 0x39}, //Dual PHA card 0
	{0x07, 0x79}, //T2
	{0x01, 0x39}, //Dual PHA card 1
	{0x07, 0x3B}, //T3
	{0x01, 0x39}, //Dual PHA card 1
	{0x07, 0x79}, //T4
	{0x02, 0x39}, //Dual PHA card 2
	{0x07, 0x3B}, //G
	{0x02, 0x39}, //Dual PHA card 2
	{0x07, 0x79}, //No Input
	{0xF8, 0x38}, //T1 T2 T3 Coincidence
	{0x0A, 0xB7}, //10sec counter R/O
	{0x0A, 0xB6},  //10sec Power R/O
    // event PSOC DAC Trigger Setup
	{0x04, 0x23},  //Header for ToF DAC
	{0x01, 0x21},  //Channel ToF 1 
	{0x00, 0x22},  //DAC Byte MSB
	{0x40, 0x23},  //DAC Byte LSB
    {0x04, 0x23},  //Header for ToF DAC
	{0x02, 0x21},  //Channel ToF 2
	{0x00, 0x22},  //DAC Byte MSB
	{0x40, 0x23},  //DAC Byte LSB
    {0x01, 0x23},  //Header for DAC Threshold Set
	{0x05, 0x21},  //Channel 5
	{0x00, 0x22},  //DAC Byte MSB
	{0x3C, 0x23},  //DAC Byte LSB
    {0x01, 0x22},  //Header for DAC Threshold Set
	{0x01, 0x21},  //Channel 1 
	{0x14, 0x22},  //DAC Byte
    {0x01, 0x22},  //Header for DAC Threshold Set
	{0x02, 0x21},  //Channel 2 
	{0x14, 0x22},  //DAC Byte
    {0x01, 0x22},  //Header for DAC Threshold Set
	{0x03, 0x21},  //Channel 3 
	{0x14, 0x22},  //DAC Byte
    {0x01, 0x22},  //Header for DAC Threshold Set
	{0x04, 0x21},  //Channel 4 
	{0x14, 0x22},  //DAC Byte    
    {0x36, 0x22},  //Header for Trigger Mask Set
	{0x01, 0x21},  //Mask 1 
	{0x0D, 0x22},  //Trigger Mask 0D T3
    {0x39, 0x22},  //Header for Trigger Prescale Set
    {0x02, 0x21},  //PMT
	{0x04, 0x22},  //Prescale by 4 
    {0x3A, 0x21},  //Header for Trigger Window Set
    {0x18, 0x21},  //Trigger Window Data
    {0x36, 0x22},  //Header for Trigger Mask Set
    {0x00, 0x21},  //Mask 2 
	{0x00, 0x22},  //Trigger Mask 00 none
    {0x30, 0x21},  //Header for Output Mode Set
	{0x00, 0x21},  //0 SPI output 
    {0x3B, 0x21},  //Header Trigger Enable Set
//	{0x01, 0x21},  //Trigger Enabled
	{0x03, 0x20},  //Read Errors DEBUG
    }; //End init cmds
#define CMD_BUFFER_SIZE (NUMBER_INIT_CMDS + NUMBER_INIT_CMDS)
uint8 buffCmd[COMMAND_SOURCES][CMD_BUFFER_SIZE][2];
uint8 readBuffCmd[COMMAND_SOURCES];// = 0;
uint8 writeBuffCmd[COMMAND_SOURCES];// = 0;
uint8 orderBuffCmd[COMMAND_SOURCES];

typedef struct BaroCoeff {
	const double U0;
	const double Y1;
	const double Y2;
	const double Y3;
	const double C1;
	const double C2;
	const double C3;
	const double D1;
	const double D2;
	const double T1;
	const double T2;
	const double T3;
	const double T4;
	const double T5;
} BaroCoEff;

#define BARO_COUNT_TO_US (12)
#define NUM_BARO 2
#define NUM_BARO_CAPTURES 4

uint16 buffBaroCap[NUM_BARO *2][NUM_BARO_CAPTURES];
uint8 buffBaroCapRead[NUM_BARO];
uint8 buffBaroCapWrite[NUM_BARO];

//const BaroCoEff baroCE[NUM_BARO] = {{.U0 = 1.0, .Y1 = 1.0, .Y2 = 1.0, .Y3 = 1.0, .C1 = 1.0, .C2 = 1.0, .C3 = 1.0, .D1 = 1.0, .D2 = 1.0, .T1 = 1.0, .T2 = 1.0, .T3 = 1.0, .T4 = 1.0, .T5 = 1.0 }};
const BaroCoEff baroCE[NUM_BARO] = {{.U0 = 5.875516, .Y1 = -3947.926, .Y2 = -10090.9, .Y3 = 0.0, .C1 = 95.4503, .C2 = 2.982818, .C3 = -135.3036, .D1 = 0.042247, .D2 = 0.0, .T1 = 27.91302, .T2 = 0.873949, .T3 = 21.00155, .T4 = 36.63574, .T5 = 0.0 }};
double curBaroTemp[NUM_BARO];
double curBaroPres[NUM_BARO];
uint32 curBaroTempCnt[NUM_BARO];
uint32 curBaroPresCnt[NUM_BARO];
uint32 baroReadReady = 0u;

uint8 loopCount = 0;
uint8 loopCountCheck = 0;
#define SELECT_HIGH_LOOPS 250

double BaroTempCalc ( double U, const BaroCoEff * bce )
{
	return (((bce->Y1) * U) + ((bce->Y2) * pow(U, 2))  + ((bce->Y3) * pow(U, 3)));
}

double BaroPresCalc ( double Tao, double U, const BaroCoEff * bce )
{
	double Usq = pow( U, 2);
	double C = ((bce->C1) + ((bce->C2) * U) + ((bce->C3) * Usq)); 
	double D = ((bce->D1) + ((bce->D2) * U)); 
	double T0 = ((bce->T1) + ((bce->T2) * U) + ((bce->T3) * Usq) + ((bce->T4) * (U * Usq)) + ((bce->T5) * (Usq * Usq))); 
	double ratio = (1 - (pow(T0, 2) / pow(Tao, 2)));
	return ((C * ratio) * (1 - (D * ratio)));
}

/*******************************************************************************
* Function Name: CmdBytes2String
********************************************************************************
*
* Summary:
*  Converts a 2 byte command in binary to a 4 byte ASCII representation of that 
*  command (null terminator is the 5th byte).  
*
* Parameters:
*  in:  uint8 pointer to 2 bytes to be converted 
*  out: uint8 pointer to 5 byte null terminted string of the result of 
*  2byte command converted to capitalized ASCII hexadecimal characters   
*  
* Return:
*  int number of charaters returned. Should be 4 on success, negative on fault
*
*******************************************************************************/
int CmdBytes2String (uint8* in, uint8* out)
{
    if ((NULL == in) || (NULL == (in + 1)) || (NULL == out)) //check for null pointers
    {
        return -EFAULT; //null pointer error, sprint might also do this
    }
	return sprintf((char*)out, "%02X%02X", *(in), *(in + 1)); //converts the 2 bytes to hex with leading zerosv
}

int SendCmdString (uint8 * in)
{
	if (0 != UART_Cmd_GetTxBufferSize()) return -EBUSY; // Not ready to send 
//	if (convert2Ascii) sprintf((char *)curCmd, "%x%x", (char)(*in), (char)*(in+1));
	for (uint8 x=0; x<3; x++)
	{
		UART_Cmd_PutArray(START_COMMAND, START_COMMAND_SIZE);
		UART_Cmd_PutArray(in, COMMAND_CHARS);
		UART_Cmd_PutArray(END_COMMAND, END_COMMAND_SIZE);
	}
	//Unix style line end
	UART_Cmd_PutChar(CR);
	UART_Cmd_PutChar(LF);
    //Debug
    if (USBUART_CD_CDCIsReady())
    {
        *(in+4) = LF;
        USBUART_CD_PutData(in, COMMAND_CHARS +1);

    }
	return 0;
}

void SendInitCmds()
{
	int i = 0;
	CyDelay(7000); //7 sec delay for boards to init TODO Debug
	while (i < NUMBER_INIT_CMDS)
	{
        int8 convResult = CmdBytes2String(initCmd[i], curCmd);
        if (4 == convResult)
        {
		    if (0 == SendCmdString(curCmd)) i++; //, TRUE)) i++;
        }
        else 
        {
            //TODO error handling and counting
            i++;   
        }
//        if (i > 24)
//        {
//            memcpy(buffUsbTxDebug, curCmd, COMMAND_CHARS);
//        	iBuffUsbTxDebug += 4;
//            buffUsbTxDebug[iBuffUsbTxDebug++] = '\n';
//        }
//		CyDelay(1000); //TODO Debug
	}
}

int SendLRScienceData()
{
    //TODO collect the subset of data 
    buffUsbTx[iBuffUsbTx++] = DLE;
    buffUsbTx[iBuffUsbTx++] = SDATA_ID;
    buffUsbTx[iBuffUsbTx++] = 1;
    buffUsbTx[iBuffUsbTx++] = 0;
    buffUsbTx[iBuffUsbTx++] = ETX;
    
    return 1;
}

int ParseCmdInputByte(uint8 tempRx, uint8 i)
{
    switch(commandStatusC[i])
    {
        case WAIT_DLE:
            if (DLE == tempRx) commandStatusC[i] = CHECK_ID;
            break;
        case CHECK_ID:
            if (CMD_ID == tempRx) commandStatusC[i] = CHECK_LEN;
            else if (REQ_ID == tempRx) commandStatusC[i] = CHECK_ETX_REQ;
            break;
        case CHECK_LEN:
            if(2 == tempRx){
                commandLenC[i] = tempRx;
                commandStatusC[i] = READ_CMD;
            }
            else commandStatusC[i] = WAIT_DLE;
            break;
        case READ_CMD:
            if(commandLenC[i] > 0)
            {
                cmdRxC[i][commandLenC[i] % 2] = tempRx;
                commandLenC[i]--;
//                        buffUsbTxDebug[iBuffUsbTxDebug++] = commandLenC[i]; //debug
                if(0 == commandLenC[i])  commandStatusC[i]= CHECK_ETX_CMD;
            }
            
            break;
        case CHECK_ETX_CMD:
            if (ETX == tempRx)
            {
                
                int tempRes = CmdBytes2String(cmdRxC[i], curCmd);
                if(tempRes >= 0)
                {
                    tempRes = SendCmdString(curCmd);  
                    if (-EBUSY == tempRes)
                    {
                        memcpy(buffCmd[i][writeBuffCmd[i]], cmdRxC[i], 2); //busy queue for later
                        writeBuffCmd[i] = WRAPINC(writeBuffCmd[i], CMD_BUFFER_SIZE);
                    }
                    else if (tempRes < 0)
                    {
                        //TODO Error handling
                    }
                }
            }
            else 
            {
                //TODO error
            }
            commandStatusC[i] = WAIT_DLE;
            break;
        case CHECK_ETX_REQ:
            if (ETX == tempRx)
            {
                SendLRScienceData();
            }
            else 
            {
                //TODO error
            }
            commandStatusC[i] = WAIT_DLE;
            break;
    }
    return 0;
}

int CheckCmdDma(uint8 chanSrc)
{
   
    uint8 tempRx;
    int16 buffNewReadLen = *buffCmdRxCWritePtr[0] - LO16((uint32)buffCmdRxC[chanSrc]);
//    buffUsbTxDebug[iBuffUsbTxDebug++] = buffNewReadLen & 255; //debug
    buffNewReadLen -= buffCmdRxCRead[chanSrc];
    if (buffNewReadLen < 0) buffNewReadLen += DMA_LR_Cmd_1_BUFFER_SIZE;
//    buffUsbTxDebug[iBuffUsbTxDebug++] = buffNewReadLen & 255; //debug
    
    if(TRUE)
    {
        
        while(buffNewReadLen-- > 1)   
        {
            buffUsbTxDebug[iBuffUsbTxDebug++] = buffNewReadLen & 255; //debug
            buffCmdRxCRead[chanSrc] = WRAPINC(buffCmdRxCRead[chanSrc], DMA_LR_Cmd_1_BUFFER_SIZE);
            tempRx = buffCmdRxC[chanSrc][buffCmdRxCRead[chanSrc]];  
            buffUsbTxDebug[iBuffUsbTxDebug++] = tempRx; //debug
            switch(commandStatusC[chanSrc])
            {
                case WAIT_DLE:
                    if (DLE == tempRx) commandStatusC[chanSrc] = CHECK_ID;
                    break;
                case CHECK_ID:
                    if (CMD_ID == tempRx) commandStatusC[chanSrc] = CHECK_LEN;
                    if (REQ_ID == tempRx) commandStatusC[chanSrc] = CHECK_ETX_REQ;
                    break;
                case CHECK_LEN:
                    if(2 == tempRx){
                        commandLenC[chanSrc] = tempRx;
                        commandStatusC[chanSrc] = READ_CMD;
                    }
                    else commandStatusC[chanSrc] = WAIT_DLE;
                    break;
                case READ_CMD:
                    if(commandLenC[0] > 0)
                    {
                        cmdRxC[chanSrc][commandLenC[chanSrc] % 2] = tempRx;
                        commandLenC[0]--;
//                        buffUsbTxDebug[iBuffUsbTxDebug++] = commandLenC[0]; //debug
                        if(0 == commandLenC[chanSrc])  commandStatusC[chanSrc]= CHECK_ETX_CMD;
                    }
                    
                    break;
                case CHECK_ETX_CMD:
                    if (ETX == tempRx)
                    {
                        
                        int tempRes = CmdBytes2String(cmdRxC[chanSrc], curCmd);
                        if(tempRes >= 0)
                        {
                            tempRes = SendCmdString(curCmd);  
                            if (-EBUSY == tempRes)
                            {
                                memcpy(buffCmd[chanSrc][writeBuffCmd[chanSrc]], cmdRxC[chanSrc], 2); //busy queue for later
                                writeBuffCmd[chanSrc] = WRAPINC(writeBuffCmd[chanSrc], CMD_BUFFER_SIZE);
                            }
                            else if (tempRes < 0)
                            {
                                //TODO Error handling
                            }
                        }
                    }
                    else 
                    {
                        //TODO error
                    }
                    commandStatusC[chanSrc] = WAIT_DLE;
                    break;
                case CHECK_ETX_REQ:
                    if (ETX == tempRx)
                    {    
                        SendLRScienceData();
                    }
                    break;
            }
                
        }
    }
    return 0;
}

int CheckCmdBuffers()
{
    if (0 != UART_Cmd_GetTxBufferSize()) return -EBUSY; // Not ready to send
    uint8 curChan;
    for (uint8 i = 0; i < COMMAND_SOURCES; i++) 
    {
        curChan = orderBuffCmd[i];
        if (readBuffCmd[curChan] != writeBuffCmd[curChan]) // check if q has cmd
        {
            int tempRes = CmdBytes2String(buffCmd[curChan][readBuffCmd[curChan]], curCmd);
            tempRes = SendCmdString(curCmd);
            readBuffCmd[curChan] = WRAPINC(readBuffCmd[curChan], CMD_BUFFER_SIZE);
            return 1;
        }
    }
    return 0;
}



CY_ISR(ISRCheckCmd)
{
    uint8 intState = CyEnterCriticalSection();
    uint8 tempStatus1 = UART_LR_Cmd_1_ReadRxStatus();
    uint8 tempStatus2 = UART_LR_Cmd_2_ReadRxStatus();
    uint8 tempRx;
    uint8 i = 0;
//    buffUsbTxDebug[iBuffUsbTxDebug++] = UART_LR_Cmd_1_GetRxBufferSize(); //debug
    if((tempStatus1 | UART_LR_Cmd_1_RX_STS_FIFO_NOTEMPTY) > 0)
    {
        
        while(UART_LR_Cmd_1_GetRxBufferSize())   
        {
            int tempRes = ParseCmdInputByte(UART_LR_Cmd_1_ReadRxData(), i);
            if (0 > tempRes)
            {
                //TODO error handling
            }
            
                
        }
    }
    
    i=1;
    if((tempStatus1 | UART_LR_Cmd_2_RX_STS_FIFO_NOTEMPTY) > 0)
    {
        
        while(UART_LR_Cmd_2_GetRxBufferSize())   
        {
            int tempRes = ParseCmdInputByte(UART_LR_Cmd_2_ReadRxData(), i);
            if (0 > tempRes)
            {
                //TODO error handling
            }
            
                
        }
    
    }
    
    CyExitCriticalSection(intState);
}

CY_ISR(ISRReadSPI)
{
//	if (iCmdBuff < CMDBUFFSIZE - 1)
//	{
//		SPIM_BP_WriteTxData(cmdBuff[iCmdBuff++]);
//	}
//	else if (!Pin_nDrdy_Read())
//	{
//		iCmdBuff = CMDBUFFSIZE - 1;
//		SPIM_BP_WriteTxData(cmdBuff[iCmdBuff]);
//	}
//	uint8 tempStatus = SPIM_BP_ReadStatus();
	uint8 intState = CyEnterCriticalSection();

	uint8 tempnDrdy = Pin_nDrdy_Filter_Read();
	SPIBufferIndex tempBuffWrite = buffSPIWrite[iSPIDev];
	uint8 tempStatus = SPIM_BP_ReadStatus();
//	Control_Reg_LoadPulse_Write(0x01);
    (*tabSPISel[iSPIDev])(0u);//select low for a peeriod of time
    Timer_SelLow_Start();
    continueRead = TRUE;
	if (tempBuffWrite != buffSPICurHead[iSPIDev]) //Check if buffer is full
	{
		buffSPIWrite[iSPIDev] = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
		 //if ((0u == Pin_nDrdy_Read()) && (0u != (SPIM_BP_TX_STATUS_REG & SPIM_BP_STS_TX_FIFO_EMPTY)) && (buffSPIWrite[iSPIDev] != buffSPIRead[iSPIDev]))
//	    uint8 tempnDrdy = Pin_nDrdy_Filter_Read(); //placed here in hopes the glith filter can change to 1 at end of data
		if ((0u != tempnDrdy) || ((WRAP3INC(buffSPIWrite[iSPIDev], SPI_BUFFER_SIZE)) == buffSPIRead[iSPIDev]))
//		if ((buffSPIWrite[iSPIDev] == buffSPIRead[iSPIDev]))
		{
			continueRead = FALSE;
//			Control_Reg_CD_Write(0x00u);
//			SPIM_BP_ClearTxBuffer();
		}
//		else 
//		{
//			Control_Reg_CD_Write(0x02u);
//			Timer_SelLow_Start();
//			if (0u != (SPIM_BP_STS_TX_FIFO_EMPTY & tempStatus))
//			{
//				SPIM_BP_WriteTxData(FILLBYTE);
//			}
//		}
//		tempStatus = SPIM_BP_ReadStatus();
		if (0u != (SPIM_BP_STS_RX_FIFO_NOT_EMPTY & tempStatus))
		{
			buffSPI[iSPIDev][tempBuffWrite] = SPIM_BP_ReadRxData();
		}
	   
		
	}
	else 
	{
//		Control_Reg_CD_Write(0x00u);
        continueRead = FALSE;
//		SPIM_BP_ClearTxBuffer();
//		tempStatus = SPIM_BP_ReadStatus();
	}
	
	CyExitCriticalSection(intState);
}
CY_ISR(ISRWriteSPI)
{
    uint8 intState = CyEnterCriticalSection();
	uint8 tempStatus = Timer_SelLow_ReadStatusRegister();
//	if (0u != (SPIM_BP_STS_TX_FIFO_EMPTY & SPIM_BP_TX_STATUS_REG))
//	{
//		SPIM_BP_WriteTxData(FILLBYTE);
//	}
//    (*tabSPISel[iSPIDev])(1u);//select high to check the selected board
    if(continueRead)
    {
        Control_Reg_LoadPulse_Write(0x01);
        SPIM_BP_WriteTxData(FILLBYTE);
        (*tabSPISel[iSPIDev])(1u);//select high to check the selected board
//        Control_Reg_CD_Write(0x02u);

    }
//    else
//    {
//        Control_Reg_CD_Write(0x00u);
//    }
//	if(0u != (Timer_SelLow_ReadControlRegister() & Timer_SelLow_CTRL_ENABLE ))
//	{
    Timer_SelLow_Stop();
//	}
    CyExitCriticalSection(intState);
}
CY_ISR(ISRReadEv)
{
	uint8 intState = CyEnterCriticalSection(); //TODO consider the mutex
	EvBufferIndex tempBuffWrite = buffEvWrite;
	uint8 tempStatus = SPIS_Ev_ReadStatus();
	if (0u != (SPIS_Ev_STS_RX_BUF_NOT_EMPTY & tempStatus)) 
	{
        buffEv[tempBuffWrite] = SPIS_Ev_ReadRxData();
		tempBuffWrite = WRAPINC(tempBuffWrite, EV_BUFFER_SIZE);
        if (tempBuffWrite == buffEvRead) buffEvRead = WRAPINC(tempBuffWrite, EV_BUFFER_SIZE); //Discard oldest byte
		tempStatus = SPIS_Ev_GetRxBufferSize();
        while (tempStatus) //get all availiable bytes
		{
			buffEv[tempBuffWrite] = SPIS_Ev_ReadRxData();
            tempBuffWrite = WRAPINC(tempBuffWrite, EV_BUFFER_SIZE);
            if (tempBuffWrite == buffEvRead) buffEvRead = WRAPINC(tempBuffWrite, EV_BUFFER_SIZE); //Discard oldest byte
            tempStatus = SPIS_Ev_GetRxBufferSize();
		}
		buffEvWrite = tempBuffWrite;
	}

	CyExitCriticalSection(intState);
}
//CY_ISR(ISRDrdyCap)
//{
//	uint8 intState = CyEnterCriticalSection();
//	uint8 tempStatus = Timer_Drdy_ReadStatusRegister();
//	
//	if ((0u != (tempStatus & Timer_Drdy_STATUS_CAPTURE)) && (0u != (tempStatus & Timer_Drdy_STATUS_FIFONEMP)))
//	{
//		uint8 tempCap;
//		while(0u != (Timer_Drdy_ReadStatusRegister() & Timer_Drdy_STATUS_FIFONEMP))
//		{
//			tempCap = Timer_Drdy_ReadCapture();
//			if (0u == Pin_nDrdy_Read())
//			{
//				lastDrdyCap = tempCap;
//			}
//		}
//	}
//	if (0u != (tempStatus & Timer_Drdy_STATUS_TC))
//	{
////		if ((0u != Pin_nDrdy_Read()) || (lastDrdyCap < MIN_DRDY_CYCLES))
////		{
//			timeoutDrdy = TRUE;
//			lastDrdyCap = Timer_Drdy_ReadPeriod();
////		}
////		else
////		{
////			Timer_Drdy_WriteCounter(Timer_Drdy_ReadPeriod());
////		if(0u != (Timer_Drdy_ReadControlRegister() & Timer_Drdy_CTRL_ENABLE ))
////		{
////			Timer_Drdy_Stop();
////		}
////			Timer_Drdy_Start();
////		}
//		
//	}
//	CyExitCriticalSection(intState);
//}

CY_ISR(ISRHRTx)
{

	uint8 tempStatus;// = UART_HR_Data_ReadTxStatus();
//	uint8 intState = CyEnterCriticalSection();
//	UART_HR_Data_PutArray(buffSPIWrite, 34);
//	isr_HR_Disable();
//	if (UART_HR_Data_GetTxBufferSize() <= 1) for(uint8 x=0;x<34;x++) UART_HR_Data_PutChar(x);
//	tempStatus = UART_HR_Data_ReadTxStatus();
//	isr_HR_ClearPending();
//	isr_HR_Enable();
	
//	if (0) //TODO integrate Baro and SPI to buffframedata
	if (UART_HR_Data_GetTxBufferSize() <= 1)
	{
//	if (FALSE !=((UART_HR_Data_TX_STS_FIFO_EMPTY | UART_HR_Data_TX_STS_COMPLETE) & tempStatus))
//	{
//		UART_HR_Data_PutArray((uint8 *)(&frameCnt), 3); //little endian, need big endian
		
		uint8 buffFrame[34];
		uint8 ibuffFrame = 0;
		
		for (int i = 2; i >= 0; i--)
		{
			buffFrame[ibuffFrame] = *((uint8*)(((uint8*) &frameCnt) + i)); //this converts to big endian 3 byte counter
			UART_HR_Data_PutChar(buffFrame[ibuffFrame]);
			ibuffFrame++;
		}
		uint8 nullFrame = FALSE;
		EvBufferIndex nDataBytesLeft = DATA_BYTES_FRAME;
//		memcpy( (buffFrame + ibuffFrame), &(frameCnt), 3);
//		ibuffFrame += 3;
		frameCnt++;
		memcpy( (buffFrame + ibuffFrame), frameSync, 2);
		ibuffFrame += 2;
		memcpy( (buffFrame + ibuffFrame), frameSync, 2);
		ibuffFrame += 2;
//		buffUsbTxDebug[iBuffUsbTxDebug++] = '{';
//		buffUsbTxDebug[iBuffUsbTxDebug++] = packetFIFOHead;
//		buffUsbTxDebug[iBuffUsbTxDebug++] = '+';
//		buffUsbTxDebug[iBuffUsbTxDebug++] = packetFIFOTail;
//		buffUsbTxDebug[iBuffUsbTxDebug++] = '}';
        if (DATA_BYTES_FRAME <= WRAP(EV_BUFFER_SIZE - buffEvRead + buffEvWrite, EV_BUFFER_SIZE)) //Full frame of event data
        {
            EvBufferIndex nBytes;
            EvBufferIndex curRead = buffEvRead;
			EvBufferIndex curEOR = WRAP(curRead + (DATA_BYTES_FRAME - 1), EV_BUFFER_SIZE);
			
			if (curEOR > curRead)
			{
				nBytes = DATA_BYTES_FRAME;
			}
			else
			{
				nBytes = EV_BUFFER_SIZE - curRead;
			}
			memcpy( (buffFrame + ibuffFrame), buffEv + curRead, nBytes);
			ibuffFrame += nBytes;
			nDataBytesLeft -= nBytes;
//				curRead += (nBytes - 1); //avoiding overflow with - 1 , will add later

			if (nDataBytesLeft > 0) //more data to fill frame
			{
                memcpy( (buffFrame + ibuffFrame), buffEv, nDataBytesLeft);
			    ibuffFrame += nDataBytesLeft;
			    nDataBytesLeft = 0;
			}
			curRead = WRAPINC(curEOR, EV_BUFFER_SIZE);
            //TODO MUTEX or volatile
            buffEvRead = curRead;
        }
		else if (packetFIFOHead == packetFIFOTail)
		{
			nullFrame = TRUE;
		}
		else
		{
			uint8 curSPIDev = packetFIFO[packetFIFOHead].index;;
			SPIBufferIndex nBytes;
			SPIBufferIndex curEOR= packetFIFO[packetFIFOHead].EOR;
			SPIBufferIndex curRead = buffSPIRead[curSPIDev];
//			buffUsbTxDebug[iBuffUsbTxDebug++] = '|';
//			buffUsbTxDebug[iBuffUsbTxDebug++] = curSPIDev;
//			buffUsbTxDebug[iBuffUsbTxDebug++] = '[';
//			buffUsbTxDebug[iBuffUsbTxDebug++] = curRead;
//			buffUsbTxDebug[iBuffUsbTxDebug++] = '-';
//			buffUsbTxDebug[iBuffUsbTxDebug++] = curEOR;
//			buffUsbTxDebug[iBuffUsbTxDebug++] = ']';
			while((packetFIFOHead != packetFIFOTail) && (nDataBytesLeft > 0))
			{
				if (curEOR >= curRead)
				{
					nBytes = MIN(((curEOR - curRead) + 1), nDataBytesLeft);
				}
				else
				{
					nBytes = MIN(SPI_BUFFER_SIZE - curRead, nDataBytesLeft);
				}
				memcpy( (buffFrame + ibuffFrame), buffSPI[curSPIDev] + curRead, nBytes);
				ibuffFrame += nBytes;
				nDataBytesLeft -= nBytes;
//				curRead += (nBytes); //avoiding overflow with - 1 , will add later
				curRead += (nBytes - 1); //avoiding overflow with - 1 , will add later
//				if ((curRead - 1)== curEOR)
				if ((curRead)== curEOR)
				{
                    curRead = WRAPINC(curRead, SPI_BUFFER_SIZE); //last increment, handling the wrap
					buffSPIRead[curSPIDev]= curRead % SPI_BUFFER_SIZE;
					packetFIFOHead = WRAPINC(packetFIFOHead, PACKET_FIFO_SIZE);
					if (packetFIFOHead != packetFIFOTail) 
					{
						curSPIDev = packetFIFO[packetFIFOHead].index;
						curEOR = packetFIFO[packetFIFOHead].EOR;
						curRead = buffSPIRead[curSPIDev];
					}
				}
//				else if (curRead >= (SPI_BUFFER_SIZE))
				else if (curRead >= (SPI_BUFFER_SIZE - 1))
				{
					curRead = buffSPIRead[curSPIDev] = 0;
				}
				else
				{
                    curRead = WRAPINC(curRead, SPI_BUFFER_SIZE); //last increment, handling the wrap
					buffSPIRead[curSPIDev] = curRead;
				}
			}
		}
		while (nDataBytesLeft > 0)
		{
			buffFrame[ibuffFrame] = NULL_HEAD;
//			UART_HR_Data_PutChar(NULL_HEAD);
			ibuffFrame++;
			nDataBytesLeft--;
			if (nDataBytesLeft > 1)
			{
				memcpy( &(buffFrame[ibuffFrame]), frame00FF, 2);
				ibuffFrame += 2;
				nDataBytesLeft -= 2;
			}
			else //TODO this is an alignment error
			{
				if (1 == nDataBytesLeft)
				{
					buffFrame[ibuffFrame] = NULL_HEAD;
					ibuffFrame++;
					nDataBytesLeft--;
				}
			}
		}
		UART_HR_Data_PutArray((uint8 *)(buffFrame + 3), 31); //already sent the 3 byte counter, send rest of frame
		if (TRUE != nullFrame)
		{
			memcpy((buffUsbTx + iBuffUsbTx), buffFrame, 34);
			iBuffUsbTx += 34;
		}
	}
	tempStatus = UART_HR_Data_ReadTxStatus();
	if ((0u != USBUART_CD_GetConfiguration()) )//&& (iBuffUsbTx > 0))
		{
 
			/* Wait until component is ready to send data to host. */
			if (USBUART_CD_CDCIsReady()) // && ((iBuffUsbTx > 0) || (iBuffUsbTxDebug > 0)))
			{
				if (iBuffUsbTx > 0)
				{
					USBUART_CD_PutData(buffUsbTx, iBuffUsbTx);
					iBuffUsbTx = 0; //TODO handle missed writes
				}
				if (iBuffUsbTxDebug > 0)
				{
					while (0 == USBUART_CD_CDCIsReady());
					USBUART_CD_PutData(buffUsbTxDebug, iBuffUsbTxDebug);
					iBuffUsbTxDebug = 0; //TODO handle missed writes
				}
			}
		}
		else
		{
			iBuffUsbTx = 0; //TODO handle missed writes
			iBuffUsbTxDebug = 0; //TODO handle missed writes
		}

//	CyExitCriticalSection(intState);
}
CY_ISR(ISRBaroCap)
{
	
	uint8 continueCheck = FALSE;
//	uint8 n =0;
	do {
		uint8 i = 0;
//		uint tempStatus = Counter_BaroTemp1_ReadStatusRegister();
//		UART_HR_Data_PutChar(Counter_BaroTemp1_STATUS_FIFONEMP);
//		UART_HR_Data_PutChar(tempStatus);
//		UART_HR_Data_PutChar(Counter_BaroTemp1_STATUS_FIFONEMP & tempStatus);
//		Counter_BaroTemp1_ReadCapture();
		continueCheck = FALSE;
		if (0 != (Counter_BaroTemp1_STATUS_FIFONEMP & Counter_BaroTemp1_ReadStatusRegister()))
		{
			continueCheck = TRUE;
			buffBaroCap[i][buffBaroCapWrite[i]] = Counter_BaroTemp1_ReadCapture();
			buffBaroCapWrite[i] = WRAPINC(buffBaroCapWrite[i], NUM_BARO_CAPTURES);
		}
		i = 2;
		if (0 != (Counter_BaroTemp2_STATUS_FIFONEMP & Counter_BaroTemp2_ReadStatusRegister()))
		{
			continueCheck = TRUE;
			buffBaroCap[i][buffBaroCapWrite[i]] = Counter_BaroTemp2_ReadCapture();
			buffBaroCapWrite[i] = WRAPINC(buffBaroCapWrite[i], NUM_BARO_CAPTURES);
		}
		i = 1;
		if (0 != (Counter_BaroPres1_STATUS_FIFONEMP & Counter_BaroPres1_ReadStatusRegister()))
		{
			continueCheck = TRUE;
			buffBaroCap[i][buffBaroCapWrite[i]] = Counter_BaroPres1_ReadCapture();
			buffBaroCapWrite[i] = WRAPINC(buffBaroCapWrite[i], NUM_BARO_CAPTURES);
		}
		i = 3;
		if (0 != (Counter_BaroPres2_STATUS_FIFONEMP & Counter_BaroPres2_ReadStatusRegister()))
		{
			continueCheck = TRUE;
			buffBaroCap[i][buffBaroCapWrite[i]] = Counter_BaroPres2_ReadCapture();
			buffBaroCapWrite[i] = WRAPINC(buffBaroCapWrite[i], NUM_BARO_CAPTURES);
		}
//		n++;
	} while(continueCheck);
	//TODO Packing of Baro values along with thers like voltage.  For now just dump it to stream
//	UART_HR_Data_PutChar(DUMP_HEAD);
//	UART_HR_Data_PutChar(n);
//	UART_HR_Data_PutArray((uint8*) buffBaroCap, sizeof(buffBaroCap));
//	UART_HR_Data_PutChar(ENDDUMP_HEAD);
	for (uint8 i=0;i<(NUM_BARO *2); i++) buffBaroCapRead[i] = buffBaroCapWrite[i];
	
	
}


int main(void)
{
//	uint8 status;
//	uint8 fillByte = 0xA3u;
//	cmdBuff[CMDBUFFSIZE - 1] = FILLBYTE;
//	uint8 buffUsbTx[SPI_BUFFER_SIZE];
//	uint8 iBuffUsbTx = 0;
//	uint8 buffUsbTxDebug[SPI_BUFFER_SIZE];
//	uint8 iBuffUsbTxDebug = 0;
	uint8 buffUsbRx[USBUART_BUFFER_SIZE];
	uint8 iBuffUsbRx = 0;
	uint8 nBuffUsbRx = 0;
	enum readStatus readStatusBP = CHECKDATA;
    
    /* Variable declarations for DMA_LR_Cmd_1 */
    /* Move these variable declarations to the top of the function */
//    uint8 DMA_LR_Cmd_1_Chan;
//    uint8 DMA_LR_Cmd_1_TD[1];
    buffEvRead = buffEvWrite = 0;
	memset(buffSPIRead, 0, NUM_SPI_DEV);
	memset(buffSPIWrite, 0, NUM_SPI_DEV);
	memset(buffSPICurHead, 0, NUM_SPI_DEV);
	memset(buffSPICompleteHead, 0, NUM_SPI_DEV);
	memset(buffUsbTx, 0, USBUART_BUFFER_SIZE);
	memset(curBaroTemp, 0, NUM_BARO);
	memset(curBaroPres, 0, NUM_BARO);
	memset(curBaroTempCnt, 0, NUM_BARO);
	memset(curBaroPresCnt, 0, NUM_BARO);
    memset(commandStatusC, WAIT_DLE, COMMAND_SOURCES);
    memset(buffCmdRxCRead, 0, COMMAND_SOURCES);
    memset(readBuffCmd, 0, COMMAND_SOURCES);
    memset(writeBuffCmd, 0, COMMAND_SOURCES);
    
    for (uint8 i = 0; i < COMMAND_SOURCES; i++)
    {
        orderBuffCmd[i] = i; //read the cmd buff in order
    }
    memcpy(&buffCmd[0][0][0], initCmd, (NUMBER_INIT_CMDS * 2));
    writeBuffCmd[0] = NUMBER_INIT_CMDS;
//    memcpy(&buffCmd[1][0][0], initCmd, (NUMBER_INIT_CMDS * 2));
//    writeBuffCmd[1] = NUMBER_INIT_CMDS;
    
//	buffUsbTx[3] = 0x55;
//	buffUsbTx[4] = 0xAA;
//	buffUsbTx[5] = 0x55;
//	buffUsbTx[6] = 0xAA;
//	iBuffUsbTx = 7;
//	uint16 tempSpinTimer = 0; //TODO replace
	
    /* DMA Configuration for DMA_LR_Cmd_1 */
//    DMA_LR_Cmd_1_Chan = DMA_LR_Cmd_1_DmaInitialize(DMA_LR_Cmd_1_BYTES_PER_BURST, DMA_LR_Cmd_1_REQUEST_PER_BURST, 
//        HI16(DMA_LR_Cmd_1_SRC_BASE), HI16(DMA_LR_Cmd_1_DST_BASE));
//    DMA_LR_Cmd_1_TD[0] = CyDmaTdAllocate();
//    CyDmaTdSetConfiguration(DMA_LR_Cmd_1_TD[0], DMA_LR_Cmd_1_BUFFER_SIZE, DMA_LR_Cmd_1_TD[0], CY_DMA_TD_INC_DST_ADR);
//    CyDmaTdSetAddress(DMA_LR_Cmd_1_TD[0], LO16((uint32)UART_LR_Cmd_1_RXDATA_REG), LO16((uint32)buffCmdRxC[0]));
//    CyDmaChSetInitialTd(DMA_LR_Cmd_1_Chan, DMA_LR_Cmd_1_TD[0]);
//    CyDmaChEnable(DMA_LR_Cmd_1_Chan, 1);
    
    buffCmdRxCWritePtr[0] = (reg16 *) &CY_DMA_TDMEM_STRUCT_PTR[0].TD1[2u];
    
	SPIM_BP_Start();
	SPIM_BP_ClearFIFO();
    SPIS_Ev_Start();
//	USBUART_CD_Start(USBFS_DEVICE, USBUART_CD_5V_OPERATION);
	USBUART_CD_Start(USBFS_DEVICE, USBUART_CD_3V_OPERATION);
	UART_Cmd_Start();
	UART_HR_Data_Start();
	UART_LR_Cmd_1_Start();
	UART_LR_Cmd_2_Start();
	UART_LR_Data_Start();
    
    Pin_Sel_HV1_Write(0);
    Pin_Sel_HV2_Write(0);
   
    Pin_Sel_Pwr_Write(0);
    Pin_Sel_PHA_Write(0);
    Pin_Sel_Ctr1_Write(0);
    Pin_Sel_Tkr_Write(0);
    Pin_Sel_Ctr3_Write(0);
    
		   /* Service USB CDC when device is configured. */
//	if ((0u != USBUART_CD_GetConfiguration()) && (iBuffUsbTx > 0))
//	{
//
//		/* Wait until component is ready to send data to host. */
//		if (USBUART_CD_CDCIsReady())
//		{
//			USBUART_CD_PutChar('S'); //TODO  different or eliminate startup message
//		}
//	}
//	lastDrdyCap = Timer_Drdy_ReadPeriod();
	
//	Control_Reg_R_Write(0x00u);

//	Control_Reg_SS_Write(tabSPISel[0u]);
//	Control_Reg_CD_Write(1u);
	
	
	
	isr_R_StartEx(ISRReadSPI);
	isr_W_StartEx(ISRWriteSPI);
//	isr_C_StartEx(ISRDrdyCap);
	isr_Cm_StartEx(ISRCheckCmd);
	isr_E_StartEx(ISRReadEv);
	
	
//	Timer_Tsync_Start();
//	Timer_SelLow_Start();
//	Timer_Drdy_Start();

	Counter_BaroPres1_Start();
	Counter_BaroTemp1_Start();
	Counter_BaroPres2_Start();
	Counter_BaroTemp2_Start();
//	cmdBuff[0] = 0x0Fu;
//	cmdBuff[1] = 0xF0u;
//	SPIM_BP_WriteTxData(cmdBuff[0]);
//	iCmdBuff = 1;
	SPIM_BP_TxDisable();
//	for(uint8 x=0;x<34;x++) UART_HR_Data_PutChar(x);
	CyGlobalIntEnable; /* Enable global interrupts. */
//	ISRHRTx();
	isr_HR_StartEx(ISRHRTx);
	
//	SendInitCmds();
	isr_B_StartEx(ISRBaroCap);
    
    
	for(;;)
	{
		
		/* Place your application code here. */
        int tempRes = CheckCmdBuffers();
		//if (SPIM_BP_GetRxBufferSize > 0)
		//{
//			SPIM_BP_ReadRxData();
			

		//while(Pin_Sel_Read());
		
			
//			do{
//				status = SPIM_BP_ReadTxStatus();
//			}while (!(status & ( SPIM_BP_STS_SPI_IDLE)));
			
//			while((Status_Reg_nSS_Read()));
//			SPIM_BP_ClearTxBuffer();
//
//			SPIM_BP_WriteTxData(fillByte);
		//}
		if (0u != USBUART_CD_IsConfigurationChanged())
		{
			/* Initialize IN endpoints when device is configured. */
			if (0u != USBUART_CD_GetConfiguration())
			{
				/* Enumeration is done, enable OUT endpoint to receive data 
				 * from host. */
				USBUART_CD_CDC_Init();
			}
		}

		/* Service USB CDC when device is configured. */
		if ((nBuffUsbRx == iBuffUsbRx) && (0u != USBUART_CD_GetConfiguration()))
		{
			/* Check for input data from host. */
			if (0u != USBUART_CD_DataIsReady())
			{
				/* Read received data and re-enable OUT endpoint. */
				nBuffUsbRx = USBUART_CD_GetAll(buffUsbRx);
				iBuffUsbRx = 0;
//                buffUsbTxDebug[iBuffUsbTxDebug++] = nBuffUsbRx; //Debug

			}
		}
//		if ((6 == nBuffUsbRx) && (DLE == buffUsbRx[0])) //debug 
//		{
//            UART_LR_Data_PutArray(buffUsbRx, 6);
////            buffUsbTxDebug[iBuffUsbTxDebug++] = '^'; //Debug
////            buffUsbTxDebug[iBuffUsbTxDebug++] = CY_DMA_TDMEM_STRUCT_PTR[0].TD1[2u] & 15; //Debug
//            
////            memcpy(buffUsbTxDebug + iBuffUsbTxDebug, buffCmdRxC, 16);
////            iBuffUsbTxDebug +=16; //debug
//        }
//        else //debug
//        {
//            UART_LR_Data_PutArray(buffUsbRx, nBuffUsbRx);
//        }
        
        for(uint8 x = 0; x < nBuffUsbRx; x++)
        {
            tempRes = ParseCmdInputByte(buffUsbRx[x], (COMMAND_SOURCES - 1));
            if (0 > tempRes)
            {
                //TODO error handling
            }
        }
        iBuffUsbRx = 0;
        nBuffUsbRx = 0;
//		if (nBuffUsbRx > iBuffUsbRx)
//		{
//			uint8 nByteCpy =  MIN(COMMAND_CHARS - iCurCmd, nBuffUsbRx - iBuffUsbRx);
//			if (nByteCpy > 0)
//			{
//				memcpy((curCmd + iCurCmd), (buffUsbRx + iBuffUsbRx), nByteCpy);
//				iCurCmd += nByteCpy;
//				iBuffUsbRx += nByteCpy;
//			}
//				
//			if ((iCurCmd >= COMMAND_CHARS) && (0u != (UART_Cmd_TX_STS_FIFO_EMPTY | UART_Cmd_ReadTxStatus())))
//			{
//				uint8 cmdValid = TRUE;
//				//all nibbles of the command must be uppercase hex char 
//				for(uint8 x = 0; ((x < COMMAND_CHARS) && cmdValid); x++)
//				{
//					if ((!(isxdigit(curCmd[x]))) || (curCmd[x] > 'F'))
//					{
//						cmdValid = FALSE; 
//					}
//				}
//				if (cmdValid)
//				{
//					//DEBUG echo command no boundary check
//					memcpy(buffUsbTxDebug, "++", 2);
//					memcpy(buffUsbTxDebug +2, curCmd, COMMAND_CHARS);
//					iBuffUsbTxDebug += 6;
//					//Write 3 times cmd on backplane
//                    SendCmdString(curCmd);//, FALSE);
////					for (uint8 x=0; x<3; x++)
////					{
////						UART_Cmd_PutArray(START_COMMAND, START_COMMAND_SIZE);
////						memcpy(buffUsbTxDebug + iBuffUsbTxDebug, START_COMMAND, START_COMMAND_SIZE);
////						iBuffUsbTxDebug += START_COMMAND_SIZE;
////						UART_Cmd_PutArray(curCmd, COMMAND_CHARS);
////						memcpy(buffUsbTxDebug + iBuffUsbTxDebug, curCmd, COMMAND_CHARS);
////						iBuffUsbTxDebug += COMMAND_CHARS;
////						UART_Cmd_PutArray(END_COMMAND, END_COMMAND_SIZE);
////						memcpy(buffUsbTxDebug + iBuffUsbTxDebug, END_COMMAND, END_COMMAND_SIZE);
////						iBuffUsbTxDebug += END_COMMAND_SIZE;
////					}
////					//Unix style line end
////					UART_Cmd_PutChar(CR);
////					UART_Cmd_PutChar(LF);	
//				}
//				else 
//				{
//					//DEBUG echo command no boundary check
//					memcpy(buffUsbTxDebug, "--", 2);
//					memcpy(buffUsbTxDebug + 2, curCmd, COMMAND_CHARS);
//					iBuffUsbTxDebug += 6;
//				}
//				iCurCmd = 0;	
//			}
			
//		}
//		CheckCmdDma(0);
        
		switch (readStatusBP)
		{
			case CHECKDATA:
                
            
//				if(0u == (Timer_Drdy_ReadControlRegister() & Timer_Drdy_CTRL_ENABLE ))
//				{
//				Control_Reg_CD_Write(0x01u);
                
                
//					lastDrdyCap = Timer_Drdy_ReadPeriod();
//					Timer_Drdy_Start();
					
//				}
                (*tabSPISel[iSPIDev])(1u);//select high to check the selected board
                uint8 highLoops;
                if (loopCount < loopCountCheck) // check overflow
                {
                    highLoops = (255 - loopCountCheck) + loopCount;
                }
                else
                {
                    highLoops = loopCount - loopCountCheck;
                }
                uint8 tempnDrdy = Pin_nDrdy_Filter_Read();
//                if(FALSE) //TODO New gltch filter test
//				if (TRUE == timeoutDrdy)
                if (SELECT_HIGH_LOOPS < highLoops) //timeout, no daata
				{  
//					if (iSPIDev >= (NUM_SPI_DEV - 1))
//					{
//						iSPIDev = 0;
//					}
//					else
//					{
//						iSPIDev++;
//					}
//					if (0x0FFFu == ++tempSpinTimer)
//					{
//					Control_Reg_CD_Write(0u);
                    (*tabSPISel[iSPIDev])(0u);//select low before switching
					iSPIDev = WRAPINC(iSPIDev, NUM_SPI_DEV);
                    (*tabSPISel[iSPIDev])(1u);//select high to check the selected board
//					Control_Reg_SS_Write(tabSPISel[iSPIDev]);
//					Control_Reg_CD_Write(1u);
					
//					timeoutDrdy = FALSE;
//					lastDrdyCap = Timer_Drdy_ReadPeriod();
//					Timer_Drdy_Stop();
//					Timer_Drdy_Start();
//					tempSpinTimer = 0;
//					}
                    loopCountCheck = loopCount;
				}
//                else if(FALSE) //TODO New gltch filter test
//				else if ((0u == Pin_nDrdy_Read()) )//&& (0u == (Timer_Drdy_ReadStatusRegister() & Timer_Drdy_STATUS_FIFONEMP)))
                else if (0u == tempnDrdy) 
				{
//					uint8 tempLastDrdyCap = lastDrdyCap;
//					Timer_Drdy_SoftwareCapture();
//					uint8 tempCounter = Timer_Drdy_ReadCounter();
//					if (tempCounter > tempLastDrdyCap) tempCounter = 0;
					//if ((0u == Pin_nDrdy_Read()) && (0u != (SPIM_BP_TX_STATUS_REG & SPIM_BP_STS_TX_FIFO_EMPTY)))
//                    if(FALSE) //TODO New gltch filter test
//					if ((tempLastDrdyCap - tempCounter) >= MIN_DRDY_CYCLES)
//					{
						SPIBufferIndex tempBuffWrite = buffSPIWrite[iSPIDev];
//						Control_Reg_CD_Write(0x03u);
						Control_Reg_LoadPulse_Write(0x01u);
						buffSPICurHead[iSPIDev] = buffSPIWrite[iSPIDev];
						buffSPIWrite[iSPIDev] = WRAP3INC(tempBuffWrite, SPI_BUFFER_SIZE);
						if (0u != (SPIM_BP_STS_TX_FIFO_EMPTY | SPIM_BP_TX_STATUS_REG))
						{
							SPIM_BP_WriteTxData(FILLBYTE);
						}
						
						buffSPI[iSPIDev][tempBuffWrite] = tabSPIHead[iSPIDev];
						tempBuffWrite=WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
						if((SPI_BUFFER_SIZE - 1) == tempBuffWrite) //check for 2 byte wrap
						{
							buffSPI[iSPIDev][(SPI_BUFFER_SIZE - 1)] = frame00FF[0];
							buffSPI[iSPIDev][0] = frame00FF[1];
						}
						else
						{
							memcpy(&(buffSPI[iSPIDev][tempBuffWrite]), frame00FF, 2);
						}
						
	  
						
						continueRead = TRUE; 
						readStatusBP = READOUTDATA;
//						timeoutDrdy = FALSE;
//						lastDrdyCap = Timer_Drdy_ReadPeriod();
						
//						if(0u != (Timer_Drdy_ReadControlRegister() & Timer_Drdy_CTRL_ENABLE ))
//						{   
//							Timer_Drdy_Stop();
//						}
//						tempSpinTimer = 0;
//					}
//					else //TODO New gltch filter test
//					{
//						buffUsbTxDebug[iBuffUsbTxDebug++] = '=';
//						buffUsbTxDebug[iBuffUsbTxDebug++] = tempLastDrdyCap;
//						buffUsbTxDebug[iBuffUsbTxDebug++] = '-';
//						buffUsbTxDebug[iBuffUsbTxDebug++] = tempCounter;
//						lastDrdyCap = tempLastDrdyCap;
//					}
				}
				
				break;
				
			case READOUTDATA:
				//TODO actually check 3 byte EOR, count errrors 
//				Control_Reg_CD_Write(0u);
//				if (TRUE == continueRead)
//				{
//					//if (continueReadFlags == (continueReadFlags | SPIM_BP_TX_STATUS_REG))
//					if ((0u != (SPIM_BP_STS_SPI_IDLE | SPIM_BP_TX_STATUS_REG)))
//					{
//						if (0u != (SPIM_BP_STS_TX_FIFO_EMPTY | SPIM_BP_TX_STATUS_REG))
//						{
//							if (0x0005 == ++tempSpinTimer)
//							{
//								SPIM_BP_WriteTxData(FILLBYTE);
//								tempSpinTimer = 0;
//							}
//							
//						}
//					}
//				}
//				if (0u == (0x03u & Control_Reg_CD_Read()))
                if((FALSE == continueRead) && (0u == (Timer_SelLow_ReadControlRegister() & Timer_SelLow_CTRL_ENABLE )))
				{
//                    Control_Reg_CD_Write(0u);
					if (buffSPICurHead[iSPIDev] == buffSPIWrite[iSPIDev]) //TODO this should't be true due to ISR
					{
											
//						uint8 nBytes = SPI_BUFFER_SIZE - buffSPIRead[iSPIDev];
//						
//						
//						memcpy((buffUsbTx + iBuffUsbTx), &(buffSPI[iSPIDev][buffSPIRead[iSPIDev]]), nBytes);
//						iBuffUsbTx += nBytes;
//						if (nBytes < SPI_BUFFER_SIZE)
//						{
//							nBytes = SPI_BUFFER_SIZE - nBytes;
//							memcpy((buffUsbTx + iBuffUsbTx), &(buffSPI[iSPIDev][0]), nBytes);
//							iBuffUsbTx += nBytes;
//						}
						readStatusBP = EORERROR;
					}
					//if ((1u == Pin_nDrdy_Read()) && (0u != (SPIM_BP_STS_SPI_IDLE | SPIM_BP_TX_STATUS_REG)))
					else
					{
						SPIBufferIndex tempBuffWrite = buffSPIWrite[iSPIDev];
						int16 tempLen = tempBuffWrite - buffSPICurHead[iSPIDev];
                        
//						uint8 nBytes;
						
                        if (0 > tempLen) tempLen += SPI_BUFFER_SIZE;
                        
                        tempLen %= 3; //bytes over 3 byte alignment
                        
                        if (tempLen) tempLen = 3 - tempLen; //check if not 3 byte aligned, then calculate number of padding bytes
                        
                        int16 tempLeft = buffSPIRead[iSPIDev] - tempBuffWrite;
                        if (0 > tempLeft) tempLeft += SPI_BUFFER_SIZE;
                        
                        if (tempLeft < (tempLen + 3))
                        {
                            readStatusBP = EORERROR;
                        }
                        else 
                        {
                            if (tempLen)
                            {
                                while (tempLen--)
                                {
                                    buffSPI[iSPIDev][tempBuffWrite] = 0; //pad 0
                                    tempBuffWrite = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
                                }
                                //buffSPIWrite[iSPIDev] = tempBuffWrite;
//                                tempBuffWrite = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
                            }
    						buffSPIWrite[iSPIDev] = WRAP3INC(tempBuffWrite, SPI_BUFFER_SIZE);
    						buffSPI[iSPIDev][tempBuffWrite] = EOR_HEAD;
    						tempBuffWrite = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
    						if((SPI_BUFFER_SIZE - 1) == tempBuffWrite) //check for 2 byte wrap
    						{
    							buffSPI[iSPIDev][(SPI_BUFFER_SIZE - 1)] = frame00FF[0];
    							buffSPI[iSPIDev][0] = frame00FF[1];
//    							tempBuffWrite = 1;
    						}
    						else
    						{
    							memcpy(&(buffSPI[iSPIDev][tempBuffWrite]), frame00FF, 2); //Copy 0x00FF
//    							tempBuffWrite += 1; 
//                                tempBuffWrite = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE); //this is te locatiion of the last byte
    						}
    						
    						packetFIFO[packetFIFOTail].header = buffSPICompleteHead[iSPIDev] = buffSPICurHead[iSPIDev];
    						packetFIFO[packetFIFOTail].index = iSPIDev;
                            if (buffSPIWrite[iSPIDev])
                            {
    						    packetFIFO[packetFIFOTail].EOR = buffSPIWrite[iSPIDev] - 1;
                            }
                            else
                            {
    						    packetFIFO[packetFIFOTail].EOR = SPI_BUFFER_SIZE - 1;
                            }
    						packetFIFOTail = WRAPINC(packetFIFOTail, PACKET_FIFO_SIZE);
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = '|';
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = iSPIDev;
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = '[';
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = buffSPICurHead[iSPIDev];
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = '-';
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = tempBuffWrite;
    //						buffUsbTxDebug[iBuffUsbTxDebug++] = ']';
    						
    						
    						
    //						buffSPI[iSPIDev][tempBuffWrite] = 0x00;
    //						tempBuffWrite = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
    //						buffSPI[iSPIDev][tempBuffWrite] = 0xFF;
    //						tempBuffWrite = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
    //						if (buffSPIRead[iSPIDev] >= tempBuffWrite)
    //						{
    //							nBytes = SPI_BUFFER_SIZE - buffSPIRead[iSPIDev];
    //							memcpy((buffUsbTx + iBuffUsbTx), &(buffSPI[iSPIDev][buffSPIRead[iSPIDev]]), nBytes);
    //							iBuffUsbTx += nBytes;
    //							buffSPIRead[iSPIDev] = 0;
    //						}
    //						nBytes = tempBuffWrite - buffSPIRead[iSPIDev];
    //						memcpy((buffUsbTx + iBuffUsbTx), &(buffSPI[iSPIDev][buffSPIRead[iSPIDev]]), nBytes);
    //						iBuffUsbTx += nBytes;
    //						buffSPIRead[iSPIDev] = tempBuffWrite;
    						readStatusBP = EORFOUND;
                        }
					}
					 
				}
                //TODO timeout
//				else 
//				{
//					if (0u != (SPIM_BP_STS_SPI_IDLE | SPIM_BP_TX_STATUS_REG))
//					{
//						if (0x0FFFu == ++tempSpinTimer)
//						{
//							readStatusBP = EORERROR;
//						}
//					}
//					else
//					{
//						tempSpinTimer = 0;
//					}
//				}
				break;
				
			case EORERROR:
			case EORFOUND:  
//				Control_Reg_CD_Write(0u);
                (*tabSPISel[iSPIDev])(0u);//select low to mke sure
//                continueRead = TRUE;
//				if(0u != (Timer_SelLow_ReadControlRegister() & Timer_SelLow_CTRL_ENABLE ))
//				{
					Timer_SelLow_Stop();
//				}
//				if (0u != (SPIM_BP_STS_SPI_IDLE | SPIM_BP_TX_STATUS_REG))
//				{
					if (0) //(0u !=(SPIM_BP_STS_RX_FIFO_NOT_EMPTY & SPIM_BP_ReadStatus())) //TODO this shouldnt happen Readout any further bytes
					{   
						SPIM_BP_ReadRxData();
//						uint8 tempBuffWrite = buffSPIWrite[iSPIDev];
//						buffSPIWrite[iSPIDev] = WRAPINC(tempBuffWrite, SPI_BUFFER_SIZE);
//						buffSPI[iSPIDev][tempBuffWrite] = SPIM_BP_ReadRxData();
//						buffUsbTx[iBuffUsbTx++] = buffSPI[iSPIDev][tempBuffWrite];
						
					}
					else
					{
//						if (buffSPIRead[iSPIDev] != buffSPIWrite[iSPIDev])
//						{
//							uint8 tempBuffWrite = buffSPIWrite[iSPIDev];
//					
//							uint8 nBytes;
//							
//							if (buffSPIRead[iSPIDev] >= tempBuffWrite)
//							{
//								nBytes = SPI_BUFFER_SIZE - buffSPIRead[iSPIDev];
//								memcpy((buffUsbTx + iBuffUsbTx), &(buffSPI[iSPIDev][buffSPIRead[iSPIDev]]), nBytes);
//								iBuffUsbTx += nBytes;
//								buffSPIRead[iSPIDev] = 0;
//							}
//							nBytes = tempBuffWrite - buffSPIRead[iSPIDev];
//							memcpy((buffUsbTx + iBuffUsbTx), &(buffSPI[iSPIDev][buffSPIRead[iSPIDev]]), nBytes);
//							iBuffUsbTx += nBytes;
//							buffSPIRead[iSPIDev] = tempBuffWrite;
//						}
						iSPIDev = WRAPINC(iSPIDev, NUM_SPI_DEV);
//						Control_Reg_SS_Write(tabSPISel[iSPIDev]);
//						Control_Reg_CD_Write(1u);
						(*tabSPISel[iSPIDev])(1u);//select high to check the selected board
//						lastDrdyCap = Timer_Drdy_ReadPeriod();
						
//						Timer_Drdy_Start();
						readStatusBP = CHECKDATA;
                        loopCountCheck = loopCount;
					}
//				}
				break;
		}
//				if (NewTransmit)
//		{
		
		//TODO Framing packets
			 /* Service USB CDC when device is configured. */
		if ((0u != USBUART_CD_GetConfiguration()) )//&& (iBuffUsbTx > 0))
		{
 
			/* Wait until component is ready to send data to host. */
			if (USBUART_CD_CDCIsReady()) // && ((iBuffUsbTx > 0) || (iBuffUsbTxDebug > 0)))
			{
//				if ((0 == iBuffUsbTx) && (0 == iBuffUsbTxDebug) && (0 == Pin_BaroPres_Read()) && (0 == Pin_BaroTemp_Read()) && (0 != baroReadReady)) // TODO Temporary barometer read, in future should be a Tsync interrupt
//				{
//					curBaroPresCnt[0] = Counter_BaroPres_ReadCapture();
//					curBaroTempCnt[0] = Counter_BaroTemp_ReadCapture();
//					double U = (double)((double) curBaroTempCnt[0] / (double) BARO_COUNT_TO_US) - baroCE[0].U0;
//					double Tao = (double)((double) curBaroPresCnt[0] / (double) BARO_COUNT_TO_US);
//					curBaroTemp[0] = BaroTempCalc(U, baroCE);
//					curBaroPres[0] = BaroPresCalc(Tao, U, baroCE);
//					uint32 curBaroTempInt = (uint32) curBaroTemp[0];
//					uint32 curBaroPresInt = (uint32) curBaroPres[0];
////					buffUsbTxDebug[0] = '^';
////					iBuffUsbTxDebug++;
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (curBaroPresCnt), 4);
////					iBuffUsbTxDebug += 4;
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (&Tao), sizeof(double));
////					iBuffUsbTxDebug += sizeof(double);
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (curBaroPres), sizeof(double));
////					iBuffUsbTxDebug += sizeof(double);
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (&curBaroPresInt), sizeof(uint32));
////					iBuffUsbTxDebug += sizeof(uint32);
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), "^!", 2);
////					iBuffUsbTxDebug += 2;
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (curBaroTempCnt), 4);
////					iBuffUsbTxDebug += 4;
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (&U), sizeof(double));
////					iBuffUsbTxDebug += sizeof(double);
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (curBaroTemp), sizeof(double));
////					iBuffUsbTxDebug += sizeof(double);
////					memcpy((buffUsbTxDebug + iBuffUsbTxDebug), (&curBaroTempInt), sizeof(uint32));
////					iBuffUsbTxDebug += sizeof(uint32);
////					buffUsbTxDebug[iBuffUsbTxDebug] = '!';
////					iBuffUsbTxDebug++;
//					Tao = 1.1; 
//					curBaroPres[0] = 22.22; 
//					U = 333.333;
//					curBaroTemp[0] = 4444.4444;
//					Tao = 2.3E-3;//08;
//					U = 1.7E+3;//08;
//					U /= Tao;
//					iBuffUsbTxDebug = sprintf( (char *) buffUsbTxDebug, "^ %lu, %f, %f^! %lu, %f, %f!", curBaroPresCnt[0], Tao, curBaroPres[0], curBaroTempCnt[0], U, curBaroTemp[0]); //1.1, 22.22, curBaroTempCnt[0], 333.333, 4444.4444);
//					USBUART_CD_PutData(buffUsbTxDebug, iBuffUsbTxDebug);
//					iBuffUsbTxDebug = 0;
//					baroReadReady = 0u;
//					
//				}
//				else if (0 != Pin_BaroPres_Read())
//				{
//					baroReadReady = 1u;
//				}
				if (iBuffUsbTx > 0)
				{
//					for(uint8 x = 0; x < iBuffUsbTx; x += USBUART_BUFFER_SIZE)
//					{
//						uint8 iTemp = iBuffUsbTx - x;
//						iTemp = MIN(iTemp, USBUART_BUFFER_SIZE);
//						uint8 tempS[4] = {'m', x, iTemp, 'n'};
//						USBUART_CD_PutData(tempS, 4);
//						while (0 == USBUART_CD_CDCIsReady());
						USBUART_CD_PutData(buffUsbTx, iBuffUsbTx);
//						if (USBUART_BUFFER_SIZE == iTemp)
//						{
//							CyDelayUs(53333);
//						}
//					}
//					USBUART_CD_PutChar('#');
//					USBUART_CD_PutData((const uint8*)(&(iBuffUsbTx)), 1);
//					char tempS[3];
//					sprintf(tempS,"%i", iBuffUsbTx);
//					USBUART_CD_PutString(tempS);
//					USBUART_CD_PutChar('#');
//					uint8 tempS[3] = {'#', iBuffUsbTx, '#'};
//					while (0 == USBUART_CD_CDCIsReady());
//					USBUART_CD_PutData(tempS, 3);
					iBuffUsbTx = 0; //TODO handle missed writes
					
				}
				if (iBuffUsbTxDebug > 0)
				{
					while (0 == USBUART_CD_CDCIsReady());
					USBUART_CD_PutData(buffUsbTxDebug, iBuffUsbTxDebug);
					iBuffUsbTxDebug = 0; //TODO handle missed writes
				}
				
				
		
				//iBuffUsbTx = 0;
			}
			
		}
		else
		{
			iBuffUsbTx = 0; //TODO handle missed writes
			iBuffUsbTxDebug = 0; //TODO handle missed writes
		}
		
				/* Send data back to host. */
			   
//				NewTransmit = FALSE;
//
//
//			}
//		}
        loopCount++;
	}
}

/* [] END OF FILE */
