/*
**************************************************************************************************
Arduino LoRa Send and Receive Programs

Copyright of the author Stuart Robinson - 25/04/2015 09:00

These programs may be used free of charge for personal, recreational and educational purposes only.

This program, or parts of it, may not be used for or in connection with any commercial purpose without
the explicit permission of the author Stuart Robinson.

The programs are supplied as is, it is up to individual to decide if the programs are suitable for the
intended purpose and free from errors.
**************************************************************************************************
*/



/*
**************************************************************************************************
Notes
**************************************************************************************************

'to do
'check operation when packet at max length

Semtech released the SX127X series of LoRa RF transceivers late in 2013 and these are now available in a range of modules,
two of which are of particular interest to the hobbyist, the Dorji DRF1278F and the Hope RFM98, both are a similar size
and cost to the FSK device Hope RFM22B. These are devices designed to operate principally in the ISM bands, 434MHz in
the UK and Europe.

LoRa (for Long Range) is a type of chirp spread spectrum modulation which is completely different to the modulation used
on FSK transceivers of which the RFM22B is the popular example. The benefit of LoRa is that it offers substantial link
margin improvement of around 20dB over previous FSK devices, this equates to a distance improvement of around 10 times.
A report on the testing and capability of the LoRa devices called  "Semtech LoRa Transceivers - a KISS approach to
Long Range Data Telemetry.PDF" will be found on the HABAXE Dropbox here;

https://www.dropbox.com/sh/5ceassf1mp6mg7p/AADnVMI61RLUVln1X8Tg83oFa?dl=0

I developed, tested and flew a High Altitude Balloon (HAB) tracker based on the RFM98 LoRa device and a PICAXE 28X2.
A portable PICAXE LoRa receiver was also built, which works well, but I wanted to include some features into the receiver
that would be difficult to implement using PICAXE BASIC. The PICAXE basic routines were first translated into MMBasic for
the MicroMite II;

http://geoffg.net/micromite.html

MMBasic is a structured BASIC with parameter passing sub routines and functions, so once the LoRa receiver was working with a
Micromite, I set about converting the routines for use on an Arduino.

An Arduino compatible shield base was designed, see the Dropbox link above, that was used to develop programs first on a PICAXE
using the PICAXE Arduino compatible shield base, then a shield base built for Micromite and finaly with an Arduino itself.


Arduino Programs
----------------

The Arduino programs use two 256 byte arrays, the first is used to build the packet to send and the other will contain the
contents of the last packet received.

Functions are also provided to initialise and set-up the LoRa device for the appropriate frequency and the particular LoRa bandwidth,
spreading factor and coding rate.

You will need to ensure that the RFM98 or DRF1278F is connected to the pins as defined in lora_Hardware.h

The intial program setup is carried out by calling functions; program_Setup(), lora_ResetDev() and lora_Setup().

To send a data packet you need to;

1.Call the frequency setting function,  lora_SetFreq() with the appropriate frequency in Mhz such as 434.400.
2.Call the modem parameters setup function, lora_SetModem() with the appropriate bandwidth, spreading factor, coding rate,
header type and whether low data rate optimisation should be on.
3.Fill the transmit buffer array lora_TXBuff with the data you want to send, set a variable (lora_TXStart) to the location in the
buffer where the payload starts and set lora_TXEnd to the location where the payload ends.
4.Call the lora_Send function, providing the buffer information above, the packet type, destination node, source node, send time-out
and power level in dBm. The packet should then be sent.

To receive a packet you need to;

1.Call the frequency setting function, lora_SetFreq() with the appropriate frequency in Mhz such as 434.400
2.Call the modem parameters set-up function, lora_SetModem() with the appropriate bandwidth, spreading factor,
coding rate, header type and whether low data rate optimisation should be on.
3.Call the lora_Listen function providing an RX time-out.
4.If a packet is received the payload will be in the array lora_RXBUFF, the start of the payload in the buffer will be pointed to by
variable lora_TXStart and the end of the payload by in lora_RXEnd.

There are routines for printing to the terminal the contents of the TX and RX payloads as ASCII, decimal numbers or hexadecimal.

A basic addressing scheme has been implemented. Preceding the transmitted and received payloads are 3 bytes automatically added
on transmit and extracted on receive, they are;

1.Packet type, a single byte defining what the contents of the packet are or what to do with it.
2.Destination node, a single byte specifying the destination station for the packet, 255 is a broadcast.
3.Source node, a single byte specifying which station the packet came from.

These 3 header bytes can be ignored, if you have no need for them.

For information on the various LoRa modes the devices are capable of, you will need to refer to the Semtech data sheet;

http://www.semtech.com/wireless-rf/rf-transceivers/sx1276/

A description of the operation of the program routines follows below. In general if a variable is restricted to a function only,
it is preceded with an L in the variable name, e.g. lora_LRegData.

Stuart Robinson
GW7HPW
April 2015



Description of Functions
------------------------

program.Setup()
Runs at program start, sets up hardware pins etc.


lora_Setup()
Initialises the LoRa device, resets the device and sets variables to defaults.


lora_CheckDevice()
On Exit:
Returns 1 if device responds correctly to a write and read of the frequency setting registers.  Global flag lora_FDeviceError set
to 0 if device responds OK, set to 1 if it fails.


lora_ResetDev()
Drives the reset pin on the LoRa device low then high to reset all registers to default.


lora_Write(lora_LReg,lora_LRegData)
Writes the value in lora_LRegData  to the register given in lora_LReg.


lora_Read(lora_LReg)
Returns the value read from the device register pointed to by lora_LReg.


lora_SetFreq(lora_LSetFreq)
Takes the value of lora_LSetFreq as a float in Mhz, calculates and sets the 3 frequency setting registers to the correct values.


lora_GetFreq()
Reads the 3 frequency setting registers and returns the result as a float in MHz and also stores the result as a float in variable
lora_Frequency as Mhz.


lora_SetModem(lora_LBW,lora_LSF, lora_LCR, lora_LHDR,lora_LLDROPT)
Sets LoRa modem to appropriate mode. The symbols to use are;
Bandwidth, lora_LBW; lora_BW7_8, lora_BW10_4, lora_BW15_6, lora_BW20_8, BW31_2, lora_BW41_7, lora_BW62_5, lora_BW125, lora_BW250,
lora_BW500
Spreading factor, lora_LSF; 6, 7, 8, 9, 10,11,12
Coding Rate, lora_LCR; lora_CR4_5, lora_CR4_6, lora_CR4_7, lora_CR4_8
Header mode, lora_LHDR;  lora_EXPLICIT, lora_IMPLICIT
Low Data Rate Optimisation mode; lora_LowDoptON, lora_LowDoptON  (see SX127X data sheet page 30 for details)


lora_PrintModem()
Prints to terminal the values of the LoRa modem settings registers which indicate bandwidth, spreading factor and coding rate in use.
Used to ensure during debug the program is in the correct mode.


lora_Print()
Prints the contents of device registers to terminal as hexadecimal, used for debugging.


lora_Send(lora_LTXStart, lora_LTXEnd, lora_LTXPacketType, lora_LTXDestination, lora_LTXSource, lora_LTXTimeout, lora_LTXPower)
Sends a LoRa packet using current modem and frequency settings.
lora_LTXStart; Start Location in buffer, pointer to start of packet in TXbuff
lora_LTXEnd; End Location in buffer, pointer to end of packet in TXbuff
lora_LTXPacketType, 0-255, byte defining the type of packet
lora_LTXDestination; TX Destination Node, 0-255, Destination node for packet, 255 is assumed broadcast
lora_LTXSource; Source Node, 0-255, Node packet is sent from
lora_LTXTimeout; Transmitter Time-out in mS, 0 for no time-out check
lora_LTXPower; Transmit power in dBm, 17 to 2
On Exit sets global variables;
lora_TXPacketL, 1-255, Length of transmitted packet, includes 3 header bytes
lora_FTXOK is 1 if packet sent OK, 0 if failed


lora_FillTX()
Fills the TX buffer array with sample data.


lora_TXPKTInfo()
Prints information on the last packet transmitted
lora_PacketTXtype
lora_TXDestination
lora_TXSOURCE
lora_TXPacketL
lora_TXpacketCount


lora_TXBuffPrint(lora_LPrint)
Print contents to terminal of the RX buffer from lora_TXStart to lora_TXEnd.
Value passed in lora_Lprint, specifies how buffer is printed;
lora_PrintASC for ASCII
lora_PrintNum for decimal numbers
lora_PrintASC for hexadecimal numbers


lora_Tone(lora_ToneLen, lora_Freq, lora_TXPower)
Generates a FM tone using the LoRa device, length in mS, frequency in hertz, power in dBm (17 to 2). Assumes PWM channel set by
constant lora_PWMCH


lora_DirectSetup()
Set-up device for Direct modulation via LoRa device DIO2 pin, i.e. FM audio tones


lora_TXONLoRa(lora_LTXPower)
Turns on transmit in LoRa packet mode, transmit power in dBm, 2-17.


lora_TXONDirect(lora_LTXPower)
Turns on transmit in direct mode, used for FM tones and FSK modes such as RTTY, transmit power in dBm, 2-17.


lora_TXOFF()
Turns off transmitter, direct and LoRa modes.


lora_Listen(lora_RXTimeout)
Loads Listen time-out in seconds, 0 for no time-out.
On Exit, sets these global variables;
lora_RXStart, Start Location of received data in buffer
lora_RXEnd, End Location of received data in buffer
lora_RXPackettype, 0-255, describes the content of packet or the command\action
lora_RXDestination, 0-255, destination node for packet, 255 is assumed broadcast
lora_RXSource, 0-255, destination node for packet, 255 is assumed broadcast, source node packet is sent from
lora_RXPacketL, 0-255, length of packet received
lora_BackGroundRSSI, 0-255, RSSI register value just before packet received
lora_PacketRSSI, 0-255, RSSI register value during packet reception
lora_PacketSNR, 0-255, SNR register value during packet reception
lora_FRXTimeout = 1 if there has been a transmit time-out, 0 otherwise
lora_FCRCerror = 1 if packet had CRC error, 0 otherwise


lora_RXBuffPrint(lora_LPrint)
Print contents to terminal of the RX buffer from lora_RXStart to lora_RXEnd.
Value passed in lora_Lprint, specifies how buffer is printed;
lora_PrintASC for ASCII
lora_PrintNum for decimal numbers
lora_PrintHEX for hexadecimal numbers


lora_RXPKTInfo()
Prints the following information on the last packet received;
lora_RXPackettype
lora_RXDestination
lora_RXSource
lora_RXPacketL
lora_BackGroundRSSI
lora_PacketRSSI
lora_PacketSNR


lora_RXOFF()
Turns off receive mode, saves power as device goes to low power mode.

*/






/*
**************************************************************************************************
LoRa Arduino Hardware Settings
**************************************************************************************************
*/


//Pins - Set these for the paricular hardware in use
const byte lora_PNSS = 8;	//pin number where the NSS line for the LoRa device is connected.
const byte lora_PLED1 = 7;      //pin number for LED on RFM98 LoRa Shield
const byte lora_PReset = 9;	//pin where LoRa device reset line is connected
const byte lora_PPWMCH = 10;    //pin number for tone generation, connects to LoRa device DIO2.


/*
**************************************************************************************************
Variable definitions
**************************************************************************************************
*/

//byte Variables
byte  lora_RXStart;			//start of packet data in RXbuff
byte  lora_RXEnd;			//end of packet data in RXbuff
byte  lora_TXStart;			//start of packet data in TXbuff
byte  lora_TXEnd;			//end of packet data in TXbuff
byte  lora_FCRCerror;			//flag, set to 1 if there is a packet CRC error
byte  lora_FRXTimeout;			//flag, set to 1 if there is a RX timeout
byte  lora_FTXOK;			//flag, set to 1 if TX OK
byte  lora_FRXOK;			//flag, set to 1 if RX is OK
byte  lora_RXPacketType;		//type number of received packet
byte  lora_TXPacketType;		//type number of packet to send
byte  lora_TXDestination;		//destination address of packet to send
byte  lora_TXSource;			//source address of packet received
byte  lora_RXDestination;		//destination address of received packet
byte  lora_RXSource;			//source address of received packet
byte  lora_BackGroundRSSI;		//measured background noise level
byte  lora_PacketRSSI;			//RSSI of received packet
byte  lora_PacketSNR;			//signal to noise ratio of received packet
byte  lora_FDeviceError;		//flag, set to 1 if RFM98 device error
byte  lora_TXPacketL;			//length of packet to send, includes source, destination and packet type.
byte  lora_RXPacketL;			//length of packet received, includes source, destination and packet type.

//byte arrays
byte  lora_TXBUFF[256];			//buffer for packet to send
byte  lora_RXBUFF[256];			//buffer where received packets are stored

//Integer variables, more than byte
long lora_RXpacketCount;		//count of valid packets received
long lora_CRCerrorcount;		//count of packets received with CRC errors
long lora_TXpacketCount;		//count of packets sent

//Float Variables
double lora_Frequency;			//last programmed frequency



/*
**************************************************************************************************
Constant definitions
**************************************************************************************************
*/

//LoRa names for bandwidth settings
const byte lora_BW7_8 = 0;      //7.8khz
const byte lora_BW10_4 = 16;    //10.4khz
const byte lora_BW15_6 = 32;    //15.6khz
const byte lora_BW20_8 = 48;    //20.8khz
const byte lora_BW31_2 = 64;    //31.2khz
const byte lora_BW41_7 = 80;    //41.7khz
const byte lora_BW62_5 = 96;    //62.5khz
const byte lora_BW125 = 112;    //125khz
const byte lora_BW250 = 128;    //250khz
const byte lora_BW500 = 144;    //500khz

//Spreading Factors
const byte lora_SF6 = 6;
const byte lora_SF7 = 7;
const byte lora_SF8 = 8;
const byte lora_SF9 = 9;
const byte lora_SF10 = 10;
const byte lora_SF11 = 11;
const byte lora_SF12 = 12;

//LORA names for coding rate settings
const byte lora_CR4_5 = 2;	//4:5
const byte lora_CR4_6 = 4;	//4:6
const byte lora_CR4_7 = 6;	//4:7
const byte lora_CR4_8 = 8;	//4:8

//LORA Header Settings
const byte lora_Explicit    = 0;	//Use to set explicit header
const byte lora_Implicit    = 1;	//Use to set implicit header

//Misc definitions
const byte lora_Deviation = 0x52;
const byte lora_LowDoptON = 0x08;       //value to turn low data rate optimization on
const byte lora_LowDoptOFF = 0x00;      //value to turn low data rate optimization off
const byte lora_PrintASC = 0;           //value to cause buffer print to appear as ASCII
const byte lora_PrintNum = 1;           //value to cause buffer print to appear as decimal numbers
const byte lora_PrintHEX = 2;           //value to cause buffer print to appear as hexadecimal numbers


//SX1278 Register names
const byte lora_RegFifo = 0x00;
const byte lora_WRegFifo = 0x80;
const byte lora_RegOpMode = 0x01;
const byte lora_RegFdevLsb = 0x05;
const byte lora_RegFrMsb = 0x06;
const byte lora_RegFrMid = 0x07;
const byte lora_RegFrLsb = 0x08;
const byte lora_RegPaConfig = 0x09;
const byte lora_RegOcp = 0x0B;
const byte lora_RegLna = 0x0C;
const byte lora_RegFifoAddrPtr = 0x0D;
const byte lora_RegFifoTxBaseAddr = 0x0E;
const byte lora_RegFifoRxBaseAddr = 0x0F;
const byte lora_RegFifoRxCurrentAddr = 0x10;
const byte lora_RegIrqFlagsMask = 0x11;
const byte lora_RegIrqFlags = 0x12;
const byte lora_RegRxNbBytes = 0x13;
const byte lora_RegRxHeaderCntValueMsb = 0x14;
const byte lora_RegRxHeaderCntValueLsb = 0x15;
const byte lora_RegRxPacketCntValueMsb = 0x16;
const byte lora_RegRxPacketCntValueLsb = 0x17;
const byte lora_RegPktSnrValue = 0x19;
const byte lora_RegPktRssiValue = 0x1A;
const byte lora_RegRssiValue = 0x1B;
const byte lora_RegFsiMSB = 0x1D;
const byte lora_RegFsiLSB = 0x1E;
const byte lora_RegModemConfig1 = 0x1D;
const byte lora_RegModemConfig2 = 0x1E;
const byte lora_RegSymbTimeoutLsb = 0x1F;
const byte lora_RegPreambleLsb = 0x21;
const byte lora_RegPayloadLength = 0x22;
const byte lora_RegFifoRxByteAddr = 0x25;
const byte lora_RegModemConfig3 = 0x26;
const byte lora_RegPacketConfig2 = 0x31;
const byte lora_TXdefaultpower = 10;



/*
**************************************************************************************************
Library Functions
**************************************************************************************************
*/

void Program_Setup()
{
  //initialise the program
  Serial.print("Program_Setup()");
  Serial.println();
  pinMode(lora_PReset, OUTPUT);		// RFM98 reset line
  digitalWrite(lora_PReset, LOW);	// Reset RFM98
  pinMode (lora_PNSS, OUTPUT);		// set the slaveSelectPin as an output:
  digitalWrite(lora_PNSS, HIGH);
  Serial.begin(9600);			// init serial port for send and receive at 9600 baud
  pinMode(lora_PLED1, OUTPUT);		// for shield LED
  digitalWrite(lora_PLED1, LOW);
  SPI.begin();				// initialize SPI:
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}


byte lora_PrintAsHEX(byte lora_Ltemp)
{
  Serial.print("0x");
  if (lora_Ltemp < 0x10) {
    Serial.print("0");			// print the leading zero
  }
  Serial.print(lora_Ltemp, HEX);	// print the rest
}


void lora_ResetDev()
{
  //resets the LoRa device
  Serial.print("LORA.ResetDev()");
  Serial.println();
  digitalWrite(lora_PReset, LOW);	// take reset line low
  delay(5);
  digitalWrite(lora_PReset, HIGH);	// take it high
}



void lora_Write(byte lora_LReg, byte lora_LData)
{
  //write a byte to a LoRa register
  //Serial.print("lora_Write() ");
  //lora_PrintAsHEX(lora_LReg);
  //Serial.print(" ");
  //lora_PrintAsHEX(lora_LData);
  //Serial.println();
  digitalWrite(lora_PNSS, LOW);		// set NSS low
  SPI.transfer(lora_LReg | 0x80);	// mask address for write
  SPI.transfer(lora_LData);		// write the byte
  digitalWrite(lora_PNSS, HIGH);	// set NSS high
}

byte lora_Read(byte lora_LReg)
{
  //read a byte to a LoRa register
  //Serial.print("lora_Read()");
  //Serial.println();
  byte lora_LRegData;
  digitalWrite(lora_PNSS, LOW);		// set NSS low
  SPI.transfer(lora_LReg & 0x7F);	// mask address for read
  lora_LRegData = SPI.transfer(0);	// read the byte
  digitalWrite(lora_PNSS, HIGH);	// set NSS high
  return lora_LRegData;
}


void lora_Print()
{
  //prints the contents of LORA registers to serial terminal
  Serial.print("lora_Print()");
  Serial.println();
  byte lora_LLoopv1;
  byte lora_LLoopv2;
  byte lora_LReg;
  byte lora_LRegData;
  Serial.print("Reg    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
  Serial.println();
  lora_LReg = 0;
  for (lora_LLoopv1 = 0; lora_LLoopv1 <= 7; lora_LLoopv1++)
  {
    Serial.print("0x");
    Serial.print(lora_LLoopv1, HEX);	// print the register number
    Serial.print("0  ");
    for (lora_LLoopv2 = 0; lora_LLoopv2 <= 15; lora_LLoopv2++)
    {
      lora_LRegData = lora_Read(lora_LReg);
      if (lora_LRegData < 0x10) {
        Serial.print("0");
      }
      Serial.print(lora_LRegData, HEX); // print the register number
      Serial.print(" ");
      lora_LReg++;
    }
    Serial.println();
  }
}

void LED1Flash(int Lflashes, int Lon, int Loff)
{
  //flash a LED
  int Ltemp;
  for (Ltemp = 0; Ltemp < Lflashes; Ltemp++ )
  {
    digitalWrite(lora_PLED1, HIGH);		// turn the LED on (HIGH is the voltage level)
    delay(Lon);					// wait for on period
    digitalWrite(lora_PLED1, LOW);		// turn the LED on (HIGH is the voltage level)
    delay(Loff);				// wait for off period
  }
}

void lora_SetFreq(float lora_LFreq)
{
  //set the LoRa frequency
  byte lora_LFMsb, lora_LFMid, lora_LFLsb;
  long lora_LLongFreq;
  lora_LLongFreq = ((lora_LFreq * 1000000) / 61.03515625);
  Serial.print("lora_setFreq() ");
  Serial.print(lora_LFreq);
  Serial.print(" 0x");
  Serial.print(lora_LLongFreq, HEX);
  Serial.println();
  lora_LFMsb =  lora_LLongFreq >> 16;
  lora_LFMid = (lora_LLongFreq & 0x00FF00) >> 8;
  lora_LFLsb =  (lora_LLongFreq & 0x0000FF);
  lora_Write(lora_RegFrMsb, lora_LFMsb);
  lora_Write(lora_RegFrMid, lora_LFMid);
  lora_Write(lora_RegFrLsb, lora_LFLsb);
}


float lora_GetFreq()
{
  //get the current set LoRa frequency
  //Serial.print("lora_GetFreq() ");
  //Serial.println();
  byte lora_LFMsb, lora_LFMid, lora_LFLsb;
  unsigned long lora_Ltemp;
  float lora_Ltemp1;
  lora_LFMsb = lora_Read(lora_RegFrMsb);
  lora_LFMid = lora_Read(lora_RegFrMid);
  lora_LFLsb = lora_Read(lora_RegFrLsb);
  lora_Ltemp = ((lora_LFMsb * 0x10000ul) + (lora_LFMid * 0x100ul) + lora_LFLsb);
  lora_Ltemp1 = ((lora_Ltemp * 61.03515625) / 1000000ul);
  return lora_Ltemp1;
}


byte lora_CheckDevice()
{
  //check there is a device out there, program the 3 frequency setting registers and read back
  Serial.print("LORA.CheckDevice() ");
  Serial.println();
  byte lora_Lvar1, lora_Lvar2, lora_Lvar3;
  lora_Write(lora_RegFrMsb, 0x5A);
  lora_Write(lora_RegFrMid, 0xAA);
  lora_Write(lora_RegFrLsb, 0xA5);
  lora_Lvar1 = lora_Read(lora_RegFrMsb);	// Read RegFrMSB
  lora_Lvar2 = lora_Read(lora_RegFrMid);	// Read RegFrMid
  lora_Lvar3 = lora_Read(lora_RegFrLsb);	// Read RegFrLSB
  if (lora_Lvar1 != 0x5A || lora_Lvar2 != 0xAA || lora_Lvar3 != 0xA5)
  { return false;
    lora_FDeviceError = 0;
  }
  else
  { return true;
    lora_FDeviceError = 1;
  }
}

void lora_Setup()
{
  //initialize LoRa device registers and check its responding
  Serial.print("lora_Setup()");
  Serial.println();
  lora_ResetDev();				// Clear all registers to default
  lora_Write(lora_RegOpMode, 0x08);		// RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOcp, 0x0B);		// RegOcp
  lora_Write(lora_RegLna, 0x23);		// RegLna
  lora_Write(lora_RegSymbTimeoutLsb, 0xFF);	// RegSymbTimeoutLsb
  lora_Write(lora_RegPreambleLsb, 0x0C);	// RegPreambleLsb, default
  lora_Write(lora_RegFdevLsb, lora_Deviation);	// LSB of deviation, 5kHz
  if (!lora_CheckDevice())
  {
    Serial.print("LoRa Device Not Working");
    Serial.println();
  }
}


byte lora_TXONDirect(byte lora_LTXPower)
{
  //turns on transmitter,in direct mode for FSK and audio  power level is from 2 to 17
  Serial.print("lora_TXONDirect(");
  Serial.print(lora_LTXPower);
  Serial.print(")");
  Serial.println();
  byte lora_Lvar1;
  lora_Lvar1 = lora_LTXPower + 0xEE;		// has effect of converting 17 into 15
  lora_Write(lora_RegPaConfig, lora_Lvar1);	// set TX power
  lora_Write(lora_RegOpMode, 0x0B);		// TX on direct mode, low frequency mode
}

void lora_TXOFF()
{
  //turns off transmitter
  Serial.print("lora_TXOFF()");
  Serial.println();
  lora_Write(lora_RegOpMode, 0x08);             // TX and RX to sleep, in direct mode
  digitalWrite(lora_PLED1, LOW);		// turn LED off
}

void lora_DirectSetup()
{
  //setup LoRa device for direct modulation mode
  Serial.print("lora_DirectSetup()");
  Serial.println();
  lora_Write(lora_RegOpMode, 0x08);
  lora_Write(lora_RegPacketConfig2, 0x00);	// set continuous mode
}


void lora_Tone(int lora_LFreq, int lora_LToneLen, int lora_LTXPower )
{
  //Transmit an FM tone
  Serial.print("lora_Tone()");
  Serial.println();
  lora_DirectSetup();
  lora_TXONDirect(lora_LTXPower);		// TXON, power 10
  lora_Write(lora_RegFdevLsb, lora_Deviation);	// We are generating a tone so set the deviation, 5kHz
  tone(lora_PPWMCH, lora_LFreq);
  delay(lora_LToneLen);
  lora_TXOFF();
}



void lora_SetModem(byte lora_LBW, byte lora_LSF, byte lora_LCR, byte lora_LHDR, byte lora_LLDROPT)
{
  //setup the LoRa modem parameters
  Serial.print("lora_SetModem()");
  Serial.println();
  byte lora_Lvar1, lora_Lvar2;
  lora_Lvar1 = lora_LBW + lora_LCR + lora_LHDR;  // calculate value of RegModemConfig1
  lora_Lvar2 = lora_LSF * 16 + 7;		 // calculate value of RegModemConfig2, ($07; Header indicates CRC on, RX Time-Out MSB = 11
  lora_Write(lora_RegOpMode, 0x08);		 // RegOpMode, need to set to sleep mode before configure for LoRa mode
  lora_Write(lora_RegOpMode, 0x88);		 // goto LoRa mode
  lora_Write(lora_RegModemConfig1, lora_Lvar1);
  lora_Write(lora_RegModemConfig2, lora_Lvar2);
  lora_Write(lora_RegModemConfig3, lora_LLDROPT);
}

void lora_PrintModem()
{
  //Print the LoRa modem parameters
  Serial.print("lora_PrintModem() ");
  byte lora_Ltemp;
  lora_Ltemp = lora_Read(lora_RegModemConfig1);
  lora_PrintAsHEX(lora_Ltemp);
  Serial.print(" ");
  lora_Ltemp = lora_Read(lora_RegModemConfig2);
  lora_PrintAsHEX(lora_Ltemp);
  Serial.print(" ");
  lora_Ltemp = lora_Read(lora_RegModemConfig3);
  lora_PrintAsHEX(lora_Ltemp);
  Serial.println();
}

void lora_PrintPower()
{
  //print current set power
  Serial.print("lora_PrintPower() ");
  byte lora_Ltemp;
  lora_Ltemp = lora_Read(lora_RegPaConfig);
  lora_Ltemp = lora_Ltemp - 0xEE;
  Serial.print(lora_Ltemp);
  Serial.println();
}

void lora_FillTX()
{
  //fill TXbuff with test data
  Serial.print("lora_FillTX() ");
  byte lora_LLoop;
  byte lora_LData;
  lora_TXEnd = 255;                                  // so a single byte packet ends up with lora_TXend of 0
  lora_TXStart = 0;
  for (lora_LLoop = 0x41; lora_LLoop <= 0x5A; lora_LLoop++)
  {
    lora_TXEnd++;
    lora_TXBUFF[lora_TXEnd] = lora_LLoop;
  }
  Serial.print(lora_TXStart);
  Serial.print(" ");
  Serial.print(lora_TXEnd);
  Serial.println();

}

byte lora_TXBuffPrint(byte lora_LPrint)
{
  //Print contents of TX buffer as ASCII,decimal or HEX
  Serial.print("lora_TXBuffPrint(");
  Serial.print(lora_LPrint);
  Serial.print(") ");
  Serial.print(lora_TXStart);
  Serial.print(" ");
  Serial.print(lora_TXEnd);
  Serial.print(" Start>>");                         // print start marker so we can be sure where packet data starts

  byte lora_LLoop;
  byte lora_LData;

  for (lora_LLoop = lora_TXStart; lora_LLoop <= lora_TXEnd; lora_LLoop++)
  {
    lora_LData = lora_TXBUFF[lora_LLoop];
    if (lora_LPrint == 0)
    {
      Serial.write(lora_LData);
    }
    if (lora_LPrint == 1)
    {
      Serial.print(lora_LData);
      Serial.print(" ");
    }

    if (lora_LPrint == 2)
    {
      Serial.print(lora_LData, HEX);
      Serial.print(" ");
    }
  }
  Serial.print("<<End");                              // print end marker so we can be sure where packet data ends
  Serial.println();
}



byte lora_RXBuffPrint(byte lora_LPrint)
{
  //Print contents of RX buffer as ASCII,decimal or HEX
  Serial.print("lora_RXBuffPrint(");
  Serial.print(lora_LPrint);
  Serial.print(") ");
  Serial.print(lora_RXStart);
  Serial.print(" ");
  Serial.print(lora_RXEnd);
  Serial.print(" Start>>");                           // print start marker so we can be sure where packet data starts

  byte lora_LLoop;
  byte lora_LData;
  byte lcount;

  for (lora_LLoop = lora_RXStart; lora_LLoop <= lora_RXEnd; lora_LLoop++)
  {
    lcount++;
    lora_LData = lora_RXBUFF[lora_LLoop];
    if (lora_LPrint == 0)
    {
      Serial.write(lora_LData);
    }
    if (lora_LPrint == 1)
    {
      Serial.print(lora_LData);
      Serial.print(" ");
    }

    if (lora_LPrint == 2)
    {
      Serial.print(lora_LData, HEX);
      Serial.print(" ");
    }
  }
  Serial.print("<<End   ");                                 // print end marker so we can be sure where packet data ends
  Serial.println();
}

void lora_RXOFF()
{
  //turns off receiver
  //Serial.print("lora_RXOFF()");
  //Serial.println();
  lora_Write(lora_RegOpMode, 0x89);                         // TX and RX to sleep, in direct mode
}

void lora_TXPKTInfo()
{
  //print the information for TX packet last sent
  Serial.print("lora_TXPKTInfo() ");
  Serial.print("TXtype,");
  Serial.print(lora_TXPacketType);
  Serial.print(",TXDestination,");
  Serial.print(lora_TXDestination);
  Serial.print(",TXSource,");
  Serial.print(lora_TXSource);
  Serial.print(",TXPacketLength,");
  Serial.print(lora_TXPacketL);
  Serial.print(",TXPacketCount,");
  Serial.print(lora_TXpacketCount);
  Serial.println();
}


byte lora_TXONLoRa(byte lora_LTXPower)
{
  //turns on LoRa transmitter, Sends packet, power level is from 2 to 17
  Serial.print("lora_TXONLoRa(");
  Serial.print(lora_LTXPower);
  Serial.print(")");
  Serial.println();
  byte lora_Lvar1;
  lora_Lvar1 = lora_LTXPower + 0xEE;                    // has effect of converting 17 into 15
  lora_Write(lora_RegPaConfig, lora_Lvar1);             // set TX power
  lora_Write(lora_RegOpMode, 0x8B);                     // TX on direct mode, low frequency mode
  digitalWrite(lora_PLED1, HIGH);			// turn LED on
}


void lora_Send(byte lora_LTXStart, byte lora_LTXEnd, byte lora_LTXPacketType, byte lora_LTXDestination, byte lora_LTXSource, long lora_LTXTimeout, byte lora_LTXPower)
{
  //fills FIFO with 3 header bytes, then from lora_TXBUFF(256) starting at lora_TXStart ending at lora_TXEnd,maximum of 252 btes
  Serial.print("lora_Send() ");
  Serial.print(lora_LTXStart);
  Serial.print(" ");
  Serial.print(lora_LTXEnd);
  Serial.println();
  long lora_Lvar1;
  byte lora_LRegData;
  byte lora_LTXPacketL;
  lora_TXStart = lora_LTXStart;
  lora_TXEnd = lora_LTXEnd;
  lora_TXDestination = lora_LTXDestination;
  lora_TXPacketType = lora_LTXPacketType;
  lora_TXSource = lora_LTXSource;
  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegIrqFlagsMask, 0xF7);
  lora_Write(lora_RegFifoTxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);  	// start burst read

  digitalWrite(lora_PNSS, LOW);			// Set NSS low
  SPI.transfer(lora_WRegFifo);			// address for burst write
  SPI.transfer(lora_LTXPacketType);        	// Write the packet type
  SPI.transfer(lora_LTXDestination);       	// Destination node
  SPI.transfer(lora_LTXSource);            	// Source node
  lora_LTXPacketL = 3;

  for (lora_Lvar1 = lora_TXStart;  lora_Lvar1 <= lora_TXEnd; lora_Lvar1++)
  {
    lora_LTXPacketL++;
    if (lora_LTXPacketL > 253)              	 // check for overlong packet here, wraps around from limit at 251 to 0
    {
      Serial.print("ERROR,PacketatLimit ");
      lora_LTXPacketL--;                         // remove increment to packet length
      break;
    }
    lora_LRegData = lora_TXBUFF[lora_Lvar1];
    SPI.transfer(lora_LRegData);
  }

  digitalWrite(lora_PNSS, HIGH);		// finish the burst write
  lora_TXPacketL = lora_LTXPacketL;
  lora_Write(lora_RegPayloadLength, lora_LTXPacketL);
  lora_Lvar1 = 0;
  Serial.print("Transmit Timeout ");
  Serial.print(lora_LTXTimeout);
  Serial.print(" Seconds");
  Serial.println();
  lora_LTXTimeout = lora_LTXTimeout * 945;	// convert seconds to mS, delay in TX done loop is 1ms
  lora_TXONLoRa(lora_LTXPower);

  do
  {
    delay(1);
    lora_Lvar1++;
    lora_LRegData = lora_Read(lora_RegIrqFlags);
  }
  while (lora_Lvar1 < lora_LTXTimeout && lora_LRegData == 0) ;  // use a timeout counter, just in case the TX sent flag is missed

  lora_TXOFF();

  if (lora_LRegData == 8)
  {
    Serial.print("Packet Sent");
    Serial.println();
    lora_FTXOK = 1;
    lora_TXpacketCount++;
  }
  else
  {
    Serial.print("ERROR,TXtimeout");
    Serial.println();
    lora_FTXOK = 0;
  }
}


void lora_RXPKTInfo()
{
  //print the information for packet last received
  byte lora_Lvar1;
  char lora_LChar;
  Serial.print("lora_RXPKTInfo() ");
  Serial.print("RXtype,");
  Serial.print(lora_RXPacketType);
  Serial.print(",Destination,");
  Serial.print(lora_RXDestination);
  Serial.print(",Source,");
  Serial.print(lora_RXSource);
  Serial.print(",Length,");
  Serial.print(lora_RXPacketL);
  Serial.print(",RXCount,");
  Serial.print(lora_RXpacketCount);
  Serial.print(",CRCErrors,");
  Serial.print(lora_CRCerrorcount);
  Serial.println();

  lora_Lvar1 = 137 - lora_PacketRSSI;
  Serial.print("RSSI -");
  Serial.print(lora_Lvar1);
  Serial.print("dBm");
  Serial.println();

  lora_Lvar1 = 137 - lora_BackGroundRSSI;
  Serial.print("Noise -");
  Serial.print(lora_Lvar1);
  Serial.print("dBm");
  Serial.println();

  if (lora_PacketSNR > 127)
  {
    lora_Lvar1 =  (255 - lora_PacketSNR) / 4;
    lora_LChar = '-';
  }
  else
  {
    lora_Lvar1 = lora_PacketSNR / 4;
    lora_LChar = '+';
  }

  Serial.print("PacketSNR ");
  Serial.print(lora_LChar);
  Serial.print(lora_Lvar1);
  Serial.print("dB");
  Serial.println();

}


void lora_Listen(long lora_LRXTimeout)
{
  //'puts SX1278 into listen mode and receives packet exits with packet in array lora_RXBUFF(256)
  Serial.print("lora_Listen() ");
  byte lora_Lvar1, lora_LRegData, lora_LLoop;
  long lora_Lvar2;
  lora_FRXOK = 1;                                        // Start assuming packet received OK, unless flag cleared
  lora_FCRCerror = 0;
  lora_FRXTimeout = 0;
  lora_RXPacketL = 0;
  lora_RXPacketType = 0;
  lora_RXDestination = 0;
  lora_RXSource = 0;
  lora_RXStart = 0;
  lora_RXEnd = 0;
  lora_Write(lora_RegOpMode, 0x09);
  lora_Write(lora_RegFifoRxBaseAddr, 0x00);
  lora_Write(lora_RegFifoAddrPtr, 0x00);
  lora_Write(lora_RegIrqFlagsMask, 0x9F);                // only allow rxdone and crc error
  lora_Write(lora_RegIrqFlags, 0xFF);
  lora_Write(lora_RegOpMode, 0x8D);
  //delay(2); //following a device reset a short dealy is needed here to allow the BackGroundRSSI to aquire a valid reading
  lora_BackGroundRSSI = lora_Read(lora_RegRssiValue);    // get the background noise level

  if (lora_LRXTimeout == 0)
  {

    do
    {
      lora_LRegData = lora_Read(lora_RegIrqFlags);
      lora_Lvar1 = (lora_LRegData & 0x40);               // mask off RX in progress flag
    }
    while (lora_Lvar1 == 0);                             // wait until RX not in progress, lora_Lvar1 is 0 if not RX in progress
  }
  else
  {
    Serial.print("Timeout ");
    Serial.print(lora_LRXTimeout);
    Serial.print(" Secs");
    Serial.println();
    lora_LRXTimeout = lora_LRXTimeout * 945;             // convert to number of 1mS steps
    lora_Lvar2 = 0;
    do
    {
      delay(1);
      lora_LRegData = lora_Read(lora_RegIrqFlags);
      lora_Lvar1 = (lora_LRegData & 0x40);               // mask off RX in progress flag
      lora_Lvar2++;
    }
    while  (lora_Lvar1  == 0 && lora_Lvar2 < lora_LRXTimeout);       // loop until RX not in progress or a timeout
  }

  lora_RXOFF();

  if (lora_Lvar2 == lora_LRXTimeout)
  {
    lora_FRXOK = 0;
    lora_FRXTimeout = 1;
    Serial.print("Nothing Received");
    Serial.println();
    return;
  }

  lora_RXPacketL = lora_Read(lora_RegRxNbBytes);
  lora_PacketRSSI = lora_Read(lora_RegPktRssiValue);
  lora_PacketSNR = lora_Read(lora_RegPktSnrValue);
  lora_RXpacketCount++;

  digitalWrite(lora_PLED1, HIGH);

  lora_Lvar1 = lora_LRegData & 0x20;                      // now check for a CRC error, lora_LRegdata has last value of int flags read
  if (lora_Lvar1 != 0)
  {
    lora_FRXOK = 0;
    lora_FCRCerror = 1;
    Serial.print("CRCFail !");
    Serial.println();
    lora_CRCerrorcount++;
    digitalWrite(lora_PLED1, LOW);
    return;
  }

  lora_Write(lora_RegFifoAddrPtr, 0x00);  	          // set FIFO ptr

  digitalWrite(lora_PNSS, LOW);		                  // start the burst read
  SPI.transfer(lora_RegFifo);			          // address for burst read
  lora_RXPacketType = SPI.transfer(0);
  lora_RXDestination = SPI.transfer(0);
  lora_RXSource = SPI.transfer(0);
  lora_RXStart = 0;
  lora_RXEnd = lora_RXPacketL - 4;                       // adjust for destination, packettype and source bytes

  Serial.print("RXbuff ");
  Serial.print(lora_RXStart);
  Serial.print(" ");
  Serial.print(lora_RXEnd);
  Serial.println();
  for (lora_Lvar1 = lora_RXStart; lora_Lvar1 <= lora_RXEnd; lora_Lvar1++)
  {
    lora_LRegData = SPI.transfer(0);
    lora_RXBUFF[lora_Lvar1] = lora_LRegData;
  }
  digitalWrite(lora_PNSS, HIGH);		           // finish, turn off LoRa device
  digitalWrite(lora_PLED1, LOW);                           // and turn LED off
}



