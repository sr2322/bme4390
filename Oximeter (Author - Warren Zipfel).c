/////////////////////////////////////////////////////////////////////
// Oximeter.c - firmware for BME4390 oximeter which uses a PIC18F2455
//              MCU and a Maxim MAX1303 16 bit 4 channel SPI ADC
//              The results are displayed on the LCD or sent
//              to the PC via USB.
//////////////////////////////////////////////////////////////////////

#include <18F2455.h>
#include <stdlib.h>
#include <math.h>
#fuses HSPLL,NOWDT,NOPROTECT,NOLVP,NODEBUG,USBDIV,PLL5,CPUDIV1,VREGEN,MCLR
#use delay(clock=48000000)

// In usb.c/h - tells the CCS PIC USB firmware to include HID handling code.
#DEFINE USB_HID_DEVICE  TRUE
#define USB_EP1_TX_ENABLE  USB_ENABLE_INTERRUPT   //turn on EP1 for IN bulk/interrupt transfers
#define USB_EP1_TX_SIZE    64  //allocate 64 bytes in the hardware for transmission   

#define USB_EP1_RX_ENABLE  USB_ENABLE_INTERRUPT   //turn on EP1 for OUT bulk/interrupt transfers
#define USB_EP1_RX_SIZE    64    // allocate 64 bytes in the hardware for reception   

#include <pic18_usb.h>            // Microchip 18Fxx5x hardware layer for usb.c
#include "PIC18F2455_desc64.h"    // HID descriptor file
#include <usb.c>                  // handles usb setup tokens and get descriptor reports

// HID command defines
#define  CMD_FLASH_LED           1  //  flash the LED as a failsafe check
#define  CMD_NORMAL_READ         2  //  one read of the 4 channels and display SpO2
#define  CMD_DISPLAY_VOLTAGES    3  //  voltage display mode  
#define  CMD_GET_DATA            4  //  return the 64 int16 values 32 at a time

// CMD_GET_DATA subcommands
#define SEND_DC_PART             1  // return the 2 DC values (4 bytes total)
#define SEND_AC660_1             2  // send the 1st 32 AC660 values (64 bytes)
#define SEND_AC660_2             3  // send the 2nd 32 AC660 values (64 bytes)
#define SEND_AC940_1             4  // send the 1st 32 AC940 values (64 bytes)
#define SEND_AC940_2             5  // send the 2nd 32 AC940 values (64 bytes)

// set PWM module to 1KHz to run the oximeter
#USE PWM(OUTPUT=PIN_C2, FREQUENCY=3000, DUTY=50)

#define LED           PIN_A0    // test LED define
#define MEASURE_PIN   PIN_B5    // pin start measurement cycle 

// pin defines for the 16x2 LCD - used by flex_lcd.c
#define LCD_E         PIN_A1  
#define LCD_RS        PIN_A2  
#define LCD_DB4       PIN_B0  
#define LCD_DB5       PIN_B1  
#define LCD_DB6       PIN_B2  
#define LCD_DB7       PIN_B3 

#include <Flex_lcd.c>

// defines for MAX1303 ADC pins
#define MAX1303_CS    PIN_A3
#define MAX1303_DIN   PIN_A4
#define MAX1303_SSTRB PIN_A5
#define MAX1303_SCLK  PIN_C0
#define MAX1303_DOUT  PIN_C1

// ADC conversion factors
#define V_CONVERT   0.000125  // (65535 / (2 * 4.096V))
#define ADC_OFFSET  32768     // 65536/2

// macro define for creating a clock tick
#define  MAX1303_CLKTIC  output_high(MAX1303_SCLK); delay_us(10); output_low(MAX1303_SCLK);

// Function prototypes
void  SetupMAX1303(void);      
int16 ReadMAX1303(int8 channel);
void  MeasureSignals(void);
void  DisplaySPO2(void);
void  DisplayVoltage(void);
void  BlinkLED(void);
void  DataLoadStatus(void);
void  LoadData(void);
void  ExecuteCmd(void);

// # measurements - use power of 2 to make division fast
#define NUM  64    // measure ~10 times per second for ~6 seconds - 64 pts total
#define NAVE 32    // number of times to average the ac signal

// global variables
int16 AC660[NUM] = { 0 };  // storage for 660 nm AC signal
int16 AC940[NUM] = { 0 };  // storage for 940 nm AC signal
int16 DC660 = 0;           // storage for 660 nm DC value
int16 DC940 = 0;           // storage for 940 nm DC value
int8  cmd[NUM];            // buffer for command string received from the PC (via usb)
int8  SequenceCode = 0; 
float spo2 = 0.0;


void main(void) {
   usb_init();                // initialize usb communications
   lcd_init();                // initialize the LCD. 
   lcd_putc("\fBME4390 Oximeter");  
   output_high(MAX1303_CS);   // set chip select high
   output_low(MAX1303_SCLK);  // set the clk pin low
   SetupMAX1303();            // initialize MAX1303
   BlinkLED();                // blink twice to indicate setup 
   BlinkLED();                // was successful.
     
  // enter the endless while
   while(TRUE)   {
     if (usb_enumerated()) {   // if on the USB bus wait for a command
        if (usb_kbhit(1)) {   // Check for USB commands
           usb_gets(1, cmd, 64, 100);
           ExecuteCmd();
        }  // kbhit if end
    } // enumerated if end
    else {
      if (MEASURE_PIN) {   // measure SpO2 if pin is high
          MeasureSignals();
          DisplaySPO2();
     }
     else {
         DisplayVoltage();
     }
    }  // else end
   } // while (1) end
} // main end

///////////////////////////////////////////////////////////////////////////
//   Executes a command based on the 1st byte in the HID string 
//   byte 0 = cmd,  byte 1-63 = data bytes  
/////////////////////////////////////////////////////////////////////////////
void ExecuteCmd(void) 
{
 
  switch (cmd[0]) {
        case CMD_FLASH_LED:   
            usb_puts(1, cmd, 64, 100);    // echo string
            BlinkLED();
            break;
            
        case CMD_NORMAL_READ:
            MeasureSignals();
            DisplaySPO2();
            break;
           
        case CMD_DISPLAY_VOLTAGES:
           DisplayVoltage();
           break;
          
        case CMD_GET_DATA:
           LoadData();
           usb_puts(1, cmd, 64, 100);    // echo string
           DataLoadStatus();
           break;
  }
}
   

////////////////////////////////////////////////////////////////////////
//  Set the mode register to internal clock
//  The config register is fine with the default values
//  which are +/- 4.095volts and single-ended inputs
//////////////////////////////////////////////////////////////////
void SetupMAX1303(void)
{
 int8 ModeByte = 0b10101000;       //  use the internal clock

 // write the mode byte
 output_low(MAX1303_CS);
 for(int8 i=0;i<8;i++) {
    if(bit_test(ModeByte, 7-i)) {
      output_high(MAX1303_DIN); 
    }
    else { 
      output_low(MAX1303_DIN);
    }
    delay_us(2);
    MAX1303_CLKTIC;   // call clock tic macro
    delay_us(2);
 }
 output_high(MAX1303_CS);
 output_low(MAX1303_DIN);
}

//////////////////////////////////////////////////////////////
// Read a channel on the MAX1303 and return the value
/////////////////////////////////////////////////////////////
int16 ReadMAX1303(int8 channel)
{
  int8 CommandByte = 0b10000000;
  channel = channel<<4; 
  CommandByte = CommandByte|channel; 
  output_low(MAX1303_CS);
  for(int8 i=0;i<8;i++) {
    if(bit_test(CommandByte, 7-i)) {
       output_high(MAX1303_DIN); 
    }
    else { 
      output_low(MAX1303_DIN);
    }
    delay_us(2);
    MAX1303_CLKTIC;   
    delay_us(2);
 }
 
 output_low(MAX1303_DIN);
 output_high(MAX1303_CS);
 while(!input(MAX1303_SSTRB)); // wait for SSTRB to go high
 output_low(MAX1303_CS);
 
 // read data
 int16 ADCval = 0;
 for(int8 i=0; i<16; i++)  {
    output_high(MAX1303_SCLK);   
    delay_us(2);
    if(input(MAX1303_DOUT)) bit_set(ADCval, 15-i);
    output_low(MAX1303_SCLK);   
    delay_us(2);
 }
 output_high(MAX1303_CS);
 return ADCval; 
}

//////////////////////////////////////////////////////////////
// Measure the 4 signals for ~6.4 seconds
// Store the AC signals for analysis to find peak-peak value
// Sum the DC signals in a 32 bit integer and then find the mean
/////////////////////////////////////////////////////////////
void MeasureSignals(void)
{
 char szBuf[18];
 int32 DC660sum = 0;
 int32 DC940sum = 0; 
 lcd_putc("\fMeasuring...    ");
 int32 mean660 = 0;
 int32 mean940 = 0;
 for(int8 i = 0; i < NUM; i++)  {
    mean660 = 0;
    mean940 = 0;
    for(int j = 0; j < NAVE; j++) {
      mean940 += ReadMAX1303(0);
      mean660 += ReadMAX1303(2);
    }
    AC940[i] = (int16) (mean940 >> 5);
    AC660[i] = (int16) (mean660 >> 5);
    DC940sum += ReadMAX1303(1);
    DC660sum += ReadMAX1303(3);
    
    // measured the time to execute the above code and it was 2.4868 ms
    delay_ms(79);
    output_high(PIN_B4);
    delay_us(500);
    output_low(PIN_B4);
    sprintf(szBuf, "\n  ---- %02u ----  ", i);     // display i
    for(int8 c = 0; c < strlen(szBuf);c++) lcd_putc(szBuf[c]); 
 }
 DC660 = (int16) (DC660sum >> 6);   // divide by 64 to get the 
 DC940 = (int16) (DC940sum >> 6);   // average DC value
}

//////////////////////////////////////////////////////////////
// Calulate and display the SPO2 value
// Uses a simple max and min search and the difference
// is taken as the AC amplitude.
/////////////////////////////////////////////////////////////
void  DisplaySPO2(void)
{
    // find the min and max of the AC signals
    int16 AC660max = 0; 
    int16 AC660min = 65535;
    int16 AC940max = 0; 
    int16 AC940min = 65535;
   
    for(int8 i = 0; i < NUM; i++) {
        AC660max = (AC660max < AC660[i]) ? AC660[i] : AC660max; 
        AC940max = (AC940max < AC940[i]) ? AC940[i] : AC940max; 
        AC660min = (AC660min > AC660[i]) ? AC660[i] : AC660min; 
        AC940min = (AC940min > AC940[i]) ? AC940[i] : AC940min; 
    }
   
    int16 AC660diff = AC660max - AC660min;
    int16 AC940diff = AC940max - AC940min;
    
    // You need to type cast as floats
    float Numerator = (float) AC660diff * (float) DC940;
    float Denominator = (float) AC940diff * (float) DC660;
    
    // Calculate R
    float R = Numerator/Denominator;   
    
    // estimate SP02 using the standard industry formula
    spo2 = 110.0 - 25.0 * R;
    
    // display the result and wait 3 seconds so it can be read
    char szBuf[32] = { 0 };
    if(spo2 > 101.0|| spo2 < 85.0) {
         sprintf(szBuf,"\f SPO2 Read Error");
    }
    else {
        sprintf(szBuf,"\f SPO2 = %3.2f\n  R = %2.4f",spo2, R);
    }
    for(int8 i = 0; i < strlen(szBuf); i++) lcd_putc(szBuf[i]);
    delay_ms(3000);  // delay so the value can be read
}

//////////////////////////////////////////////////////////////
// Display the voltage readings if PIN_B5 is held high
/////////////////////////////////////////////////////////////
void  DisplayVoltage(void)
{
   float voltage[4];
   char  szBuf[32];
   int32 mean[4] = {0};
   int16 val[4] = {0}; 
   
   for(int i = 0; i < nave; i++) {
      mean[0] += (int32) ReadMAX1303(0);
      mean[1] += (int32) ReadMAX1303(1);
      mean[2] += (int32) ReadMAX1303(2);
      mean[3] += (int32) ReadMAX1303(3);
    }
    
   for(int8 i = 0; i < 4; i++) {
       val[i] = (int16) (mean[i] >> 4);
       voltage[i] = V_CONVERT * (float) (val[i] - ADC_OFFSET);
   }
   
   sprintf(szBuf,"\fIR: %1.3f %1.3f\nRed:%1.3f %1.3f", voltage[0], voltage[1], voltage[2], voltage[3]);
   for(int8 i = 0; i < strlen(szBuf); i++) lcd_putc(szBuf[i]);
   delay_ms(1000);
}

//////////////////////////////////////////////////////
// Load the data in the cmd array and return it.
// Incoming cmd[1] the sequence number. 
// The function returns:
// if SequenceCode = 1 :SEND_DC_PART
//    cmd[0] = HIBYTE(DC660) cmd[1] = LOBYTE(DC660)
//    cmd[2] = HIBYTE(DC940) cmd[3] = LOBYTE(DC940)
//    cmd[4] = whole number part of Sp02 
//    cmd[5] = decimal part of SpO2
// 
// if SequenceCode = 2 : SEND_AC660_1 first 32 values (HIBYTE then LOBYTE) of the AC660 array
// if SequenceCode = 3 : SEND_AC660_2 next 32 values of AC660
// if SequenceCode = 4 : SEND_AC940_1 first 32 values (HIBYTE then LOBYTE) of the AC940 array
// if SequenceCode = 5 : SEND_AC940_2 next 32 values of AC940
////////////////////////////////////////////////////////
void LoadData(void)
{
  SequenceCode = cmd[1];
  int8 index = 0; 
  // to break the 16 bit int into two 8 bit int's use make8(var, offset)
  // Make8 extracts the byte at offset from var. 
  // Same as: i8 = (((var >> (offset*8)) & 0xff) except it is done with a single byte move
  
  switch (SequenceCode) {
    case SEND_DC_PART:
         cmd[0] = make8(DC660, 1);
         cmd[1] = make8(DC660, 0);
         cmd[2] = make8(DC940, 1);
         cmd[3] = make8(DC940, 0);
         // send the PIC measured spo2 value
         cmd[4] = (int8) floor(spo2);  
         cmd[5] = (int8) (1000.0 * (spo2 - floor(spo2)));
         break;
  
    case SEND_AC660_1:
         index = 0;
         for(int8 i = 0; i < 32; i++) {  
             cmd[index++] = MAKE8(AC660[i], 1);     
             cmd[index++] = MAKE8(AC660[i], 0); 
         }
         break;
  
    case SEND_AC660_2:
         index = 0;
         for(int8 i = 32; i < 64; i++) {  
             cmd[index++] = MAKE8(AC660[i], 1);     
             cmd[index++] = MAKE8(AC660[i], 0); 
         }
         break;
  
    case SEND_AC940_1:
         index = 0;
         for(int8 i = 0; i < 32; i++) {  
             cmd[index++] = MAKE8(AC940[i], 1);     
             cmd[index++] = MAKE8(AC940[i], 0); 
         }
         break;
   
  
    case SEND_AC940_2:
         index = 0;
         for(int8 i = 32; i < 64; i++) {  
             cmd[index++] = MAKE8(AC940[i], 1);     
             cmd[index++] = MAKE8(AC940[i], 0); 
         }
         break;
     
    default:
        lcd_putc("/fBad Data load/ncode sent."); 
        break;
  }
  
}


///////////////////////////////////////////////////////////////
// Displays the data segment number after sending it to the PC
//////////////////////////////////////////////////////////////
void DataLoadStatus(void)
{
  
   switch (SequenceCode) {
      case SEND_DC_PART:
          lcd_putc("\fDC data sent\nto PC");
          break;
  
      case SEND_AC660_1:
          lcd_putc("\f1st half of AC660\ndata sent to PC");
          break;
  
      case SEND_AC660_2:
          lcd_putc("\f2nd half of AC660\ndata sent to PC");
          break;
  
     case SEND_AC940_1:
         lcd_putc("\f1st half of AC940\ndata sent to PC");
         break;
  
     case SEND_AC940_2:
         lcd_putc("\f2nd half of AC940\ndata sent to PC");
         break;
        
     default:
         lcd_putc("/fBad Data load/ncode sent."); 
         break;
   }
}


//////////////////////////////////////////////////////////////
// Blink the LED for 500 ms 
/////////////////////////////////////////////////////////////
void BlinkLED(void)
{
   output_high(LED);
   delay_ms(500);
   output_low(LED);
}



