//******************************************************************************
//  Praveen Aroul
//  HealthTech, MHR
//  (C) Texas Instruments Inc., 2013
//  All Rights Reserved.
//  Built with IAR Workbench 5.52.1
//ver 1.3
//******************************************************************************

#include <intrinsics.h>
#include <string.h>
#include <math.h>

#include "descriptors.h"

#include "device.h"
#include "types.h"               //Basic Type declarations
#include "usb.h"                 //USB-specific functions

#include "HAL_UCS.h"
#include "HAL_PMM.h"

#include "UsbCdc.h"
#include "usbConstructs.h"

#include "lmt70_main.h"
#include "AFE44xx.h"
#include "CalibrationRoutine.h"
#include "MCU_BASE.h"
#include "OLED_2864HSWEG02.h"
#include "LMT70_sensor.h"


//==============================================================================
//Global flags set by events
volatile BYTE bCDCDataReceived_event = FALSE;   //Indicates data has been received without an open rcv operation

#define MAX_STR_LENGTH 64
char wholeString[MAX_STR_LENGTH] = "";          //The entire input string from the last 'return'

unsigned long AFE44xx_SPO2_Data_buf[6];
//long AFE44xx_SPO2_Data_buf[6];
unsigned char txString[MAX_STR_LENGTH] = "";
//char startCaptureFlag = 0;

enum CAP_MODE {FINITE, CONTINUOUS};
char captureMode = FINITE;
char captureInProgressFlag = 0;

char sendDataFlag = 0;
char readDataFlag = 0;

//unsigned long AFE44xxRegArr[49];
//unsigned char AFE44xxRegAddr[49];

unsigned char AFE44xxAddr;
unsigned long AFE44xxRegVal;
unsigned long totalCount;
unsigned long sampleCount;

char CALIBRATION_ENABLED = FALSE;

/* Variables to modify depending on the application */
const unsigned char ILED1_CURR_MAX_mA = 45;     // LED1 max current reqd. for application
const unsigned char ILED1_CURR_MIN_mA = 5;      // LED1 min current reqd. for application
const unsigned char ILED2_CURR_MAX_mA = 45;     // LED2 max current reqd. for application
const unsigned char ILED2_CURR_MIN_mA = 5;      // LED2 min current reqd. for application
const unsigned char LOW_THR_PERCENT = 10;       // Low Threshold Percent
const unsigned char HIGH_THR_PERCENT = 90;      // High Threshold percent
const unsigned char HYS_PERCENT = 3;            // Hysteresis percent
const unsigned char TARGET_THR_PERCENT = 33;    // Target Threshold percent
/* End of Variables to modify depending on the application */
calibRoutineParams calibParams;

//test only !!!!!
unsigned long AFE44xx_readback_Register_Val[52];


unsigned char OLED_GRAM[128][8];

float lmt70_result,lmt70_result_pre,lmt70_diff;
/*  
* ======== main ========
*/
VOID main (VOID)
{
  WDTCTL = WDTPW + WDTHOLD;                                   //Stop watchdog timer
  
  Init_Ports();                                               //Init ports (do first ports because clocks do change ports)
  SetVCore(3);
  Init_Clock();                                               //Init clocks
  
  AFE44xx_PowerOn_Init(); 
  AFE44xx_Read_All_Regs(AFE44xx_readback_Register_Val);
  USB_init();                 //Init USB
  Init_TimerA1();
  P1OUT^=0x20;   //p1.5 output low
  
  //Enable various USB event handling routines
  USB_setEnabledEvents(
                       kUSB_VbusOnEvent + kUSB_VbusOffEvent + kUSB_receiveCompletedEvent
                         + kUSB_dataReceivedEvent + kUSB_UsbSuspendEvent + kUSB_UsbResumeEvent +
                           kUSB_UsbResetEvent);
  
  //See if we're already attached physically to USB, and if so, connect to it
  //Normally applications don't invoke the event handlers, but this is an exception.
  if (USB_connectionInfo() & kUSB_vbusPresent){
    USB_handleVbusOnEvent();
  }
  
  __enable_interrupt();                           //Enable interrupts globally
  
  IIC_init();
  SSD1306_init();
  OLED_Fill(5,5,10,10,1) ;
  OLED_Refresh_Gram();
  
  AD12_GPIO_set();
  AD12_init();
  AD12_startConversion();
  
  Init_UART();
  UART_send(txString, 1);   //test UART only
  
  
#if 1
  while (1)
    {   
        //------------------1  采集血氧数据--------------
        if (readDataFlag)
        {
          readDataFlag = 0;
          sampleCount++;
          if (sampleCount == totalCount)
          {

            sampleCount = 0;
            totalCount = 1;
            Disable_AFE44xx_DRDY_Interrupt();
            //P5OUT &= ~BIT0;                                                 //Turn off LED P5.0 (Green)
          }
          AFE44xx_SPO2_Data_buf[0] = AFE44xx_Reg_Read(42);  //read RED Data
          AFE44xx_SPO2_Data_buf[1] = AFE44xx_Reg_Read(43);  //read Ambient data
          AFE44xx_SPO2_Data_buf[2] = AFE44xx_Reg_Read(44);  //read IR Data
          AFE44xx_SPO2_Data_buf[3] = AFE44xx_Reg_Read(45);  //read Ambient Data
          AFE44xx_SPO2_Data_buf[4] = AFE44xx_Reg_Read(46);  //read RED - Ambient Data
          AFE44xx_SPO2_Data_buf[5] = AFE44xx_Reg_Read(47);  //read IR - Ambient Data
          sendDataFlag = 1;        
          
        }
        
//        //-----------------2  每采样500次血氧，采集一次体温--------------------
//        //-----------------LMT70-------------------------
//        if (0==(sampleCount%500))
//        {
//          ADC12_trigConversion();
//          lmt70_result=AD12_getResult();
//          lmt70_diff=fabs(lmt70_result-lmt70_result_pre);
////          if(lmt70_result>lmt70_result_pre)
////            lmt70_diff=lmt70_result-lmt70_result_pre;
////          else
////            lmt70_diff=lmt70_result_pre-lmt70_result;
//          
//          lmt70_result_pre=lmt70_result;
//          //-----！！！！！把AFE44xx_SPO2_Data_buf[5]替换成体温数据---hml for test only---
//          AFE44xx_SPO2_Data_buf[5]=(unsigned long)(lmt70_diff*1000000);
//          //AFE44xx_SPO2_Data_buf[5]=(unsigned long)(lmt70_result*1000000);
//        }
        //------------------3  如果需要发送数据到Labview--------------------
        if (sendDataFlag)
        {
          sendDataFlag = 0;
          txString[0] = (unsigned char) START_READ_ADC_REG_CMD;
          txString[1] = (unsigned char) SOT;
          txString[2] = (unsigned char)(AFE44xx_SPO2_Data_buf[0] & 0x000000FF);
          txString[3] = (unsigned char)((AFE44xx_SPO2_Data_buf[0] & 0x0000FF00) >> 8);
          txString[4] = (unsigned char)((AFE44xx_SPO2_Data_buf[0] & 0x00FF0000) >> 16);
          txString[5] = (unsigned char)(AFE44xx_SPO2_Data_buf[1] & 0x000000FF);
          txString[6] = (unsigned char)((AFE44xx_SPO2_Data_buf[1] & 0x0000FF00) >> 8);
          txString[7] = (unsigned char)((AFE44xx_SPO2_Data_buf[1] & 0x00FF0000) >> 16);
          txString[8] = (unsigned char)(AFE44xx_SPO2_Data_buf[2] & 0x000000FF);
          txString[9] = (unsigned char)((AFE44xx_SPO2_Data_buf[2] & 0x0000FF00) >> 8);
          txString[10] = (unsigned char)((AFE44xx_SPO2_Data_buf[2] & 0x00FF0000) >> 16);
          txString[11] = (unsigned char)(AFE44xx_SPO2_Data_buf[3] & 0x000000FF);
          txString[12] = (unsigned char)((AFE44xx_SPO2_Data_buf[3] & 0x0000FF00) >> 8);
          txString[13] = (unsigned char)((AFE44xx_SPO2_Data_buf[3] & 0x00FF0000) >> 16);
          txString[14] = (unsigned char)(AFE44xx_SPO2_Data_buf[4] & 0x000000FF);
          txString[15] = (unsigned char)((AFE44xx_SPO2_Data_buf[4] & 0x0000FF00) >> 8);
          txString[16] = (unsigned char)((AFE44xx_SPO2_Data_buf[4] & 0x00FF0000) >> 16);
          txString[17] = (unsigned char)(AFE44xx_SPO2_Data_buf[5] & 0x000000FF);
          txString[18] = (unsigned char)((AFE44xx_SPO2_Data_buf[5] & 0x0000FF00) >> 8);
          txString[19] = (unsigned char)((AFE44xx_SPO2_Data_buf[5] & 0x00FF0000) >> 16);
                    
          txString[20] = (unsigned char) EOT;
          txString[21] = (unsigned char) CR;
          cdcSendDataInBackground((BYTE*)txString,22,CDC0_INTFNUM,0);           // Send the response over USB
        }
        BYTE i;
        //Check the USB state and directly main loop accordingly
        switch (USB_connectionState())
        {
            case ST_USB_DISCONNECTED:
                //__bis_SR_register(LPM3_bits + GIE);                                 //Enter LPM3 w/ interrupts enabled
                _NOP();                                                             //For Debugger
                break;

            case ST_USB_CONNECTED_NO_ENUM:
                break;

            case ST_ENUM_ACTIVE:
                //__bis_SR_register(LPM0_bits + GIE);                                 //Enter LPM0 (can't do LPM3 when active)
                _NOP();                                                             //For Debugger
                                                                                    //Exit LPM on USB receive and perform a receive
                                                                                    //operation
                if (bCDCDataReceived_event){                                        //Some data is in the buffer; begin receiving a
                                                                                    //command
                    P5OUT |= BIT1;                                                        //Turn on LED P5.1 (Blue)
                    
                    char pieceOfString[MAX_STR_LENGTH] = "";                        //Holds the new addition to the string
                    //char outString[MAX_STR_LENGTH] = "";                            //Holds the outgoing string

                                                                                    //Add bytes in USB buffer to theCommand
                    cdcReceiveDataInBuffer((BYTE*)pieceOfString,        
                        MAX_STR_LENGTH,
                        CDC0_INTFNUM);                                                         //Get the next piece of the string
                    strcat(wholeString,pieceOfString);
                    //cdcSendDataInBackground((BYTE*)pieceOfString,
                    //    strlen(pieceOfString),CDC0_INTFNUM,0);                      //Echoes back the characters received (needed
                                                                                    //for Hyperterm)

                    if (retInString(wholeString)){                                  //Has the user pressed return yet?
                        if (wholeString[0] == WRITE_REG_CMD) // AFE44xx Write Operation
                        {
                          AFE44xxAddr = (ascii2uint8 (wholeString[1]) << 4) | ascii2uint8 (wholeString[2]);
                          unsigned long AFE44xxRegData[3];
                          AFE44xxRegData[0] = (ascii2uint8 (wholeString[3]) << 4) | ascii2uint8 (wholeString[4]);
                          AFE44xxRegData[1] = (ascii2uint8 (wholeString[5]) << 4) | ascii2uint8 (wholeString[6]);
                          AFE44xxRegData[2] = (ascii2uint8 (wholeString[7]) << 4) | ascii2uint8 (wholeString[8]);
                          AFE44xxRegVal = (AFE44xxRegData[0]<<16)| (AFE44xxRegData[1]<<8) | (AFE44xxRegData[2]);
                                                    
                          AFE44xx_Reg_Write(AFE44xxAddr, AFE44xxRegVal);
                        } 
                        else if (wholeString[0] == READ_REG_CMD) // AFE44xx Read Operation
                        {
                          AFE44xxAddr = (ascii2uint8 (wholeString[1]) << 4) | ascii2uint8 (wholeString[2]);
                          AFE44xxRegVal = AFE44xx_Reg_Read(AFE44xxAddr);
                          txString[0] = (unsigned char) READ_REG_CMD;
                          txString[1] = (unsigned char) SOT;
                          txString[2] = (unsigned char)(AFE44xxRegVal & 0x000000FF);
                          txString[3] = (unsigned char)((AFE44xxRegVal & 0x0000FF00) >> 8);
                          txString[4] = (unsigned char)((AFE44xxRegVal & 0x00FF0000) >> 16);
                          txString[5] = (unsigned char) EOT;
                          txString[6] = (unsigned char) CR;
                          cdcSendDataInBackground((BYTE*)txString,7,CDC0_INTFNUM,0);          // Send the response over USB                         
                        }
                        else if (wholeString[0] == START_READ_ADC_REG_CMD) // Start AFE44x0 ADC Reg Read Command
                        {
                          totalCount = 1;
                          sampleCount = 0;
                          for (i = 0; i < ((ascii2uint8 (wholeString[2]) << 4) | ascii2uint8 (wholeString[3])); i++)
                          {
                            totalCount *= 2;
                          }
                          Enable_AFE44xx_DRDY_Interrupt();			// Enable DRDY interrupt
                          
                          readDataFlag = 0;
                          sendDataFlag = 0;
                          AFE44xx_SPO2_Data_buf[0] = 0;
                          AFE44xx_SPO2_Data_buf[1] = 0;
                          AFE44xx_SPO2_Data_buf[2] = 0;
                          AFE44xx_SPO2_Data_buf[3] = 0;
                          AFE44xx_SPO2_Data_buf[4] = 0;
                          AFE44xx_SPO2_Data_buf[5] = 0;
                        }
                        else if (wholeString[0] == STOP_READ_ADC_REG_CMD) // Stop AFE44x0 ADC Reg Read Command
                        {
                          sampleCount = 0;
                          totalCount = 1;
                          Disable_AFE44xx_DRDY_Interrupt();
                          readDataFlag = 0;
                          sendDataFlag = 0;
                          //P5OUT &= ~BIT0;                                                        //Turn off LED P5.0 (Green)
                        }
                        else if (wholeString[0] == FW_UPGRADE_CMD) // Firmware Upgrade Command
                        {

                          USB_disable();   				// Disable CDC connection						
		          TA1CTL &= ~MC_1;                              // Turn off Timer                     

                          Disable_AFE44xx_DRDY_Interrupt();             // Disable interrupt
                          readDataFlag = 0;                             // Disable Read Data flag
                          sendDataFlag = 0;                             // Disable send Data flag
                          __delay_cycles(200);				// Delay
                          ((void (*)())0x1000)();			// Go to BSL
                        }

                        else if (wholeString[0] == DEV_ID_CMD) // Dev ID Command
                        {

                          txString[0] = (unsigned char)DEV_ID_CMD;
                          txString[1] = (unsigned char)SOT;
                          txString[2] = 0x34;
                          txString[3] = 0x34;
#ifdef __AFE4400__
                          txString[4] = 0x30;
#endif
#ifdef __AFE4490__
                          txString[4] = 0x39;
#endif
                          txString[5] = 0x30;
                          txString[6] = (unsigned char)EOT;
                          txString[7] = (unsigned char)CR;
                          cdcSendDataInBackground((BYTE*)txString,8,CDC0_INTFNUM,0);          // Send the response over USB 
                        }
                        else if (wholeString[0] == FW_VERSION_CMD) // FW Version Command
                        {
                          txString[0] = (unsigned char)FW_VERSION_CMD;
                          txString[1] = (unsigned char)SOT;
                          txString[2] = (unsigned char)AFE44x0_Major_Number;
                          txString[3] = (unsigned char)AFE44x0_Minor_Number;
                          txString[4] = (unsigned char)EOT;
                          txString[5] = (unsigned char)CR;
                          cdcSendDataInBackground((BYTE*)txString,6,CDC0_INTFNUM,0);          // Send the response over USB 
                        }
                        for (i = 0; i < MAX_STR_LENGTH; i++){                       //Clear the string in preparation for the next
                                                                                    //one
                            wholeString[i] = 0x00;
                        }
                        P5OUT &= ~BIT1;                                                        //Turn off LED P5.1 (Blue)
                    }
                    bCDCDataReceived_event = FALSE;
                }
                break;

            case ST_ENUM_SUSPENDED:
                P5OUT &= ~BIT0;                                                     //When suspended, turn off LED
                //__bis_SR_register(LPM3_bits + GIE);                                 //Enter LPM3 w/ interrupts
                _NOP();
                break;

            case ST_ENUM_IN_PROGRESS:
                break;

            case ST_NOENUM_SUSPENDED:
                P5OUT &= ~BIT0;
                //__bis_SR_register(LPM3_bits + GIE);
                _NOP();
                break;

            case ST_ERROR:
                _NOP();
                break;

            default:;
        }
    }  //while(1)
#endif 
  
} //main()



// Port 2 interrupt service routine
#pragma vector=PORT2_VECTOR  //DRDY interrupt
__interrupt void Port_2(void)
{
  switch (P2IV)
  {
  case P2IV_P2IFG2:
    P2IFG &= ~BIT2;                                   // Clear P2.2 IFG i.e Data RDY interrupt status
    //P5OUT |= BIT0;                                    //Turn on LED P5.0 (Green)
    readDataFlag = 1;                                 // Set Flag to read AFE44x0 ADC REG data
    break;
    
  case P2IV_NONE:
    break;
  }
}



