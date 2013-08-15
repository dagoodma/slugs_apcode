/*
The MIT License

Copyright (c) 2009 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "UPpal.h"


//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "Placemnt"
#pragma link "ABSMain"
#pragma link "AdPort"
#pragma link "OoMisc"
#pragma link "_GClass"
#pragma link "AbLED"
#pragma link "AbOpHour"
#pragma link "AbMTrend"
#pragma link "AbVCInd"
#pragma link "AbCBitBt"
#pragma link "ToolEdit"
#pragma link "WSocket"
#pragma link "OoMisc"
#pragma link "AbTank"
#pragma link "AbVBar"
#pragma link "AbHBar"
#pragma link "CurrEdit"
#pragma link "OoMisc"
#pragma link "RxCombos"
#pragma link "RXSlider"
#pragma resource "*.dfm"


TFPpal *FPpal;
//---------------------------------------------------------------------------
__fastcall TFPpal::TFPpal(TComponent* Owner)
        : TForm(Owner)
{
}
//---------------------------------------------------------------------------


 void __fastcall TFPpal::skt_rcvDataAvailable(TObject *Sender, WORD ErrCode)
{
    char        Buffer[113];
    int         Len;
    Winsock::TSockAddrIn Src;
    int         SrcLen;

    memset(Buffer, 0, 113);

    SrcLen = sizeof(Src);

    Len    = skt_rcv->ReceiveFrom(Buffer, sizeof(Buffer), Src, SrcLen);

    if (Len >= 0) {
        if ((FServerAddr.s_addr == INADDR_ANY) ||
            (FServerAddr.s_addr == Src.sin_addr.s_addr)) {
            if (bt_filter->Tag){
               processUdpMsg(Buffer);
            }
            DataAvailableLabel->Caption =
                IntToStr(atoi(DataAvailableLabel->Caption.c_str()) + 1) +
                " Packets Received";
        }
       TxPWMMsg ();
    }
}
//---------------------------------------------------------------------------

void __fastcall TFPpal::cp_serialTriggerAvail(TObject *CP, WORD Count)
{
/*

  */
}
//---------------------------------------------------------------------------

void __fastcall TFPpal::bt_serialClick(TObject *Sender)
{    /*

 */
}
//---------------------------------------------------------------------------

bool TFPpal::myisnan(float var) {
   volatile float temp1 = var;
   volatile float temp2 = var;
   return temp1 != temp2;
}

//---------------------------------------------------------------------------
void __fastcall TFPpal::bt_filterClick(TObject *Sender)
{
    bt_filter->Tag ^=1;

    if (bt_filter->Tag) {
        // Socket Receive
        FServerAddr                = WSocketResolveHost(ServerEdit->Text);
        if (FServerAddr.s_addr == htonl(INADDR_LOOPBACK)) {
           // Replace loopback address by real localhost IP addr
           FServerAddr            = WSocketResolveHost(LocalHostName());
        }
        skt_rcv->Proto             = "udp";
        skt_rcv->Addr              = "0.0.0.0";
        skt_rcv->Port              = PortEdit->Text;
        skt_rcv->Listen();
        PortEdit->Enabled          = FALSE;
        ServerEdit->Enabled        = FALSE;
        AnyServerCheckBox->Enabled = FALSE;


        // Socket Send
        skt_send->Proto = "udp";
        skt_send->Addr  = ed_hostSend->Text;
        skt_send->Port  = ed_portSend->Text;
        skt_send->Connect();
        ed_portSend->Enabled = FALSE;
        ed_hostSend->Enabled = FALSE;
        et_connSend->Caption   = "Connected";

        // Serial Port
        cp_hil->Open = True;
        ld_filter->StatusInt= 1;
        bt_filter->Caption = "HIL Off";
    } else {

       // Socket Receive
       PortEdit->Enabled          = TRUE;
       ServerEdit->Enabled        = TRUE;
       AnyServerCheckBox->Enabled = TRUE;
       skt_rcv->Close();

       // Socket send
       skt_send->Close();
       ed_portSend->Enabled         = TRUE;
       ed_hostSend->Enabled         = TRUE;
       et_connSend->Caption         = "Disconnected";
       et_sent->Caption             = "";

       // Serial Port
       cp_hil->Open = False;
       ld_filter->StatusInt= 0;
       bt_filter->Caption = "HIL On";

    }

}
//---------------------------------------------------------------------------



void __fastcall TFPpal::skt_rcvSessionClosed(TObject *Sender, WORD ErrCode)
{
     PortEdit->Enabled           = TRUE;
    ServerEdit->Enabled         = TRUE;
    AnyServerCheckBox->Enabled  = TRUE;
    InfoLabel->Caption          = "Disconnected";
    DataAvailableLabel->Caption = "";    
}
//---------------------------------------------------------------------------

void __fastcall TFPpal::skt_rcvSessionConnected(TObject *Sender,
      WORD ErrCode)
{
     InfoLabel->Caption   = "Connected";
}
//---------------------------------------------------------------------------


void  TFPpal::processUdpMsg (unsigned char * buffer)
{
  unsigned char hilMsg [100];
  unsigned char len;
  static unsigned char  vr_roundRobin = 1;
  static unsigned char  vr_posOrAtt = 0;

  // reset message variable
  memset(hilMsg, 0, 100);

  switch (vr_roundRobin){
         case 1:
           // GPS Sentence
           // ============
           // Assemble the message for transmission
           len = assembleMsg(&buffer[HIL_GPS_START],HIL_GPS,hilMsg);
         break;

         case 2:
           // Assemble the message for transmission
           len = assembleMsg(&buffer[HIL_GPS_START],HIL_GPS_DATE_TIME,hilMsg);
         break;

         case 3:
           // Air Data Sentence
           // =================
           // Assemble the message for transmission
           len = assembleMsg(&buffer[HIL_DYN_START],HIL_DYN,hilMsg);

         break;

         case 4:
           // Raw Data Sentence
           // =================
           // Assemble the message for transmission
           len = assembleMsg(&buffer[HIL_RAW_START],HIL_RAW,hilMsg);

         break;

         case 5:
           // Assemble the message for transmission
           len = assembleMsg(&buffer[HIL_RAW_AIR_START],HIL_RAW_AIR,hilMsg);

         break;

  } // switch round Robin

  // Send the data
  cp_hil->PutBlock(hilMsg,len);
  vr_roundRobin = (vr_roundRobin > 5)? 1 : vr_roundRobin + 1;



  // reset message variable
  memset(hilMsg, 0, 100);



  if (vr_posOrAtt) {
     // Att Data Sentence
     // =================

     // Assemble the message for transmission
     len = assembleMsg(&buffer[HIL_ATT_START], HIL_ATT,hilMsg);
  } else {

    // XYZ Data Sentence
    // =================
    // Assemble the message for transmission
    len = assembleMsg(&buffer[HIL_XYZ_START], HIL_XYZ,hilMsg);
  }
  // Send the data
  cp_hil->PutBlock(hilMsg, len);
  vr_posOrAtt ^=1;

}
//---------------------------------------------------------------------------

unsigned char TFPpal::assembleMsg(unsigned char* rawData , unsigned char type, unsigned char* protMsg ){

unsigned char i = 0;
mavlink_message_t msg;

mavlink_gps_raw_t mlGPSData;
mavlink_gps_date_time_t mlGPSDateTime;
mavlink_raw_imu_t mlRawData;
mavlink_raw_pressure_t mlRawPressure;
mavlink_scaled_pressure_t mlAirData;
mavlink_local_position_t mlLocalPositionData;

switch (type){

    case HIL_GPS:
	  i+=6;
	  mlGPSData.lat = getFloatFromDatagram (rawData, &i);
	  mlGPSData.lon = getFloatFromDatagram (rawData, &i);
	  mlGPSData.alt = getFloatFromDatagram (rawData, &i);
	  mlGPSData.hdg = static_cast<float>(getUint16FromDatagram(rawData, &i));
	  mlGPSData.v   = static_cast<float>(getUint16FromDatagram(rawData, &i));
	  mlGPSData.eph = static_cast<float>(getUint16FromDatagram(rawData, &i));

          mavlink_msg_gps_raw_encode( ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlGPSData);
    break;

    case HIL_GPS_DATE_TIME:
       	  mlGPSDateTime.year  = rawData[i++];
	  mlGPSDateTime.month = rawData[i++];
	  mlGPSDateTime.day   = rawData[i++];
	  mlGPSDateTime.hour  = rawData[i++];
	  mlGPSDateTime.min   = rawData[i++];
	  mlGPSDateTime.sec   = rawData[i++];
          i+=19;
          mlGPSDateTime.visSat= rawData[i++];


          mavlink_msg_gps_date_time_encode( ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlGPSDateTime);
    break;

   case HIL_DYN:
         mlAirData.press_diff = getFloatFromDatagram (rawData, &i);
         mlAirData.press_abs = getFloatFromDatagram (rawData, &i);
         mlAirData.temperature = getInt16FromDatagram(rawData, &i);

         mavlink_msg_scaled_pressure_encode( ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlAirData);

    break;

    case HIL_RAW:
         mlRawData.xgyro = getInt16FromDatagram(rawData, &i);
         mlRawData.ygyro = getInt16FromDatagram(rawData, &i);
         mlRawData.zgyro = getInt16FromDatagram(rawData, &i);
         mlRawData.xacc  = getInt16FromDatagram(rawData, &i);
         mlRawData.yacc  = getInt16FromDatagram(rawData, &i);
         mlRawData.zacc  = getInt16FromDatagram(rawData, &i);
         mlRawData.xmag  = getInt16FromDatagram(rawData, &i);
         mlRawData.ymag  = getInt16FromDatagram(rawData, &i);
         mlRawData.zmag  = getInt16FromDatagram(rawData, &i);

         mavlink_msg_raw_imu_encode(ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlRawData);
    break;

    case HIL_RAW_AIR:
         mlRawPressure.press_diff1 = getInt16FromDatagram(rawData, &i);
         mlRawPressure.press_abs   = getInt16FromDatagram(rawData, &i);
         mlRawPressure.temperature = getInt16FromDatagram(rawData, &i);

         mavlink_msg_raw_pressure_encode(ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlRawPressure);
    break;

    case HIL_ATT:
         mlAttitudeData.roll  = getFloatFromDatagram(rawData, &i);
         mlAttitudeData.pitch = getFloatFromDatagram(rawData, &i);
         mlAttitudeData.yaw   = getFloatFromDatagram(rawData, &i);

         mlAttitudeData.rollspeed    = getFloatFromDatagram(rawData, &i);
         mlAttitudeData.pitchspeed   = getFloatFromDatagram(rawData, &i);
         mlAttitudeData.yawspeed     = getFloatFromDatagram(rawData, &i);

         mlAttitudeData.usec = getUint32FromDatagram(rawData, &i);

         mavlink_msg_attitude_encode(ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlAttitudeData);
    break;

    case HIL_XYZ:
         mlLocalPositionData.x  = getFloatFromDatagram(rawData, &i);
         mlLocalPositionData.y  = getFloatFromDatagram(rawData, &i);
         mlLocalPositionData.z  = getFloatFromDatagram(rawData, &i);

         mlLocalPositionData.vx  = getFloatFromDatagram(rawData, &i);
         mlLocalPositionData.vy  = getFloatFromDatagram(rawData, &i);
         mlLocalPositionData.vz  = getFloatFromDatagram(rawData, &i);

         mavlink_msg_local_position_encode(ed_slugsSysId->Text.ToInt(),ed_slugsCompId->Text.ToInt(),&msg,&mlLocalPositionData);
    break;

}



 return mavlink_msg_to_send_buffer(protMsg, &msg);


}

//---------------------------------------------------------------------------
void TFPpal::TxPWMMsg (void){
   char send_buffer[20];
   tUint16ToChar temp;
   tUint32ToChar temp32;

   tUint32ToChar timeStamp;
   timeStamp.uiData = static_cast<uint32_t>(mlAttitudeData.usec);
   // Freeze the PWM data to avoid changing data in the
   // middle of a send UDP
   mavlink_servo_output_raw_t pwmSampleLocal = mlPwmCommands;

   temp.uiData = pwmSampleLocal.servo1_raw;
   send_buffer[0] 	= temp.chData[0];
   send_buffer[1] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo2_raw;
   send_buffer[2] 	= temp.chData[0];
   send_buffer[3] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo3_raw;
   send_buffer[4] 	= temp.chData[0];
   send_buffer[5] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo4_raw;
   send_buffer[6] 	= temp.chData[0];
   send_buffer[7] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo5_raw;
   send_buffer[8] 	= temp.chData[0];
   send_buffer[9] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo6_raw;
   send_buffer[10] 	= temp.chData[0];
   send_buffer[11] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo7_raw;
   send_buffer[12] 	= temp.chData[0];
   send_buffer[13] 	= temp.chData[1];

   temp.uiData = pwmSampleLocal.servo8_raw;
   send_buffer[14] 	= temp.chData[0];
   send_buffer[15] 	= temp.chData[1];

   send_buffer[16] 	= timeStamp.chData[0];
   send_buffer[17] 	= timeStamp.chData[1];
   send_buffer[18] 	= timeStamp.chData[2];
   send_buffer[19] 	= timeStamp.chData[3];


  skt_send->Send(send_buffer,20);
  et_sent->Caption = IntToStr(atoi(et_sent->Caption.c_str()) + 1) + " Packets Sent";

}

//---------------------------------------------------------------------------


void __fastcall TFPpal::AnyServerCheckBoxClick(TObject *Sender)
{
    if (AnyServerCheckBox->Checked)
        ServerEdit->Text = "0.0.0.0";
}
//---------------------------------------------------------------------------

float TFPpal::getFloatFromDatagram (unsigned char* datagram, unsigned char * i){
  tFloatToChar tmpF2C;

  memset(&tmpF2C,0, sizeof(tFloatToChar));

  tmpF2C.chData[0] = datagram[(*i)++];
  tmpF2C.chData[1] = datagram[(*i)++];
  tmpF2C.chData[2] = datagram[(*i)++];
  tmpF2C.chData[3] = datagram[(*i)++];

  return tmpF2C.flData;
}

uint16_t TFPpal::getUint16FromDatagram (unsigned char* datagram, unsigned char * i){
  tUint16ToChar tmpU2C;

  memset(&tmpU2C,0, sizeof(tUint16ToChar));
  
  tmpU2C.chData[0] = datagram[(*i)++];
  tmpU2C.chData[1] = datagram[(*i)++];

  return tmpU2C.uiData;

}

uint32_t TFPpal::getUint32FromDatagram (unsigned char* datagram, unsigned char * i){
  tUint32ToChar tmpU2C;

  memset(&tmpU2C,0, sizeof(tUint32ToChar));

  tmpU2C.chData[0] = datagram[(*i)++];
  tmpU2C.chData[1] = datagram[(*i)++];
  tmpU2C.chData[2] = datagram[(*i)++];
  tmpU2C.chData[3] = datagram[(*i)++];

  return tmpU2C.uiData;

}


int16_t TFPpal::getInt16FromDatagram (unsigned char* datagram, unsigned char * i){
  tInt16ToChar tmpI2C;

  memset(&tmpI2C,0, sizeof(tInt16ToChar));

  tmpI2C.chData[0] = datagram[(*i)++];
  tmpI2C.chData[1] = datagram[(*i)++];

  return tmpI2C.inData;

}

/// ================================================================





void __fastcall TFPpal::bt_clearClick(TObject *Sender)
{
      mm_diagnose->Clear();
}
//---------------------------------------------------------------------------

void __fastcall TFPpal::cp_hilTriggerAvail(TObject *CP, WORD Count)
{

  unsigned char fromSerial[LOGSIZE];

  unsigned char bytesReceived;
  int bytesRemain = Count;

  static unsigned char prevException = 0;
  uint8_t i;
  mavlink_message_t msg;
  mavlink_status_t status;

  try {
    while (bytesRemain > 0) {
       if (bytesRemain <= LOGSIZE) {
         cp_hil->GetBlock(fromSerial, bytesRemain);
         bytesReceived = bytesRemain;
         bytesRemain = 0;
       } else{
         cp_hil->GetBlock(fromSerial, LOGSIZE);
         bytesRemain -= LOGSIZE;
         bytesReceived = LOGSIZE;
       }

       // decode the chunk you just got
       for(i = 0; i < bytesReceived; i++ ){
           //mm_diagnose->Lines->Add(IntToStr(fromSerial[i]));
           // Try to get a new message
           if(mavlink_parse_char(0, fromSerial[i], &msg, &status)) {
           //mm_diagnose->Lines->Add("=================");
                // Handle message
                switch(msg.msgid){
                   case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
                        mavlink_msg_servo_output_raw_decode(&msg, &mlPwmCommands);
                   break;
                } // switch
           } // if
       }// for
       // no previous exception
       prevException =0;
    } // While
  } // try
   catch (...) {
      mm_diagnose->Lines->Add("Exception Caught");
      prevException = 1;
  } // catcj

}
//---------------------------------------------------------------------------

void __fastcall TFPpal::FormCreate(TObject *Sender)
{
  memset(&mlPwmCommands ,0, sizeof(mavlink_servo_output_raw_t));
  memset(&mlAttitudeData ,0, sizeof(mavlink_attitude_t));

}
//---------------------------------------------------------------------------

