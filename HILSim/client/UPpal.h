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

#ifndef UPpalH
#define UPpalH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
#include <Graphics.hpp>
#include <ComCtrls.hpp>
//#include "Placemnt.hpp"
#include <Buttons.hpp>
//#include "ABSMain.hpp"
//#include <DB.hpp>
//#include <DBCtrls.hpp>
#include <Dialogs.hpp>
#include <Mask.hpp>
#include "AdPort.hpp"
#include "_GClass.hpp"
#include "AbLED.hpp"
#include "AbOpHour.hpp"
#include "AbMTrend.hpp"
#include <math.h>
#include <exception.h>
#include "AbVCInd.hpp"
#include "AbCBitBt.hpp"
#include "ToolEdit.hpp"
#include "WSocket.hpp"
#include "AbTank.hpp"
#include "AbVBar.hpp"
#include "AbHBar.hpp"
#include "CurrEdit.hpp"
#include "OoMisc.hpp"
#include "RxCombos.hpp"
#include "RXSlider.hpp"
#include <string>
#include <limits.h>

#include "../mavlink_hil/include/slugs/slugs.h"

#define  DISLIMIT      30.0

#define HIL_GPS_START        0
#define HIL_DYN_START        27
#define HIL_RAW_START        37
#define HIL_RAW_AIR_START    55
#define HIL_ATT_START        61
#define HIL_XYZ_START        89

enum{
  HIL_GPS,
  HIL_GPS_DATE_TIME,
  HIL_DYN,
  HIL_RAW,
  HIL_RAW_AIR,
  HIL_ATT,
  HIL_XYZ
};

#define LOGSIZE        254

//---------------------------------------------------------------------------
class TFPpal : public TForm
{
__published:	// IDE-managed Components
    TPanel *Panel1;
    TLabel *Label1;
    TMemo *mm_diagnose;
    TSpeedButton *bt_clear;
    TApdComPort *cp_serial;
    TWSocket *skt_send;
    TApdComPort *cp_hil;
    TWSocket *skt_rcv;
        TGroupBox *GroupBox12;
        TLabel *DataAvailableLabel;
        TLabel *InfoLabel;
        TLabel *Label66;
        TLabel *Label67;
        TEdit *PortEdit;
        TEdit *ServerEdit;
        TCheckBox *AnyServerCheckBox;
        TGroupBox *GroupBox11;
        TLabel *et_connSend;
        TLabel *Label64;
        TLabel *Label65;
        TLabel *et_sent;
        TEdit *ed_portSend;
        TEdit *ed_hostSend;
        TAbLED *ld_filter;
        TSpeedButton *bt_filter;
        TGroupBox *GroupBox1;
        TLabel *Label4;
        TLabel *Label5;
        TEdit *ed_slugsSysId;
        TEdit *ed_slugsCompId;
        TGroupBox *GroupBox2;
        TLabel *Label2;
        TLabel *Label3;
        TEdit *ed_gsSysId;
        TEdit *ed_gsCompId;
        void __fastcall bt_serialClick(TObject *Sender);
        void __fastcall cp_serialTriggerAvail(TObject *CP, WORD Count);
        void __fastcall bt_filterClick(TObject *Sender);
        void __fastcall skt_rcvSessionClosed(TObject *Sender, WORD ErrCode);
        void __fastcall skt_rcvSessionConnected(TObject *Sender, WORD ErrCode);
        void __fastcall skt_rcvDataAvailable(TObject *Sender, WORD ErrCode);
        void __fastcall AnyServerCheckBoxClick(TObject *Sender);
        void __fastcall bt_clearClick(TObject *Sender);
        void __fastcall cp_hilTriggerAvail(TObject *CP, WORD Count);
        void __fastcall FormCreate(TObject *Sender);

private:	// User declarations
public:		// User declarations
        __fastcall TFPpal(TComponent* Owner);
        void createBlankKML(char KMLType);
        void updateKML(void);
        String getHexColor(unsigned char whichColor);
        String getPlaneCoordinates(void);
        unsigned char assembleMsg(unsigned char* rawData , unsigned char type, unsigned char* protMsg );
        float getFloatFromDatagram (unsigned char* datagram, unsigned char * i);
        uint16_t getUint16FromDatagram (unsigned char* datagram, unsigned char * i);
        int16_t getInt16FromDatagram (unsigned char* datagram, unsigned char * i);
        uint32_t getUint32FromDatagram (unsigned char* datagram, unsigned char * i);

        String str_modes[10];

        float csFail;

        bool logIsOpen;
        bool waitingDelay;
        unsigned char pidRequestQueue;
        unsigned char allGains;

        bool myisnan(float var);


        // HIL
        Winsock::TInAddr      FServerAddr;
        void processUdpMsg(unsigned char * buffer);
        void TxPWMMsg (void);

        typedef union _tFloatToChar {
                unsigned char   chData[4];
		float   	flData;
	} tFloatToChar;

	typedef union _tUint16ToChar {
		unsigned char   chData[2];
		uint16_t   	uiData;
	} tUint16ToChar;

        typedef union _tInt16ToChar {
                unsigned char   chData[2];
                int16_t   	inData;
        } tInt16ToChar;

        typedef union _tUint32ToChar {
		unsigned char   chData[4];
		uint32_t   	uiData;
	} tUint32ToChar;

        mavlink_servo_output_raw_t	mlPwmCommands;
        mavlink_attitude_t              mlAttitudeData;


};
//---------------------------------------------------------------------------
extern PACKAGE TFPpal *FPpal;
//---------------------------------------------------------------------------
#endif
