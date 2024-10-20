// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "ublox.h"
#include "../config.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include "../pgmspace.h"
#include "../NutsBolts.h"
#include <math.h>


SFE_UBLOX_GPS configGPS; // used for f9p module configuration only


// used to send .ubx log files via 'sendgps.py' to Arduino (also set GPS to Serial in config for this)
//#define GPS_DUMP   1


UBLOX::UBLOX()
{
  debug = false;
  verbose = false;
  useTCP = false;
  solutionTimeout = 0;
#ifdef GPS_DUMP
  verbose = true;
#endif
}

void UBLOX::begin()
{
  wxLogMessage("using gps driver: UBLOX");
  this->state    = GOT_NONE;
  this->msgclass = -1;
  this->msgid    = -1;
  this->msglen   = -1;
  this->chka     = -1;
  this->chkb     = -1;
  this->count    = 0;
  this->dgpsAge  = 0;
  this->solutionAvail = false;
  this->numSV    = 0;
  this->numSVdgps    = 0;
  this->accuracy  =0;
  this->chksumErrorCounter = 0;
  this->dgpsChecksumErrorCounter = 0;
  this->dgpsPacketCounter = 0;
}

void UBLOX::begin(Client &client, char *host, uint16_t port)
{
  wxLogMessage("UBLOX::begin tcp");
  useTCP = true;
  _client = &client;
  if(!client.connect(host,port))
    {
      wxLogMessage("Cannot connect to %c ! %i", host, port);
    }
  // start streaming-in
  begin();
}


/* starts the serial communication */
void UBLOX::begin(HardwareSerial& bus,uint32_t baud)
{
  wxLogMessage("UBLOX::begin serial");
  _bus = &bus;
  _baud = baud;
  // begin the serial port for uBlox
  _bus->begin(_baud);
  // start streaming-in
  begin();
  if (GPS_CONFIG)
    {
      configure();
    }
}


bool UBLOX::configure()
{
  wxLogMessage("trying to connect to ublox f9p...");
  wxLogMessage("NOTE: if GPS is not responding either set 'GPS_CONFIG=false' in config.h or perform GPS wire fix (see Wiki)");
  //configGPS.enableDebugging(CONSOLE, false);

  while(true)
    {
      wxLogMessage("trying baud %i", _baud);
      if (configGPS.begin(*_bus)) break;
      wxLogMessage(F("ERROR: GPS receiver is not responding"));
      wxLogMessage("trying baud 38400");
      _bus->begin(38400);
      if (configGPS.begin(*_bus))
        {
          configGPS.setVal32(0x40520001, _baud, VAL_LAYER_RAM);  // CFG-UART1-BAUDRATE   (Ardumower)
          _bus->begin(_baud);
          break;
        }
      _bus->begin(_baud);
      wxLogMessage(F("ERROR: GPS receiver is not responding"));
    }

  wxLogMessage("GPS receiver found!");

  wxLogMessage("ublox f9p: sending GPS rover configuration...");

  int16_t timeout = 2000;
  //int16_t idx = 0;
  int16_t succeeded = 0;
  for (int16_t idx=0; idx < 3; idx++)
    {
      for (int16_t i=1; i < 3; i++)
        {
          bool setValueSuccess = true;
          wxLogMessage("idx=%i...", idx);
          if (idx == 0)
            {
              // ----- USB messages (Ardumower) -----------------
              setValueSuccess &= configGPS.newCfgValset8(0x20910009, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_USB    (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910090, 0); // CFG-MSGOUT-UBX_NAV_RELPOSNED_USB  (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910036, 0); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910045, 0); // CFG-MSGOUT-UBX_NAV_VELNED_USB     (off)
              setValueSuccess &= configGPS.addCfgValset8(0x2091026b, 0); // CFG-MSGOUT-UBX_RXM_RTCM_USB   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910348, 0); // CFG-MSGOUT-UBX_NAV_SIG_USB   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x2091005e, 0); // CFG-MSGOUT-UBX_NAV_TIMEUTC_USB   (off)

              // ----- uart1 messages (Ardumower) -----------------
              setValueSuccess &= configGPS.addCfgValset8(0x20910007, 0); // CFG-MSGOUT-UBX_NAV_PVT_UART1   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x2091008e, 0); // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1  (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910034, 0); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910043, 0); // CFG-MSGOUT-UBX_NAV_VELNED_UART1     (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910269, 0); // CFG-MSGOUT-UBX_RXM_RTCM_UART1   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910346, 0); // CFG-MSGOUT-UBX_NAV_SIG_UART1   (off)
              setValueSuccess &= configGPS.sendCfgValset8(0x2091005c, timeout); // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   (off)
            }
          else if (idx == 1)
            {
              setValueSuccess &= configGPS.newCfgValset8(0x209100a8, 0, VAL_LAYER_RAM); // CFG-MSGOUT-NMEA_ID_DTM_UART2  (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100df, 0); // CFG-MSGOUT-NMEA_ID_GBS_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100bc, 60); // CFG-MSGOUT-NMEA_ID_GGA_UART2  (every 60 solutions)
              setValueSuccess &= configGPS.addCfgValset8(0x209100cb, 0); // CFG-MSGOUT-NMEA_ID_GLL_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100b7, 0); // CFG-MSGOUT-NMEA_ID_GNS_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100d0, 0); // CFG-MSGOUT-NMEA_ID_GRS_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100c1, 0); // CFG-MSGOUT-NMEA_ID_GSA_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100d5, 0); // CFG-MSGOUT-NMEA_ID_GST_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100c6, 0); // CFG-MSGOUT-NMEA_ID_GSV_UART2   (off)
              //setValueSuccess &= configGPS.addCfgValset8(0x20910402, 0); // CFG-MSGOUT-NMEA_ID_RLM_UART2    (fails)
              setValueSuccess &= configGPS.addCfgValset8(0x209100ad, 0); // CFG-MSGOUT-NMEA_ID_RMC_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100e9, 0); // CFG-MSGOUT-NMEA_ID_VLW_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100b2, 0); // CFG-MSGOUT-NMEA_ID_VTG_UART2   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x209100da, 0);  // CFG-MSGOUT-NMEA_ID_ZDA_UART2  (off)
              // uart2 protocols (Xbee/NTRIP)
              setValueSuccess &= configGPS.addCfgValset8(0x10750001, 0); // CFG-UART2INPROT-UBX        (off)
              setValueSuccess &= configGPS.addCfgValset8(0x10750002, 0); // CFG-UART2INPROT-NMEA       (off)
              setValueSuccess &= configGPS.addCfgValset8(0x10750004, 1); // CFG-UART2INPROT-RTCM3X     (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10760001, 0); // CFG-UART2OUTPROT-UBX       (off)
              setValueSuccess &= configGPS.addCfgValset8(0x10760002, 1); // CFG-UART2OUTPROT-NMEA      (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10760004, 0); // CFG-UART2OUTPROT-RTCM3X    (off)
              // uart2 baudrate  (Xbee/NTRIP)
              setValueSuccess &= configGPS.addCfgValset32(0x40530001, 115200); // CFG-UART2-BAUDRATE

              // ----- uart1 protocols (Ardumower) ---------------
              setValueSuccess &= configGPS.addCfgValset8(0x10730001, 1); // CFG-UART1INPROT-UBX     (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10730002, 1); // CFG-UART1INPROT-NMEA    (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10730004, 1); // CFG-UART1INPROT-RTCM3X  (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10740001, 1); // CFG-UART1OUTPROT-UBX    (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10740002, 0); // CFG-UART1OUTPROT-NMEA   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x10740004, 0); // CFG-UART1OUTPROT-RTCM3X (off)

              // ----- USB protocols (Ardumower) -----------------
              setValueSuccess &= configGPS.addCfgValset8(0x10770001, 1); // CFG-USBINPROT-UBX     (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10770002, 1); // CFG-USBINPROT-NMEA    (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10770004, 1); // CFG-USBINPROT-RTCM3X  (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10780001, 1); // CFG-USBOUTPROT-UBX    (on)
              setValueSuccess &= configGPS.addCfgValset8(0x10780002, 0); // CFG-USBOUTPROT-NMEA   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x10780004, 0); // CFG-USBOUTPROT-RTCM3X (off)

              // ---- gps fix mode ---------------------------------------------
              // we contrain altitude here (when receiver is started in docking station it may report a wrong
              // altitude without correction data and SAPOS will not work with an unplausible reported altitute - the contrains are
              // ignored once receiver is working in RTK mode)
              setValueSuccess &= configGPS.addCfgValset8(0x20110011, 3); // CFG-NAVSPG-FIXMODE    (1=2d only, 2=3d only, 3=auto)
              setValueSuccess &= configGPS.addCfgValset8(0x10110013, 0); // CFG-NAVSPG-INIFIX3D   (no 3D fix required for initial solution)
              setValueSuccess &= configGPS.addCfgValset32(0x401100c1, 10000); // CFG-NAVSPG-CONSTR_ALT    (100m)

              // ----  gps navx5 input filter ----------------------------------
              // minimum input signals the receiver should use
              // https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#RTK_float-to-fix_recovery_and_false-fix_issues
              //setValueSuccess &= configGPS.addCfgValset8(0x201100a1, 3); // CFG-NAVSPG-INFIL_MINSVS
              //setValueSuccess &= configGPS.addCfgValset8(0x201100a2, 32); // CFG-NAVSPG-INFIL_MAXSVS
              //setValueSuccess &= configGPS.addCfgValset8(0x201100a3, 6); // CFG-NAVSPG-INFIL_MINCNO
              // ----  gps nav5 input filter ----------------------------------
              // minimum condition when the receiver should try a navigation solution
              // https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#RTK_float-to-fix_recovery_and_false-fix_issues
              if (GPS_CONFIG_FILTER)  // custom filter settings
                {
                  setValueSuccess &= configGPS.addCfgValset8(0x201100a4, CPG_CONFIG_FILTER_MINELEV); // CFG-NAVSPG-INFIL_MINELEV  (10 Min SV elevation degree)
                  setValueSuccess &= configGPS.addCfgValset8(0x201100aa, CPG_CONFIG_FILTER_NCNOTHRS); // CFG-NAVSPG-INFIL_NCNOTHRS (10 C/N0 Threshold #SVs)
                  setValueSuccess &= configGPS.addCfgValset8(0x201100ab, CPG_CONFIG_FILTER_CNOTHRS); // CFG-NAVSPG-INFIL_CNOTHRS  (30 dbHz)
                }
              else     // ublox default filter settings
                {
                  setValueSuccess &= configGPS.addCfgValset8(0x201100a4, 10); // CFG-NAVSPG-INFIL_MINELEV  (10 Min SV elevation degree)
                  setValueSuccess &= configGPS.addCfgValset8(0x201100aa, 0);  // CFG-NAVSPG-INFIL_NCNOTHRS (0 C/N0 Threshold #SVs)
                  setValueSuccess &= configGPS.addCfgValset8(0x201100ab, 0);  // CFG-NAVSPG-INFIL_CNOTHRS  (0 dbHz)
                }
              // ----  gps rates ----------------------------------
              setValueSuccess &= configGPS.addCfgValset16(0x30210001, 200); // CFG-RATE-MEAS       (measurement period 200 ms)
              setValueSuccess &= configGPS.sendCfgValset16(0x30210002, 1,   timeout); //CFG-RATE-NAV  (navigation rate cycles 1)
            }
          else if (idx == 2)
            {
              // ----- USB messages (Ardumower) -----------------
              setValueSuccess &= configGPS.newCfgValset8(0x20910009, 0, VAL_LAYER_RAM); // CFG-MSGOUT-UBX_NAV_PVT_USB    (off)
              setValueSuccess &= configGPS.addCfgValset8(0x20910090, 1); // CFG-MSGOUT-UBX_NAV_RELPOSNED_USB  (every solution)
              setValueSuccess &= configGPS.addCfgValset8(0x20910036, 1); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_USB   (every solution)
              setValueSuccess &= configGPS.addCfgValset8(0x20910045, 1); // CFG-MSGOUT-UBX_NAV_VELNED_USB     (every solution)
              setValueSuccess &= configGPS.addCfgValset8(0x2091026b, 5); // CFG-MSGOUT-UBX_RXM_RTCM_USB   (every 5 solutions)
              setValueSuccess &= configGPS.addCfgValset8(0x20910348, 20); // CFG-MSGOUT-UBX_NAV_SIG_USB   (every 20 solutions)
              setValueSuccess &= configGPS.addCfgValset8(0x2091005e, 0); // CFG-MSGOUT-UBX_NAV_TIMEUTC_USB   (off)

              // ----- uart1 messages (Ardumower) -----------------
              setValueSuccess &= configGPS.addCfgValset8(0x20910007, 0); // CFG-MSGOUT-UBX_NAV_PVT_UART1   (off)
              setValueSuccess &= configGPS.addCfgValset8(0x2091008e, 1); // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1  (every solution)
              setValueSuccess &= configGPS.addCfgValset8(0x20910034, 1); // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1   (every solution)
              setValueSuccess &= configGPS.addCfgValset8(0x20910043, 1); // CFG-MSGOUT-UBX_NAV_VELNED_UART1     (every solution)
              setValueSuccess &= configGPS.addCfgValset8(0x20910269, 5); // CFG-MSGOUT-UBX_RXM_RTCM_UART1   (every 5 solutions)
              setValueSuccess &= configGPS.addCfgValset8(0x20910346, 20); // CFG-MSGOUT-UBX_NAV_SIG_UART1   (every 20 solutions)
              setValueSuccess &= configGPS.sendCfgValset8(0x2091005c, 0, timeout); // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1   (off)
            }
          if (setValueSuccess)
            {
              wxLogMessage("OK");
              succeeded++;
              break;
            }
          else
            {

            }
        }
    }
  if (succeeded == 3)
    {
      wxLogMessage("config sent successfully");
      return true;
    }
  else
    {
      wxLogMessage("ERROR: config sending failed");
      return false;
    }
}

void UBLOX::reboot()
{
  wxLogMessage("rebooting GPS receiver...");
  //configGPS.hardReset();
  configGPS.GNSSRestart();
}

void UBLOX::parse(int16_t b)
{
//  if (debug) wxLogMessage(b, HEX);
//  if (debug) wxLogMessage(",");
  if ((b == 0xB5) && (this->state == GOT_NONE))
    {

      //if (debug) wxLogMessage("\n");
      this->state = GOT_SYNC1;
    }

  else if ((b == 0x62) && (this->state == GOT_SYNC1))
    {

      this->state = GOT_SYNC2;
      this->chka = 0;
      this->chkb = 0;
    }

  else if (this->state == GOT_SYNC2)
    {

      this->state = GOT_CLASS;
      this->msgclass = b;
      this->addchk(b);
    }

  else if (this->state == GOT_CLASS)
    {

      this->state = GOT_ID;
      this->msgid = b;
      this->addchk(b);
    }

  else if (this->state == GOT_ID)
    {

      this->state = GOT_LENGTH1;
      this->msglen = b;
      this->addchk(b);
    }

  else if (this->state == GOT_LENGTH1)
    {

      this->state = GOT_LENGTH2;
      this->msglen += (b << 8);
      /*if (debug) {
        wxLogMessage("payload size ");
        wxLogMessage(this->msglen, HEX);
        wxLogMessage(",");
      }*/
      this->count = 0;
      this->addchk(b);
    }

  else if (this->state == GOT_LENGTH2)
    {

      this->addchk(b);
      if ((uint32_t)this->count < sizeof(this->payload))
        {
          this->payload[this->count] = b;
        }
      this->count += 1;

      if (this->count >= this->msglen)
        {

          this->state = GOT_PAYLOAD;
        }
    }

  else if (this->state == GOT_PAYLOAD)
    {

      if (b == this->chka)
        {
          this->state = GOT_CHKA;
        }
      else
        {
          wxLogMessage("ublox chka error, msgclass=");/*
        wxLogMessage(this->msgclass, HEX);
        wxLogMessage(", msgid=");
        wxLogMessage(this->msgid, HEX);
        wxLogMessage(", msglen=");
        wxLogMessage(this->msglen, HEX);
        wxLogMessage(": ");
        wxLogMessage(b, HEX);
        wxLogMessage("!=");
        wxLogMessage(this->chka, HEX);*/
          this->state = GOT_NONE;
          this->chksumErrorCounter++;
        }
    }

  else if (this->state == GOT_CHKA)
    {

      if (b == this->chkb)
        {
          this->dispatchMessage();
          this->state = GOT_NONE;
        }

      else
        {
          wxLogMessage("ublox chkb error, msgclass=");/*
          wxLogMessage(this->msgclass, HEX);
          wxLogMessage(", msgid=");
          wxLogMessage(this->msgid, HEX);
          wxLogMessage(", msglen=");
          wxLogMessage(this->msglen, HEX);
          wxLogMessage(": ");
          wxLogMessage(b, HEX);
          wxLogMessage("!=");
          wxLogMessage(this->chkb, HEX);*/
          this->state = GOT_NONE;
          this->chksumErrorCounter++;
        }
    }
}

void UBLOX::addchk(int16_t b)
{

  this->chka = (this->chka + b) & 0xFF;
  this->chkb = (this->chkb + this->chka) & 0xFF;
}


void UBLOX::dispatchMessage()
{
  if (verbose)
    switch (this->msgclass)
      {
      case 0x01:
        switch (this->msgid)
          {
          case 0x021:
          {
            // UBX-NAV-TIMEUTC
            iTOW = (uint32_t)this->unpack_int32(0);
            year = (unsigned short)this->unpack_int16(12);
            month = (uint8_t)this->unpack_int8(14);
            day = (uint8_t)this->unpack_int8(15);
            hour = (uint8_t)this->unpack_int8(16);
            mins = (uint8_t)this->unpack_int8(17);
            sec = (uint8_t)this->unpack_int8(18);
            /*if (verbose) {
              wxLogMessage("UBX-NAV-TIMEUTC ");
              wxLogMessage("year=");
              wxLogMessage(year);
              wxLogMessage("  month=");
              wxLogMessage(month);
              wxLogMessage("  day=");
              wxLogMessage(day);
              wxLogMessage("  hour=");
              wxLogMessage(hour);
              wxLogMessage("  min=");
              wxLogMessage(mins);
              wxLogMessage("  sec=");
              wxLogMessage(sec);
            }*/
          }
          break;
          case 0x07:
          {
            // UBX-NAV-PVT
            iTOW = (uint32_t)this->unpack_int32(0);
            //numSV = this->unpack_int8(23);
            if (verbose) wxLogMessage("UBX-NAV-PVT");
          }
          break;
          case 0x12:
          {
            // UBX-NAV-VELNED
            iTOW = (uint32_t)this->unpack_int32(0);
            groundSpeed = ((double)((uint32_t)this->unpack_int32(20))) / 100.0;
            heading = ((double)this->unpack_int32(24)) * 1e-5 / 180.0 * PI;
            //wxLogMessage("heading:");
            //wxLogMessage(heading);
            /*if (verbose) {
              wxLogMessage("UBX-NAV-VELNED ");
              wxLogMessage("groundSpeed=");
              wxLogMessage(groundSpeed);
              wxLogMessage("  heading=");
              wxLogMessage(heading);
            }*/
          }
          break;
          case 0x14:
          {
            // UBX-NAV-HPPOSLLH
            iTOW = (uint32_t)this->unpack_int32(4);
            lon = (1e-7  * (this->unpack_int32(8)   +  (this->unpack_int8(24) * 1e-2)));
            lat = (1e-7  * (this->unpack_int32(12)  +  (this->unpack_int8(25) * 1e-2)));
            height = (1e-3 * (this->unpack_int32(16) +  (this->unpack_int8(26) * 1e-2))); // HAE (WGS84 height)
            //height = (1e-3 * (this->unpack_int32(20) +  (this->unpack_int8(27) * 1e-2))); // MSL height
            hAccuracy = ((double)((uint32_t)this->unpack_int32(28))) * 0.1 / 1000.0;
            vAccuracy = ((double)((uint32_t)this->unpack_int32(32))) * 0.1 / 1000.0;
            accuracy = sqrt(SQ(hAccuracy) + SQ(vAccuracy));
            // int32_t hMSL = this->unpack_int32(16);
            //uint32_t hAcc = (uint32_t)this->unpack_int32(20);
            //uint32_t vAcc = (uint32_t)this->unpack_int32(24);
            /*if (verbose) {
              wxLogMessage("UBX-NAV-HPPOSLLH ");
              wxLogMessage("lon=");
              wxLogMessage(lon,8);
              wxLogMessage("  lat=");
              wxLogMessage(lat,8);
            }*/
          }
          break;
          case 0x43:
          {
            // UBX-NAV-SIG
            //if (verbose) wxLogMessage("UBX-NAV-SIG ");
            iTOW = (uint32_t)this->unpack_int32(0);
            int16_t numSigs = this->unpack_int8(5);
            //float ravg = 0;
            float rmax = 0;
            float rmin = 9999;
            float rsum = 0;
            int16_t crcnt = 0;
            int16_t healthycnt = 0;
            for (int16_t i=0; i < numSigs; i++)
              {
                float prRes = ((float)((short)this->unpack_int16(12+16*i))) * 0.1;
                //float cno = ((float)this->unpack_int8(14+16*i));
                //int16_t qualityInd = this->unpack_int8(15+16*i);
                //int16_t corrSource = this->unpack_int8(16+16*i);
                int16_t sigFlags = (unsigned short)this->unpack_int16(18+16*i);
                bool prUsed = ((sigFlags & 8) != 0);
                //bool crUsed = ((sigFlags & 16) != 0);
                //bool doUsed = ((sigFlags & 32) != 0);
                //bool prCorrUsed = ((sigFlags & 64) != 0);
                bool crCorrUsed = ((sigFlags & 128) != 0);
                //bool doCorrUsed = ((sigFlags & 256) != 0);
                bool health = ((sigFlags & 3) == 1);
                if (health)        // signal is healthy
                  {
                    if (prUsed)      // pseudorange has been used (indicates satellites will be also used for carrier correction)
                      {
                        //if (cno > 0){  // signal has some strength (carriar-to-noise)
                        healthycnt++;
                        if (crCorrUsed)   // Carrier range corrections have been used
                          {
                            /*wxLogMessage(sigFlags);
                            wxLogMessage(",");
                            wxLogMessage(qualityInd);
                            wxLogMessage(",");
                            wxLogMessage(prRes);
                            wxLogMessage(",");
                            wxLogMessage(cno); */
                            rsum += fabs(prRes);    // pseudorange residual
                            rmax = MAX(rmax, fabs(prRes));
                            rmin = MIN(rmin, fabs(prRes));
                            crcnt++;
                          }
                      }
                  }
              }
            //ravg = rsum/((float)crcnt);
            numSVdgps = crcnt;
            numSV = healthycnt;
            /*if (verbose){
              wxLogMessage("sol=");
              wxLogMessage(solution);
              wxLogMessage("\t");
              wxLogMessage("hAcc=");
              wxLogMessage(hAccuracy);
              wxLogMessage("\tvAcc=");
              wxLogMessage(vAccuracy);
              wxLogMessage("\t#");
              wxLogMessage(crcnt);
              wxLogMessage("/");
              wxLogMessage(numSigs);
              wxLogMessage("\t");
              wxLogMessage("rsum=");
              wxLogMessage(rsum);
              wxLogMessage("\t");
              wxLogMessage("ravg=");
              wxLogMessage(ravg);
              wxLogMessage("\t");
              wxLogMessage("rmin=");
              wxLogMessage(rmin);
              wxLogMessage("\t");
              wxLogMessage("rmax=");
              wxLogMessage(rmax);
            }*/
          }
          break;
          case 0x3C:
          {
            // UBX-NAV-RELPOSNED
            iTOW = (uint32_t)this->unpack_int32(4);
            relPosN = ((float)this->unpack_int32(8))/100.0;
            relPosE = ((float)this->unpack_int32(12))/100.0;
            relPosD = ((float)this->unpack_int32(16))/100.0;
            solution = (SolType)((this->unpack_int32(60) >> 3) & 3);
            solutionAvail = true;
            solutionTimeout=millis() + 1000;
            /*if (verbose){
              wxLogMessage("UBX-NAV-RELPOSNED ");
              wxLogMessage("n=");
              wxLogMessage(relPosN,2);
              wxLogMessage("  e=");
              wxLogMessage(relPosE,2);
              wxLogMessage("  sol=");
              wxLogMessage(solution);
              wxLogMessage(" ");
              switch(solution){
                case 0:
                  wxLogMessage("invalid");
                  break;
                case 1:
                  wxLogMessage("float");
                  break;
                case 2:
                  wxLogMessage("fix");
                  break;
                default:
                  wxLogMessage("unknown");
                  break;
              }

            }*/
          }
          break;
          }
        break;
      case 0x02:
        switch (this->msgid)
          {
          case 0x32:
          {
            // UBX-RXM-RTCM
            //if (verbose) wxLogMessage("UBX-RXM-RTCM");
            uint8_t flags = (uint8_t)this->unpack_int8(1);
            if ((flags & 1) != 0) dgpsChecksumErrorCounter++;
            dgpsPacketCounter++;
            dgpsAge = millis();
          }
          break;
          }
        break;
      }
}

int32_t UBLOX::unpack_int32(int16_t offset)
{

  return (int32_t)this->unpack(offset, 4);
}

int16_t UBLOX::unpack_int16(int16_t offset)
{

  return (int16_t)this->unpack(offset, 2);
}

int8_t UBLOX::unpack_int8(int16_t offset)
{

  return (int8_t)this->unpack(offset, 1);
}

int32_t UBLOX::unpack(int16_t offset, int16_t size)
{

  int32_t value = 0; // use the good type

  for (uint8_t k=0; k<size; ++k)
    {
      value <<= 8;
      value |= (0xFF & this->payload[offset+size-k-1]);
    }

  return value;
}



/* parse the uBlox data */
void UBLOX::run()
{
  if (millis() > solutionTimeout)
    {
      //wxLogMessage("UBLOX::solutionTimeout");
      solution = SOL_INVALID;
      solutionTimeout = millis() + 1000;
      solutionAvail = true;
    }

  // read a uint8_t from the serial port
  if (!_bus->available()) return;
  while (_bus->available())
    {
      uint8_t data = _bus->read();
      parse(data);
#ifdef GPS_DUMP
      /*if (data == 0xB5) wxLogMessage("\n");
      wxLogMessage(data, HEX);
      wxLogMessage(",");*/
#endif
    }
}


