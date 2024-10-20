#include "reset.h"
#include "config.h"
#include "NutsBolts.h"

ResetCause getResetCause()
{
  return RST_UNKNOWN;
}



void logResetCause()
{
  wxLogMessage("RESET cause: ");
  switch (getResetCause())
    {
    case RST_UNKNOWN:
      wxLogMessage("unknown");
      break;
    case RST_POWER_ON :
      wxLogMessage("power-on");
      break;
    case RST_EXTERNAL :
      wxLogMessage("external");
      break;
    case RST_BROWN_OUT :
      wxLogMessage("brown-out");
      break;
    case RST_WATCHDOG :
      wxLogMessage("watchdog");
      break;
    case RST_SOFTWARE :
      wxLogMessage("software");
      break;
    case RST_BACKUP:
      wxLogMessage("backup");
      break;
    }
}


// get free memory
// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
int32_t freeMemory()
{
  return 1000000;
//TODO
}

