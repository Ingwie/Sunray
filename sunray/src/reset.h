#ifndef RESET_H
#define RESET_H

#include "inttypes.h"


enum ResetCause
{
  RST_UNKNOWN,
  RST_POWER_ON,
  RST_EXTERNAL,
  RST_BROWN_OUT,
  RST_WATCHDOG,
  RST_SOFTWARE,
  RST_BACKUP,
};

ResetCause getResetCause();
void logResetCause();
int32_t freeMemory();

#endif

