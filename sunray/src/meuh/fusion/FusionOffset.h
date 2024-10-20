/**
 * @file FusionOffset.h
 * @author Seb Madgwick
 * @brief Gyroscope offset correction algorithm for run-time calibration of the
 * gyroscope offset.
 */

#ifndef FUSION_OFFSET_H
#define FUSION_OFFSET_H

//------------------------------------------------------------------------------
// Includes

#include "FusionMath.h"

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Gyroscope offset algorithm structure.  Structure members are used
 * internally and must not be accessed by the application.
 */
typedef struct
{
  float filterCoefficient;
  uint16_t timeout;
  uint16_t timer;
  FusionVector gyroscopeOffset;
} FusionOffset;

//------------------------------------------------------------------------------
// Function declarations

void FusionOffsetInitialise(FusionOffset *const offset, const uint16_t sampleRate);

FusionVector FusionOffsetUpdate(FusionOffset *const offset, FusionVector gyroscope);

#endif

//------------------------------------------------------------------------------
// End of file
