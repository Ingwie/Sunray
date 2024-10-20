/*
.-------.        ,-----.     _______       ,-----.  ,---------. ,---.    ,---.    .-''-.    ___    _ .---.  .---.
|  _ _   \     .'  .-,  '.  \  ____  \   .'  .-,  '.\          \|    \  /    |  .'_ _   \ .'   |  | ||   |  |_ _|
| ( ' )  |    / ,-.|  \ _ \ | |    \ |  / ,-.|  \ _ \`--.  ,---'|  ,  \/  ,  | / ( ` )   '|   .'  | ||   |  ( ' )
|(_ o _) /   ;  \  '_ /  | :| |____/ / ;  \  '_ /  | :  |   \   |  |\_   /|  |. (_ o _)  |.'  '_  | ||   '-(_{;}_)
| (_,_).' __ |  _`,/ \ _/  ||   _ _ '. |  _`,/ \ _/  |  :_ _:   |  _( )_/ |  ||  (_,_)___|'   ( \.-.||      (_,_)
|  |\ \  |  |: (  '\_/ \   ;|  ( ' )  \: (  '\_/ \   ;  (_I_)   | (_ o _) |  |'  \   .---.' (`. _` /|| _ _--.   |
|  | \ `'   / \ `"/  \  ) / | (_{;}_) | \ `"/  \  ) /  (_(=)_)  |  (_,_)  |  | \  `-'    /| (_ (_) _)|( ' ) |   |
|  |  \    /   '. \_/``".'  |  (_,_)  /  '. \_/``".'    (_I_)   |  |      |  |  \       /  \ /  . \ /(_{;}_)|   |
''-'   `'-'      '-----'    /_______.'     '-----'      '---'   '--'      '--'   `'-..-'    ``-'`-'' '(_,_) '---'
*/
/*          Copyright 2023-2024 by Ingwie (Bracame)    */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*    Ardumower Alfred mod to drive my autoclip 325    */

#ifndef NUTSBOLT_H
#define NUTSBOLT_H

#include <inttypes.h>
#include <sys/time.h>
#include <stddef.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <sched.h>
#include <errno.h>
#include <string.h>

#include <wx/log.h>
#include <wx/process.h>
#include <wx/file.h>

#define HIGH 0x1
#define LOW  0x0

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define ABS(x) ((x)>0?(x):-(x))
#define CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define ROUND(x)     ((x)>=0?(int32_t)((x)+0.5):(int32_t)((x)-0.5))
#define RADIAN(deg) ((deg)*DEG_TO_RAD)
#define DEGREES(rad) ((rad)*RAD_TO_DEG)
#define SQ(x) ((x)*(x))

#define LSBFIRST 0
#define MSBFIRST 1

extern volatile bool StartRunDone;
extern volatile bool StartIsRunning;
extern volatile bool LoopIsRunning;

extern wxString AppPath;
uint32_t micros();
uint32_t millis();

void delay(uint32_t m);
void delayMicroseconds(uint32_t m);

void randomSeed(uint32_t seed);
int32_t random(int32_t howsmall, int32_t howbig);
int32_t random(int32_t howbig);

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

// wxFile macro
#define WXFILEREAD(file, data, sucess)   sucess &= (file.Read((uint8_t*)&data, sizeof(data)) == sizeof(data))
#define WXFILEWRITE(file, data, sucess)   sucess &= (file.Write((uint8_t*)&data, sizeof(data)) == sizeof(data))



//Threads
typedef void *(*thread_fn)(void *);
void      thread_yield();
pthread_t thread_self();
pthread_t thread_create(thread_fn fn, void * arg);
int16_t       thread_set_name(pthread_t t, const char *name);
int16_t       thread_set_priority(const int16_t pri);
int16_t       thread_detach(pthread_t t);
int16_t       thread_terminate(pthread_t t);
uint8_t   thread_running(pthread_t t);
uint8_t   thread_equals(pthread_t t);
void      thread_lock(int16_t index);
void      thread_unlock(int16_t index);


#endif
