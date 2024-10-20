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

#include "NutsBolts.h"

volatile bool StartRunDone = false;
volatile bool StartIsRunning = false;
volatile bool LoopIsRunning = false;

uint32_t startMillis = 0;

uint32_t micros()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return 1000000 * tv.tv_sec + tv.tv_usec - startMillis*1000;
}

uint32_t millis()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);
  return 1000 * tv.tv_sec + tv.tv_usec/1000 - startMillis;
}

void delay(uint32_t m)
{
  usleep(m * 1000);
}

void delayMicroseconds(uint32_t m)
{
  usleep(m);
}

void randomSeed(uint32_t seed)
{
  if (seed != 0)
    {
      srandom(seed);
    }
}


int32_t random(int32_t howsmall, int32_t howbig)
{
  if (howsmall >= howbig)
    {
      return howsmall;
    }
  int32_t diff = howbig - howsmall;
  return random(diff) + howsmall;
}

int32_t random(int32_t howbig)
{
  if (howbig == 0)
    {
      return 0;
    }
  return random() % howbig;
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static pthread_mutex_t thread_mutexes[10];

void thread_yield()
{
  sched_yield();
}

pthread_t thread_self()
{
  return pthread_self();
}

pthread_t thread_create(thread_fn fn, void * arg)
{
  pthread_t myThread ;
  if(pthread_create(&myThread, NULL, fn, arg) != 0)
    {
      return 0;
    }
  pthread_detach(myThread);
  return myThread;
}

int16_t thread_set_name(pthread_t t, const char *name)
{
  return pthread_setname_np(t, name);
}

int16_t thread_set_priority(const int16_t pri)
{
  struct sched_param sched ;
  memset (&sched, 0, sizeof(sched)) ;
  if (pri > sched_get_priority_max(SCHED_OTHER))
    {
      sched.sched_priority = sched_get_priority_max(SCHED_OTHER);
    }
  else
    {
      sched.sched_priority = pri ;
    }
  return sched_setscheduler(0, SCHED_OTHER, &sched) ;
}

int16_t thread_detach(pthread_t t)
{
  return pthread_detach(t);
}

int16_t thread_terminate(pthread_t t)
{
  return pthread_cancel(t);
}

uint8_t thread_running(pthread_t t)
{
  int16_t r = pthread_tryjoin_np(t, NULL);
  return (r==0 || r==EBUSY);
}

uint8_t thread_equals(pthread_t t)
{
  return pthread_equal(pthread_self(),t);
}

void thread_lock(int16_t index)
{
  pthread_mutex_lock(&thread_mutexes[index]);
}

void thread_unlock(int16_t index)
{
  pthread_mutex_unlock(&thread_mutexes[index]);
}
