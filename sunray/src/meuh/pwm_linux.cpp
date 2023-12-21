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
/*          Copyright 2023 by Ingwie (Bracame)         */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*    Ardumower Alfred mod to drive my autoclip 325    */

// Linux pwm driver

#include "pwm_linux.h"

int pwmExport(int pwmio)
{
  char buf[50];
  int efd, ret;

  efd = open("/sys/class/pwm/pwmchip0/export", O_WRONLY | O_SYNC);

  if(efd != -1)
    {
      sprintf(buf, "%d", pwmio);
      ret = write(efd, buf, strlen(buf));
      if(ret < 0)
        {
          perror("error: PWM export failed");
          return -2;
        }
      close(efd);
    }
  else
    {
      // If we can't open the export file, we probably
      // dont have any pwmio permissions
      perror("error: no PWM permission");
      return -1;
    }
  return 0;
}

void pwmUnexport(int pwmio)
{
  int pwmfd, ret;
  char buf[50];
  pwmfd = open("/sys/class/pwm/pwmchip0/unexport", O_WRONLY | O_SYNC);
  sprintf(buf, "%d", pwmio);
  ret = write(pwmfd, buf, strlen(buf));
  close(pwmfd);
}

int pwmSetEnable(int pwmio, int enable)
{
  char buf[50];
  int pwmfd, ret;
  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/enable", pwmio);
  pwmfd = open(buf, O_WRONLY);
  if(pwmfd < 0)
    {
      perror("error opening PWM enable");
      return -1;
    }
  sprintf(buf, "%d", enable);
  ret = write(pwmfd, buf, strlen(buf));
  if(ret < 0)
    {
      perror("failed to set PWM enable");
      close(pwmfd);
      return -1;
    }
  close(pwmfd);
  return 1;
}

int pwmSetPolarity(int pwmio, int polarity)
{
  char buf[50];
  int pwmfd, ret;
  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/polarity", pwmio);
  pwmfd = open(buf, O_WRONLY);
  if(pwmfd < 0)
    {
      perror("error opening PWM polarity");
      return -1;
    }
  if (polarity == 0) {
  char pol[] = "normal";
  ret = write(pwmfd, pol, strlen(pol));
  } else {
  char pol[] = "inversed";
  ret = write(pwmfd, pol, strlen(pol));
  }
  if(ret < 0)
    {
      perror("failed to set PWM polarity");
      close(pwmfd);
      return -1;
    }
  close(pwmfd);
  return 1;
}

int pwmSetPeriod(int pwmio, uint32_t period)
{
  char buf[50];
  int pwmfd, ret;
  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/period", pwmio);
  pwmfd = open(buf, O_WRONLY);
  if(pwmfd < 0)
    {
      perror("error opening PWM period");
      return -1;
    }
  sprintf(buf, "%d", period);
  ret = write(pwmfd, buf, strlen(buf));
  if (ret < 0)
    {
      perror("failed to set PWM period");
      close(pwmfd);
      return -1;
    }
  close(pwmfd);
  return 1;
}

int pwmSetDutyCycle(int pwmio, uint32_t duty)
{
  char buf[50];
  int pwmfd, ret;
  sprintf(buf, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pwmio);
  pwmfd = open(buf, O_WRONLY);
  if(pwmfd < 0)
    {
      perror("error opening PWM duty_cycle");
      return -1;
    }
  sprintf(buf, "%d", duty);
  ret = write(pwmfd, buf, strlen(buf));
  if(ret < 0)
    {
      perror("failed to set PWM duty_cylce");
      close(pwmfd);
      return -1;
    }
  close(pwmfd);
  return 1;
}
