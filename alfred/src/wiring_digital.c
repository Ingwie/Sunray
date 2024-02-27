/*
  GPIO API for Linux modded for fast read/write
*/

#include "Arduino.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/stat.h>


int fd[53] = {0}; // Last GPIO usable is number 53, file descriptors keeped open

void gpio_unexport(int gpio)
{
  char buf[50];
  close(fd[gpio]);
  fd[gpio] = open("/sys/class/gpio/unexport", O_WRONLY | O_SYNC);
  sprintf(buf, "%d", gpio);
  write(fd[gpio], buf, strlen(buf));
  close(fd[gpio]);
}


int gpio_export(int gpio)
{
  char buf[50];
  int gpiofd, ret;

  /* Quick test if it has already been exported */
  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd[gpio] = open(buf, O_WRONLY);
  if(fd[gpio] != -1)
    {
      // already exported -> unexport
      gpio_unexport(gpio);
    }

  fd[gpio] = open("/sys/class/gpio/export", O_WRONLY | O_SYNC);

  if(fd[gpio] != -1)
    {
      sprintf(buf, "%d", gpio);
      ret = write(fd[gpio], buf, strlen(buf));
      if(ret < 0)
        {
          perror("error: GPIO export failed");
          return -2;
        }
      close(fd[gpio]);
    }
  else
    {
      // If we can't open the export file, we probably
      // dont have any gpio permissions
      perror("error: no GPIO permission");
      return -1;
    }
  return 0;
}


int gpio_getfd(int gpio)
{
  return fd[gpio];
}


int gpio_setedge(int gpio, int rising, int falling)
{
  int ret = 0;
  char buf[50];
  sprintf(buf, "/sys/class/gpio/gpio%d/edge", gpio);
  int gpiofd = open(buf, O_WRONLY);
  if(gpiofd < 0)
    {
      perror("Couldn't open IRQ file");
      ret = -1;
    }

  if(gpiofd && rising && falling)
    {
      if(4 != write(gpiofd, "both", 4))
        {
          perror("Failed to set IRQ to both falling & rising");
          ret = -2;
        }
    }
  else
    {
      if(rising && gpiofd)
        {
          if(6 != write(gpiofd, "rising", 6))
            {
              perror("Failed to set IRQ to rising");
              ret = -2;
            }
        }
      else if(falling && gpiofd)
        {
          if(7 != write(gpiofd, "falling", 7))
            {
              perror("Failed to set IRQ to falling");
              ret = -3;
            }
        }
    }

  close(gpiofd);
}


// ---------------------------------------------------

void pinMode(uint8_t gpio, uint8_t mode)
{
  gpio_export(gpio);
  int ret = 0;
  char buf[50];
  sprintf(buf, "/sys/class/gpio/gpio%d/direction", gpio);
  fd[gpio] = open(buf, O_WRONLY | O_SYNC);
  if(fd[gpio] < 0)
    {
      perror("Couldn't open direction file");
      ret = -1;
    }
  if((mode == OUTPUT) && (fd[gpio]))
    {
      if (4 != write(fd[gpio], "out", 4))
        {
          fprintf(stderr, "Couldn't set GPIO %d direction to out: %s\n",
                  gpio,
                  strerror(errno));
          ret = -2;
        }
    }
  else if(fd[gpio])
    {
      if(3 != write(fd[gpio], "in", 3))
        {
          fprintf(stderr, "Couldn't set GPIO %d direction to in: %s\n",
                  gpio,
                  strerror(errno));
          ret = -3;
        }
    }

  close(fd[gpio]);
  sprintf(buf, "/sys/class/gpio/gpio%d/value", gpio);
  fd[gpio] = open(buf, O_RDWR);
}

void digitalWrite(uint8_t gpio, uint8_t val)
{
  if(fd[gpio] > 0)
    {
      if (val == 0)
        {
          write(fd[gpio], "0", 1);
        }
      else
        {
          write(fd[gpio], "1", 1);
        }

      lseek(fd[gpio], 0, SEEK_SET);
    }
}

int digitalRead(uint8_t gpio)
{
  char in[3] = {0, 0, 0};
  int c = read(fd[gpio], in, 3);
  lseek(fd[gpio], 0, SEEK_SET);
  int ret = 1;
  if (in[0] == '0') ret = 0;
  return ret;
}

