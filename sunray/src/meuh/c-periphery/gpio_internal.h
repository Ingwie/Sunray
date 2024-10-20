/*
 * c-periphery
 * https://github.com/vsergeev/c-periphery
 * License: MIT
 */

#ifndef _PERIPHERY_GPIO_INTERNAL_H
#define _PERIPHERY_GPIO_INTERNAL_H

#include <stdarg.h>

#include "gpio.h"

/*********************************************************************************/
/* Operations table and handle structure */
/*********************************************************************************/

struct gpio_ops
{
  int16_t (*read)(gpio_t *gpio, bool *value);
  int16_t (*write)(gpio_t *gpio, bool value);
  int16_t (*read_event)(gpio_t *gpio, gpio_edge_t *edge, uint64_t *timestamp);
  int16_t (*poll)(gpio_t *gpio, int16_t timeout_ms);
  int16_t (*close)(gpio_t *gpio);
  int16_t (*get_direction)(gpio_t *gpio, gpio_direction_t *direction);
  int16_t (*get_edge)(gpio_t *gpio, gpio_edge_t *edge);
  int16_t (*get_bias)(gpio_t *gpio, gpio_bias_t *bias);
  int16_t (*get_drive)(gpio_t *gpio, gpio_drive_t *drive);
  int16_t (*get_inverted)(gpio_t *gpio, bool *inverted);
  int16_t (*set_direction)(gpio_t *gpio, gpio_direction_t direction);
  int16_t (*set_edge)(gpio_t *gpio, gpio_edge_t edge);
  int16_t (*set_bias)(gpio_t *gpio, gpio_bias_t bias);
  int16_t (*set_drive)(gpio_t *gpio, gpio_drive_t drive);
  int16_t (*set_inverted)(gpio_t *gpio, bool inverted);
  uint16_t (*line)(gpio_t *gpio);
  int16_t (*fd)(gpio_t *gpio);
  int16_t (*name)(gpio_t *gpio, char *str, size_t len);
  int16_t (*label)(gpio_t *gpio, char *str, size_t len);
  int16_t (*chip_fd)(gpio_t *gpio);
  int16_t (*chip_name)(gpio_t *gpio, char *str, size_t len);
  int16_t (*chip_label)(gpio_t *gpio, char *str, size_t len);
  int16_t (*tostring)(gpio_t *gpio, char *str, size_t len);
};

struct gpio_handle
{
  const struct gpio_ops *ops;

  union
  {
    struct
    {
      uint16_t line;
      int16_t line_fd;
      int16_t chip_fd;
      gpio_direction_t direction;
      gpio_edge_t edge;
      gpio_bias_t bias;
      gpio_drive_t drive;
      bool inverted;
      char label[32];
    } cdev;
    struct
    {
      uint16_t line;
      int16_t line_fd;
      bool exported;
    } sysfs;
  } u;

  /* error state */
  struct
  {
    int16_t c_errno;
    char errmsg[96];
  } error;
};

/*********************************************************************************/
/* Common error formatting function */
/*********************************************************************************/

inline static int16_t _gpio_error(gpio_t *gpio, int16_t code, int16_t c_errno, const char *fmt, ...)
{
  va_list ap;

  gpio->error.c_errno = c_errno;

  va_start(ap, fmt);
  vsnprintf(gpio->error.errmsg, sizeof(gpio->error.errmsg), fmt, ap);
  va_end(ap);

  /* Tack on strerror() and errno */
  if (c_errno)
    {
      char buf[64];
      strerror_r(c_errno, buf, sizeof(buf));
      snprintf(gpio->error.errmsg+strlen(gpio->error.errmsg), sizeof(gpio->error.errmsg)-strlen(gpio->error.errmsg), ": %s [errno %d]", buf, c_errno);
    }

  return code;
}

#endif

