/*
 * c-periphery
 * https://github.com/vsergeev/c-periphery
 * License: MIT
 */

#ifndef _PERIPHERY_SERIAL_H
#define _PERIPHERY_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

enum serial_error_code
{
  SERIAL_ERROR_ARG            = -1, /* Invalid arguments */
  SERIAL_ERROR_OPEN           = -2, /* Opening serial port */
  SERIAL_ERROR_QUERY          = -3, /* Querying serial port attributes */
  SERIAL_ERROR_CONFIGURE      = -4, /* Configuring serial port attributes */
  SERIAL_ERROR_IO             = -5, /* Reading/writing serial port */
  SERIAL_ERROR_CLOSE          = -6, /* Closing serial port */
};

typedef enum serial_parity
{
  PARITY_NONE,
  PARITY_ODD,
  PARITY_EVEN,
} serial_parity_t;

typedef struct serial_handle serial_t;

/* Primary Functions */
serial_t *serial_new(void);
int16_t serial_open(serial_t *serial, const char *path, uint32_t baudrate);
int16_t serial_open_advanced(serial_t *serial, const char *path,
                             uint32_t baudrate, uint16_t databits,
                             serial_parity_t parity, uint16_t stopbits,
                             bool xonxoff, bool rtscts);
int16_t serial_read(serial_t *serial, uint8_t *buf, size_t len, int16_t timeout_ms);
int16_t serial_write(serial_t *serial, const uint8_t *buf, size_t len);
int16_t serial_flush(serial_t *serial);
int16_t serial_input_waiting(serial_t *serial, uint16_t *count);
int16_t serial_output_waiting(serial_t *serial, uint16_t *count);
int16_t serial_poll(serial_t *serial, int16_t timeout_ms);
int16_t serial_close(serial_t *serial);
void serial_free(serial_t *serial);

/* Getters */
int16_t serial_get_baudrate(serial_t *serial, uint32_t *baudrate);
int16_t serial_get_databits(serial_t *serial, uint16_t *databits);
int16_t serial_get_parity(serial_t *serial, serial_parity_t *parity);
int16_t serial_get_stopbits(serial_t *serial, uint16_t *stopbits);
int16_t serial_get_xonxoff(serial_t *serial, bool *xonxoff);
int16_t serial_get_rtscts(serial_t *serial, bool *rtscts);
int16_t serial_get_vmin(serial_t *serial, uint16_t *vmin);
int16_t serial_get_vtime(serial_t *serial, float* vtime);

/* Setters */
int16_t serial_set_baudrate(serial_t *serial, uint32_t baudrate);
int16_t serial_set_databits(serial_t *serial, uint16_t databits);
int16_t serial_set_parity(serial_t *serial, enum serial_parity parity);
int16_t serial_set_stopbits(serial_t *serial, uint16_t stopbits);
int16_t serial_set_xonxoff(serial_t *serial, bool enabled);
int16_t serial_set_rtscts(serial_t *serial, bool enabled);
int16_t serial_set_vmin(serial_t *serial, uint16_t vmin);
int16_t serial_set_vtime(serial_t *serial, float vtime);

/* Miscellaneous */
int16_t serial_fd(serial_t *serial);
int16_t serial_tostring(serial_t *serial, char *str, size_t len);

/* Error Handling */
int16_t serial_errno(serial_t *serial);
const char *serial_errmsg(serial_t *serial);

#ifdef __cplusplus
}
#endif

#endif

