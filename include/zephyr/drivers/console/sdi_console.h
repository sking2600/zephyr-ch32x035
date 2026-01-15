#ifndef DIRECT_SDI_CONSOLE_H
#define DIRECT_SDI_CONSOLE_H

#include <stdint.h>

/**
 * @brief Initialize the Direct SDI Console.
 * 
 * This enables the necessary debug module registers if needed.
 */
void sdi_console_init(void);

/**
 * @brief Write a string to the Direct SDI Console using DMDATA registers.
 * 
 * @param str The null-terminated string to write.
 */
void sdi_console_puts(const char *str);

/**
 * @brief Write formatted output to the Direct SDI Console.
 * 
 * @param format The format string.
 */
void sdi_console_printf(const char *format, ...);

#endif /* DIRECT_SDI_CONSOLE_H */
