#include <zephyr/drivers/console/sdi_console.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/devicetree.h>

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sdi), okay)
#define DMDATA0_ADDR DT_REG_ADDR(DT_NODELABEL(sdi))
#define DMDATA1_ADDR (DMDATA0_ADDR + 4)
#else
// Fallback to V4 default if no node found
#define DMDATA0_ADDR 0xe0000380
#define DMDATA1_ADDR 0xe0000384
#endif

#define DMDATA0 (*(volatile uint32_t *)DMDATA0_ADDR)
#define DMDATA1 (*(volatile uint32_t *)DMDATA1_ADDR)

#define SDI_TIMEOUT 0x80000

void sdi_console_init(void)
{
    DMDATA1 = 0x00;
    DMDATA0 = 0x80;
}

static void sdi_write_chunk(const char *buf, int len)
{
    uint32_t timeout = SDI_TIMEOUT;

    if (len > 7) len = 7;

    while ((DMDATA0 & 0x80) && timeout--) {
        // Wait for host to acknowledge previous data
    }

    if (timeout == 0) return;

    uint32_t d0 = (len + 4) | 0x80;
    uint32_t d1 = 0;
    uint8_t *p0 = (uint8_t *)&d0;
    uint8_t *p1 = (uint8_t *)&d1;

    for (int i = 0; i < len; i++) {
        if (i < 3) {
            p0[i + 1] = buf[i];
        } else {
            p1[i - 3] = buf[i];
        }
    }

    DMDATA1 = d1;
    DMDATA0 = d0;
}

void sdi_console_puts(const char *str)
{
    int len = strlen(str);
    int pos = 0;
    while (pos < len) {
        int chunk_len = len - pos;
        if (chunk_len > 7) chunk_len = 7;
        sdi_write_chunk(str + pos, chunk_len);
        pos += chunk_len;
    }
}

#include <zephyr/init.h>
#include <zephyr/sys/printk-hooks.h>
#include <zephyr/sys/libc-hooks.h>

void sdi_console_printf(const char *format, ...)
{
    char buf[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len > 0) {
        sdi_console_puts(buf);
    }
}

static int sdi_console_out(int character)
{
    char c = (char)character;
    sdi_write_chunk(&c, 1);
    return character;
}

static int sdi_console_sys_init(void)
{
    sdi_console_init();
    __printk_hook_install(sdi_console_out);
    __stdout_hook_install(sdi_console_out);
    return 0;
}



// SYS_INIT commented out to verify boot
SYS_INIT(sdi_console_sys_init, PRE_KERNEL_1, CONFIG_CONSOLE_INIT_PRIORITY);
