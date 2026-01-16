#include <zephyr/drivers/console/sdi_console.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DMDATA0_ADDR 0xe0000380
#define DMDATA1_ADDR 0xe0000384

#define DMDATA0 (*(volatile uint32_t *)DMDATA0_ADDR)
#define DMDATA1 (*(volatile uint32_t *)DMDATA1_ADDR)

#define SDI_TIMEOUT 0x80000

void sdi_console_init(void)
{
    // Usually no init needed for DMDATA, but we can ensure it's cleared
    DMDATA0 = 0x84; // Reset state for minichlink
}

static void sdi_write_chunk(const char *buf, int len)
{
    uint32_t timeout = SDI_TIMEOUT;
    while ((DMDATA0 & 0x80) && timeout--) {
        // Wait for host to acknowledge previous data
    }

    if (timeout == 0) return;

    char chunk[8] = {0};
    memcpy(chunk, buf, len);

    DMDATA1 = *(uint32_t *)(chunk + 3);
    DMDATA0 = (len + 4) | (chunk[0] << 8) | (chunk[1] << 16) | (chunk[2] << 24) | 0x80;
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
    char buf[256];
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
