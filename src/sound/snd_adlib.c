/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Adlib emulation + embedded Z80 sidecore (6 MHz) with ROM loader.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Jasmine Iwanek, <jriwanek@gmail.com>
 *
 *          Z80 integration: for Clara (2025)
 *
 *          Copyright 2008-2018 Sarah Walker.
 *          Copyright 2016-2025 Miran Grca.
 *          Copyright 2024-2025 Jasmine Iwanek.
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H

#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mca.h>
#include <86box/sound.h>
#include <86box/timer.h>
#include <86box/snd_opl.h>
#include <86box/plat_unused.h>

/* ======================= Z80 integration =========================== */
#include "z80.h"   /* from your uploaded libz80 (z80.c/z80.h) */

/* clock & cadence knobs */
#ifndef ADLIB_Z80_CLOCK_HZ
#  define ADLIB_Z80_CLOCK_HZ   6000000u   /* 6.000 MHz effective */
#endif
#ifndef ADLIB_Z80_TICK_US
#  define ADLIB_Z80_TICK_US    100.0      /* execute every 100 µs => 600 T-states/tick */
#endif

/* ROM source knobs */
#ifndef ADLIB_Z80_ROM_ENV
#  define ADLIB_Z80_ROM_ENV    "ADLIB_Z80_ROM"
#endif
#ifndef ADLIB_Z80_ROM_DEFAULT
#  define ADLIB_Z80_ROM_DEFAULT "adlib_z80.bin"
#endif
/* ================================================================== */

#ifdef ENABLE_ADLIB_LOG
int adlib_do_log = ENABLE_ADLIB_LOG;
static void adlib_log(const char *fmt, ...)
{
    va_list ap;
    if (adlib_do_log) {
        va_start(ap, fmt);
        pclog_ex(fmt, ap);
        va_end(ap);
    }
}
#else
#  define adlib_log(fmt, ...)
#endif

typedef struct adlib_s {
    fm_drv_t   opl;
    uint8_t    pos_regs[8];

    /* --------- Z80 state --------- */
    Z80Context z80;
    pc_timer_t z80_timer;
    double     z80_tick_us;
    unsigned   z80_clock_hz;
    uint8_t    z80_mem[0x10000];   /* flat 64 KiB */
    size_t     z80_rom_len;        /* ROM window size (read-only) */
} adlib_t;

/* --------- Z80 memory / I/O glue ---------- */
static byte z80_mem_read_cb(size_t param, ushort addr)
{
    adlib_t *d = (adlib_t *)(uintptr_t)param;
    return d->z80_mem[addr];
}
static void z80_mem_write_cb(size_t param, ushort addr, byte data)
{
    adlib_t *d = (adlib_t *)(uintptr_t)param;
    /* keep ROM region read-only */
    if (addr < d->z80_rom_len) return;
    d->z80_mem[addr] = data;
}
static byte z80_io_read_cb(size_t param, ushort port)
{
    (void)param; (void)port;
    /* TODO: wire to your PIU/USART/OPP as needed */
    return 0xFF;
}
static void z80_io_write_cb(size_t param, ushort port, byte data)
{
    (void)param; (void)port; (void)data;
    /* TODO: wire to your PIU/USART/OPP as needed */
}

/* load ROM into 0x0000.., return bytes loaded (<= 65536) */
static size_t adlib_z80_load_rom(adlib_t *d)
{
    const char *path = getenv(ADLIB_Z80_ROM_ENV);
    if (!path || !*path) path = ADLIB_Z80_ROM_DEFAULT;

    FILE *f = fopen(path, "rb");
    if (!f) {
        adlib_log("Z80: ROM not found (%s), running blank RAM\n", path);
        return 0;
    }
    size_t n = fread(d->z80_mem, 1, sizeof(d->z80_mem), f);
    fclose(f);
    adlib_log("Z80: ROM loaded: %zu bytes from %s\n", n, path);
    return n;
}

/* timer callback -> execute T-states then reschedule */
static void adlib_z80_timer_cb(void *priv)
{
    adlib_t *d = (adlib_t *)priv;
    unsigned tstates = (unsigned)((double)d->z80_clock_hz * (d->z80_tick_us * 1e-6));
    if (!tstates) tstates = 1;
    Z80ExecuteTStates(&d->z80, tstates);
    timer_on_auto(&d->z80_timer, d->z80_tick_us);
}

static void adlib_z80_init(adlib_t *d)
{
    memset(d->z80_mem, 0x00, sizeof(d->z80_mem));
    d->z80_rom_len = adlib_z80_load_rom(d);

    d->z80.memRead  = z80_mem_read_cb;
    d->z80.memWrite = z80_mem_write_cb;
    d->z80.memParam = (size_t)(uintptr_t)d;

    d->z80.ioRead   = z80_io_read_cb;
    d->z80.ioWrite  = z80_io_write_cb;
    d->z80.ioParam  = (size_t)(uintptr_t)d;

    Z80RESET(&d->z80);
    /* If your ROM entry point differs, set PC explicitly:
       d->z80.PC = 0x0000; */

    d->z80_clock_hz = ADLIB_Z80_CLOCK_HZ;
    d->z80_tick_us  = ADLIB_Z80_TICK_US;

    timer_add(&d->z80_timer, adlib_z80_timer_cb, d, 0);
    timer_on_auto(&d->z80_timer, d->z80_tick_us);

    adlib_log("Z80: init complete @ %.3f MHz, tick %.1f us\n",
              d->z80_clock_hz / 1e6, d->z80_tick_us);
}
/* ==================== end Z80 integration ========================== */

static void
adlib_get_buffer(int32_t *buffer, int len, void *priv)
{
    adlib_t *adlib = (adlib_t *) priv;

    const int32_t *opl_buf = adlib->opl.update(adlib->opl.priv);

    for (int c = 0; c < len * 2; c++)
        buffer[c] += opl_buf[c];

    adlib->opl.reset_buffer(adlib->opl.priv);
}

uint8_t
adlib_mca_read(int port, void *priv)
{
    const adlib_t *adlib = (adlib_t *) priv;

    adlib_log("adlib_mca_read: port=%04x\n", port);

    return adlib->pos_regs[port & 7];
}

void
adlib_mca_write(int port, uint8_t val, void *priv)
{
    adlib_t *adlib = (adlib_t *) priv;

    if (port < 0x102)
        return;

    adlib_log("adlib_mca_write: port=%04x val=%02x\n", port, val);

    switch (port) {
        case 0x102:
            if ((adlib->pos_regs[2] & 1) && !(val & 1))
                io_removehandler(0x0388, 0x0002,
                                 adlib->opl.read, NULL, NULL,
                                 adlib->opl.write, NULL, NULL,
                                 adlib->opl.priv);
            if (!(adlib->pos_regs[2] & 1) && (val & 1))
                io_sethandler(0x0388, 0x0002,
                              adlib->opl.read, NULL, NULL,
                              adlib->opl.write, NULL, NULL,
                              adlib->opl.priv);
            break;

        default:
            break;
    }
    adlib->pos_regs[port & 7] = val;
}

uint8_t
adlib_mca_feedb(void *priv)
{
    const adlib_t *adlib = (adlib_t *) priv;

    return (adlib->pos_regs[2] & 1);
}

void *
adlib_init(UNUSED(const device_t *info))
{
    adlib_t *adlib = calloc(1, sizeof(adlib_t));

    adlib_log("adlib_init\n");
    fm_driver_get(FM_YM3812, &adlib->opl);
    io_sethandler(0x0388, 0x0002,
                  adlib->opl.read, NULL, NULL,
                  adlib->opl.write, NULL, NULL,
                  adlib->opl.priv);
    music_add_handler(adlib_get_buffer, adlib);

    /* bring up Z80 sidecore */
    adlib_z80_init(adlib);

    return adlib;
}

void *
adlib_mca_init(const device_t *info)
{
    adlib_t *adlib = adlib_init(info);

    io_removehandler(0x0388, 0x0002,
                     adlib->opl.read, NULL, NULL,
                     adlib->opl.write, NULL, NULL,
                     adlib->opl.priv);
    mca_add(adlib_mca_read,
            adlib_mca_write,
            adlib_mca_feedb,
            NULL,
            adlib);
    adlib->pos_regs[0] = 0xd7;
    adlib->pos_regs[1] = 0x70;

    return adlib;
}

void
adlib_close(void *priv)
{
    adlib_t *adlib = (adlib_t *) priv;

    /* stop Z80 timer */
    timer_on_auto(&adlib->z80_timer, 0.0);

    free(adlib);
}

const device_t adlib_device = {
    .name          = "AdLib",
    .internal_name = "adlib",
    .flags         = DEVICE_ISA | DEVICE_SIDECAR,
    .local         = 0,
    .init          = adlib_init,
    .close         = adlib_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};

const device_t adlib_mca_device = {
    .name          = "AdLib (MCA)",
    .internal_name = "adlib_mca",
    .flags         = DEVICE_MCA,
    .local         = 0,
    .init          = adlib_mca_init,
    .close         = adlib_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
