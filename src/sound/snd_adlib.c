/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Adlib emulation.
 *
 * Authors: Sarah Walker, <https://pcem-emulator.co.uk/>
 *          Miran Grca, <mgrca8@gmail.com>
 *          Jasmine Iwanek, <jriwanek@gmail.com>
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
#define ENABLE_ADLIB_SERIAL_IO_MIRROR
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mca.h>
#include <86box/sound.h>
#include <86box/timer.h>
#include <86box/snd_opl.h>
#include <86box/plat_unused.h>
#ifdef ENABLE_ADLIB_SERIAL_IO_MIRROR
#include <windows.h>
#include <process.h>
#include <stdbool.h>

#ifndef ADLIB_SERIAL_RING_BYTES
#  define ADLIB_SERIAL_RING_BYTES 16384
#endif

static HANDLE adlib_serial = INVALID_HANDLE_VALUE;
static uint8_t adlib_ring[ADLIB_SERIAL_RING_BYTES];
static volatile size_t adlib_head = 0, adlib_tail = 0;
static CRITICAL_SECTION adlib_ring_lock;
static HANDLE adlib_serial_thread_handle = NULL;
static volatile bool adlib_serial_thread_running = false;

// Defaults (overridable by env)
static const char *adlib_serial_port_default = "\\\\.\\COM10";
static const DWORD adlib_serial_baud_default = 921600;

// ---- ring helpers ----
static inline bool adlib_ring_empty(void) { return adlib_head == adlib_tail; }
static inline size_t adlib_ring_next(size_t i){ return (i + 1) % ADLIB_SERIAL_RING_BYTES; }

static inline bool adlib_ring_push(uint8_t b){
    size_t n = adlib_ring_next(adlib_head);
    if (n == adlib_tail) return false;
    adlib_ring[adlib_head] = b;
    adlib_head = n;
    return true;
}

static inline void adlib_serial_ringbuf_push_pair(uint8_t reg, uint8_t val){
    EnterCriticalSection(&adlib_ring_lock);
    (void)adlib_ring_push(reg);
    (void)adlib_ring_push(val);
    LeaveCriticalSection(&adlib_ring_lock);
}

static inline bool adlib_ring_pop(uint8_t *out){
    if (adlib_ring_empty()) return false;
    *out = adlib_ring[adlib_tail];
    adlib_tail = adlib_ring_next(adlib_tail);
    return true;
}

// ---- serial thread ----
static unsigned __stdcall adlib_serial_thread_proc(void *arg){
    (void)arg;
    DWORD written;
    uint8_t b;
    while (adlib_serial_thread_running){
        bool sent = false;
        for (int i=0; i<256; ++i){
            EnterCriticalSection(&adlib_ring_lock);
            bool ok = adlib_ring_pop(&b);
            LeaveCriticalSection(&adlib_ring_lock);
            if (!ok) break;
            WriteFile(adlib_serial, &b, 1, &written, NULL);
            sent = true;
        }
        if (!sent) Sleep(1);
    }
    _endthreadex(0);
    return 0;
}

static void adlib_serial_open(void){
    if (adlib_serial != INVALID_HANDLE_VALUE) return;

    InitializeCriticalSection(&adlib_ring_lock);

    const char *port_env = getenv("ADLIB_SERIAL_PORT");
    const char *port = (port_env && *port_env) ? port_env : adlib_serial_port_default;

    adlib_serial = CreateFileA(port, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (adlib_serial == INVALID_HANDLE_VALUE){
        MessageBoxA(NULL, "Failed to open ADLIB serial port", "AdLib Mirror", MB_OK | MB_ICONERROR);
        DeleteCriticalSection(&adlib_ring_lock);
        return;
    }

    const char *baud_env = getenv("ADLIB_SERIAL_BAUD");
    DWORD baud = adlib_serial_baud_default;
    if (baud_env && *baud_env){
        unsigned long b = strtoul(baud_env, NULL, 10);
        if (b > 0) baud = (DWORD)b;
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(adlib_serial, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fRtsControl  = RTS_CONTROL_DISABLE;
    SetCommState(adlib_serial, &dcb);

    COMMTIMEOUTS t = {0};
    t.WriteTotalTimeoutConstant = 10;
    t.WriteTotalTimeoutMultiplier = 1;
    SetCommTimeouts(adlib_serial, &t);

    adlib_serial_thread_running = true;
    adlib_serial_thread_handle =
        (HANDLE)_beginthreadex(NULL, 0, adlib_serial_thread_proc, NULL, 0, NULL);
}

static void adlib_serial_close(void){
    adlib_serial_thread_running = false;
    if (adlib_serial_thread_handle){
        WaitForSingleObject(adlib_serial_thread_handle, 1000);
        CloseHandle(adlib_serial_thread_handle);
        adlib_serial_thread_handle = NULL;
    }
    if (adlib_serial != INVALID_HANDLE_VALUE){
        CloseHandle(adlib_serial);
        adlib_serial = INVALID_HANDLE_VALUE;
    }
    DeleteCriticalSection(&adlib_ring_lock);
}
#else
#  define adlib_serial_open()  ((void)0)
#  define adlib_serial_close() ((void)0)
static inline void adlib_serial_ringbuf_push_pair(uint8_t r, uint8_t v){ (void)r; (void)v; }
#endif

// Mirror SID writes (0x0280 base) but keep "adlib" naming for consistency.
static const uint16_t adlib_sid_base = 0x0280;   // SSI-2001 typical base

static void adlib_serial_write(uint16_t port, uint8_t val, void *priv)
{
    // Compute SID register from port
    uint16_t off = (uint16_t)(port - adlib_sid_base);
    uint8_t reg = (uint8_t)(off & 0x1F);  // 0x00..0x1F window

    if (reg < 0x19) {
        adlib_serial_ringbuf_push_pair(reg, val);   // send (reg,val) to COM
    }

}

typedef struct adlib_s {
    fm_drv_t opl;

    uint8_t pos_regs[8];
} adlib_t;

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

    return adlib->pos_regs[port & 7];
}

void
adlib_mca_write(int port, uint8_t val, void *priv)
{
    adlib_t *adlib = (adlib_t *) priv;

    if (port < 0x102)
        return;

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

void *adlib_init(UNUSED(const device_t *info))
{
    adlib_t *adlib = calloc(1, sizeof(adlib_t));

    fm_driver_get(FM_YM3812, &adlib->opl);

    // Save original, then wrap with our serial-mirroring write
    adlib->opl.write   = adlib_serial_write;

    // If you intend to watch the SID address window (0x0280..),
    // keep this registration as you have it:
    io_sethandler(0x0280, 0x0020,
                  adlib->opl.read, NULL, NULL,
                  adlib->opl.write, NULL, NULL,
                  adlib->opl.priv);

    adlib_serial_open();              // start COM + thread
    music_add_handler(adlib_get_buffer, adlib);
    return adlib;
}

void adlib_close(void *priv)
{
    adlib_serial_close();             // stop thread + close COM + delete CS
    adlib_t *adlib = (adlib_t *) priv;
    free(adlib);
}


void *
adlib_mca_init(const device_t *info)
{
    adlib_t *adlib = adlib_init(info);

    io_removehandler(0x0378, 0x0002,
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
