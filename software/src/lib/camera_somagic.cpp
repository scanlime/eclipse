/*
 * USB Driver for Somagic EasyCAP DC60 and Somagic EasyCAP002
 * USB ID 1c88:0007
 *
 * Implements the abstract camera interface in camera.h.
 * This runs on a separate thread, and handles USB hotplug.
 *
 * Modifications by Micah Elizabeth Scott, 2014.
 *
 * Originally part of the somagic_easycap project:
 *   http://code.google.com/p/easycap-somagic-linux/
 *
 * *****************************************************************************
 *
 * Copyright 2011, 2012 Tony Brown, Jeffry Johnston
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "camera.h"
#include "tinythread.h"

#include <algorithm>
#include <strings.h>
#include <ctype.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libusb-1.0/libusb.h>

using namespace Camera;

#define SOMAGIC_FIRMWARE_PATH       "data/somagic_firmware.bin"
#define VENDOR                      0x1c88
#define PRODUCT_WITHOUT_FIRMWARE    0x0007

// Different firmware versions yield different vendor IDs
#define PRODUCT_COUNT 4
static const int PRODUCT[PRODUCT_COUNT] = {
    0x003c,
    0x003d,
    0x003e,
    0x003f
};

enum tv_standards {
    NTSC,         /* 525/60 */
    PAL_60,       /* 525/60 */
    NTSC_60,      /* 525/60 */
    PAL_M,        /* 525/60 */
    PAL,          /* 625/50 */
    NTSC_50,      /* 525/50 (different) */
    PAL_COMBO_N,  /* 625/50 */
    NTSC_N,       /* 625/50 */
    SECAM,        /* 625/50 */
};

/* Input types */
#define CVBS   0      /* DC60: "CVBS", 002: "2" */
#define SVIDEO 7      /* DC60: "S-VIDEO" */

/* CVBS inputs */
#define VIDEO1 2
#define VIDEO2 3
#define VIDEO3 0
#define VIDEO4 1

/* Options */

/* Television standard (see tv_standards) */
static int tv_standard = NTSC;

/* Input type select (see Input types) */
static int input_type = CVBS;

/* CVBS input select */
static int cvbs_input = VIDEO3;

/* Luminance mode (CVBS only): 0 = 4.1 MHz, 1 = 3.8 MHz, 2 = 2.6 MHz, 3 = 2.9 MHz */
static int luminance_mode = 0;

/* Luminance prefilter: 0 = bypassed, 1 = active */
static int luminance_prefilter = 1;

/* Hue phase in degrees: -128 to 127 (-180 to 178.59375), increments of 1.40625 degrees */
static uint8_t hue = 0;

/* Chrominance saturation: -128 to 127 (1.984375 to -2.000000), increments of 0.015625 */
static uint8_t saturation = 64;

/* Luminance contrast: -128 to 127 (1.984375 to -2.000000), increments of 0.015625 */
static uint8_t contrast = 90;

/* Luminance brightness: 0 to 255 */
static uint8_t brightness = 128;

/* Luminance aperture factor: 0 = 0, 1 = 0.25, 2 = 0.5, 3 = 1.0 */
static int luminance_aperture = 1;

/* Control the number of concurrent ISO transfers we have running. <= 30 */
static const int num_iso_transfers = 30;

enum sync_state {
    HSYNC = 0,
    SYNCZ1,
    SYNCZ2,
    SYNCAV,
    VBLANK,
    VACTIVE,
    REMAINDER
};

struct alg1_video_state_t {
    int line_remaining;
    int active_line_count;
    int vblank_found;
    int field;
    int frame_count;

    enum sync_state state;
};

static struct alg1_video_state_t alg1_vs;

static tthread::thread *cameraThread = 0;
static videoCallback_t videoCallback;
static void *videoCallbackContext;

static struct libusb_device_handle *devh;
static int pending_requests;
static unsigned resubmit_bitmask;
static int lines_per_field;


static void release_usb_device(int ret)
{
    fprintf(stderr, "Emergency exit\n");
    ret = libusb_release_interface(devh, 0);
    if (!ret) {
        perror("Failed to release interface");
    }
    libusb_close(devh);
    libusb_exit(NULL);
    exit(1);
}

static void print_bytes(unsigned char *bytes, int len)
{
    int i;
    if (len > 0) {
        for (i = 0; i < len; i++) {
            fprintf(stderr, "%02x ", (int)bytes[i]);
        }
        fprintf(stderr, "\"");
        for (i = 0; i < len; i++) {
            fprintf(stderr, "%c", isprint(bytes[i]) ? bytes[i] : '.');
        }
        fprintf(stderr, "\"");
    }
}

static struct libusb_device *find_device(int vendor, int product)
{
    struct libusb_device **list;
    struct libusb_device *dev = NULL;
    struct libusb_device_descriptor descriptor;
    struct libusb_device *item;
    int i;
    ssize_t count;
    count = libusb_get_device_list(NULL, &list);
    for (i = 0; i < count; i++) {
        item = list[i];
        libusb_get_device_descriptor(item, &descriptor);
        if (descriptor.idVendor == vendor && descriptor.idProduct == product) {
            dev = item;
        } else {
            libusb_unref_device(item);
        }
    }
    libusb_free_device_list(list, 0);
    return dev;
}

static void install_firmware(struct libusb_device *dev)
{
    int ret;
    unsigned char buf[65535];
    char *firmware = 0;
    FILE *infile = 0;
    int i;
    const char *firmware_path = SOMAGIC_FIRMWARE_PATH;
    int firmware_length;

    fprintf(stderr, "camera: Installing firmware...\n");

    /* Read firmware file */
    infile = fopen(firmware_path, "r");
    if (infile == NULL) {
        fprintf(stderr, "camera: Failed to open firmware file '%s': %s\n", firmware_path, strerror(errno));
        goto done;
    }

    fseek(infile, 0, SEEK_END);
    firmware_length = ftell(infile);
    firmware = (char*) malloc(firmware_length);

    if (firmware == NULL) {
        fprintf(stderr, "camera: Failed to allocate '%i' bytes of memory for firmware file '%s': %s\n", firmware_length, firmware_path, strerror(errno));
        goto done;
    }

    fseek(infile, 0, SEEK_SET);
    if (fread(firmware, 1, firmware_length, infile) != (unsigned)firmware_length) {
        fprintf(stderr, "camera: Failed to read firmware file '%s': %s\n", firmware_path, strerror(errno));
        goto done;
    }

    ret = libusb_open(dev, &devh);
    if (!devh) {
        perror("Failed to open USB device");
        goto done;
    }

    signal(SIGTERM, release_usb_device);
    ret = libusb_claim_interface(devh, 0);
    if (ret != 0) {
        perror("Failed to claim device interface");
        goto done;
    }

    ret = libusb_set_interface_alt_setting(devh, 0, 0);
    if (ret != 0) {
        perror("Failed to activate alternate setting for interface");
        goto done;
    }

    ret = libusb_get_descriptor(devh, 0x0000001, 0x0000000, buf, 0x0000012);
    ret = libusb_get_descriptor(devh, 0x0000002, 0x0000000, buf, 0x0000009);
    ret = libusb_get_descriptor(devh, 0x0000002, 0x0000000, buf, 0x0000022);
    ret = libusb_release_interface(devh, 0);
    if (ret != 0) {
        perror("Failed to release interface (before set_configuration)");
        goto done;
    }
    ret = libusb_set_configuration(devh, 0x0000001);
    if (ret != 0) {
        perror("Failed to set active device configuration");
        goto done;
    }
    ret = libusb_claim_interface(devh, 0);
    if (ret != 0) {
        perror("Failed to claim device interface (after set_configuration)");
        goto done;
    }
    ret = libusb_set_interface_alt_setting(devh, 0, 0);
    if (ret != 0) {
        perror("Failed to set active alternate setting for interface (after set_configuration)");
        goto done;
    }

    ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE + LIBUSB_ENDPOINT_IN, 0x0000001, 0x0000001, 0x0000000, buf, 2, 1000);

    for (i = 0; i < firmware_length; i += 62) {
        memcpy(buf, "\x05\xff", 2);
        memcpy(buf + 2, firmware + i, 62);
        ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x0000001, 0x0000005, 0x0000000, buf, 64, 1000);

        if (ret != 64) {
            fprintf(stderr, "control msg returned %d, bytes: ", ret);
            print_bytes(buf, ret);
            fprintf(stderr, "\n");
            goto done;
        }
    }

    memcpy(buf, "\x07\x00", 2);
    ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x0000001, 0x0000007, 0x0000000, buf, 2, 1000);

    // Let device reset before we try detecting it
    usleep(1000 * 250);

done:
    if (firmware) {
        free(firmware);
    }
    if (devh) {
        libusb_close(devh);
        devh = 0;
    }
    if (infile) {
        fclose(infile);
    }
}

static void alg1_process(struct alg1_video_state_t *vs, unsigned char *buffer, int length)
{
    unsigned char *next = buffer;
    unsigned char *end = buffer + length;
    int bs = 0; /* bad (lost) sync: 0=no, 1=yes */
    int hs = 0;
    int lines_per_field = (tv_standard == PAL ? 288 : 240);
    unsigned char nc;
    int skip;
    int wrote;
    do {
        nc = *next;
        /*
         * Timing reference code (TRC):
         *     [ff 00 00 SAV] [ff 00 00 EAV]
         * Where SAV is 80 or c7, and EAV is 9d or da.
         * A line of video will look like (1448 bytes total):
         *     [ff 00 00 EAV] [ff 00 00 SAV] [1440 bytes of UYVY video] (repeat on next line)
         */
        switch (vs->state) {
            case HSYNC:
                hs++;
                if (nc == (unsigned char)0xff) {
                    vs->state = SYNCZ1;
                    bs = 0;
                } else if (bs != 1) {
                    /*
                     * The 1st byte in the TRC must be 0xff. It
                     * wasn't, so sync was either lost or has not
                     * yet been regained. Sync is regained by
                     * ignoring bytes until the next 0xff.
                     */
                    bs = 1;
                }
                next++;
                break;
            case SYNCZ1:
                if (nc == (unsigned char)0x00) {
                    vs->state = SYNCZ2;
                } else {
                    /*
                     * The 2nd byte in the TRC must be 0x00. It
                     * wasn't, so sync was lost.
                     */
                    vs->state = HSYNC;
                }
                next++;
                break;
            case SYNCZ2:
                if (nc == (unsigned char)0x00) {
                    vs->state = SYNCAV;
                } else {
                    /*
                     * The 3rd byte in the TRC must be 0x00. It
                     * wasn't, so sync was lost.
                     */
                    vs->state = HSYNC;
                }
                next++;
                break;
            case SYNCAV:
                /*
                 * Found 0xff 0x00 0x00, now expecting SAV or EAV. Might
                 * also be the SDID (sliced data ID), 0x00.
                 */
                if (nc == (unsigned char)0x00) {
                    /*
                     * SDID detected, so we still haven't found the
                     * active YUV data.
                     */
                    vs->state = HSYNC;
                    next++;
                    break;
                }

                /*
                 * H = Bit 4 (mask 0x10).
                 * 0: in SAV, 1: in EAV.
                 */
                if (nc & (unsigned char)0x10) {
                    /* EAV (end of active data) */
                    vs->state = HSYNC;
                } else {
                    /* SAV (start of active data) */
                    /*
                        * F (field bit) = Bit 6 (mask 0x40).
                        * 0: first field, 1: 2nd field.
                        */
                    vs->field = (nc & (unsigned char)0x40) ? 1 : 0;
                    /*
                        * V (vertical blanking bit) = Bit 5 (mask 0x20).
                        * 1: in VBI, 0: in active video.
                        */
                    if (nc & (unsigned char)0x20) {
                        /* VBI (vertical blank) */
                        vs->state = VBLANK;
                        vs->vblank_found++;
                        if (vs->active_line_count > (lines_per_field - 8)) {
                            vs->vblank_found = 0;
                            vs->frame_count = std::min<int>(INT_MAX - 1, vs->frame_count) + 1;
                        }
                        vs->active_line_count = 0;
                    } else {
                        /* Line is active video */
                        vs->state = VACTIVE;
                    }
                    vs->line_remaining = 720 * 2;
                }
                next++;
                break;
            case VBLANK:
            case VACTIVE:
            case REMAINDER:
                if (vs->state == VBLANK || vs->vblank_found < 20) {
                    skip = std::min<int>(vs->line_remaining, (end - next));
                    vs->line_remaining -= skip;
                    next += skip ;
                } else {
                    wrote = std::min<unsigned>(end - next, vs->line_remaining);

                    if (vs->frame_count > 2) {
                        // We're definitely synchronized; process this video data

                        VideoChunk chunk;
                        chunk.data = next;
                        chunk.byteCount = wrote;
                        chunk.byteOffset = kBytesPerLine - vs->line_remaining;
                        chunk.line = vs->active_line_count;
                        chunk.field = vs->field;

                        if (chunk.line < kLinesPerField) {
                            videoCallback(chunk, videoCallbackContext);
                        }
                    }

                    vs->line_remaining -= wrote;
                    next += wrote;
                    if (vs->line_remaining <= 0) {
                        vs->active_line_count++;
                    }
                }
                if (vs->line_remaining <= 0) {
                    vs->state = HSYNC;
                } else {
                    vs->state = REMAINDER;
                    /* no more data in this buffer. exit loop */
                    next = end;
                }
                break;
        } /* end switch */
    } while (next < end);
}

static void camera_usb_callback(struct libusb_transfer *tfr)
{
    unsigned tfrIndex = (uintptr_t) tfr->user_data;
    int num = tfr->num_iso_packets;
    int i;
    unsigned char *data;
    int length;
    int pos;

    pending_requests--;

    for (i = 0; i < num; i++) {
        data = libusb_get_iso_packet_buffer_simple(tfr, i);
        length = tfr->iso_packet_desc[i].actual_length;
        pos = 0;

        while (pos < length) {
            /*
             * Within each packet of the transfer, the video data is divided
             * into blocks of 0x400 bytes beginning with [0xaa 0xaa 0x00 0x00].
             * Check for this signature and process each block of data individually.
             */
            if (data[pos] == 0xaa && data[pos + 1] == 0xaa && data[pos + 2] == 0x00 && data[pos + 3] == 0x00) {
                /* Process received video data, excluding the 4 marker bytes */
                alg1_process(&alg1_vs, data + 4 + pos, 0x400 - 4);
            } else {
                fprintf(stderr, "Unexpected block, expected [aa aa 00 00] found [%02x %02x %02x %02x]\n", data[pos], data[pos + 1], data[pos + 2], data[pos + 3]);
            }
            pos += 0x400;
        }
    }

    // Remember to resubmit this one later. We can't do it safely from inside
    // the callback due to a libusb quirk. If the device was removed, this
    // will cause a use-after-free crash.

    resubmit_bitmask |= 1 << tfrIndex;
}

static int somagic_write_reg(uint16_t reg, uint8_t val)
{
    int ret;
    uint8_t buf[8];

    memcpy(buf, "\x0b\x00\x00\x82\x01\x00\x3a\x00", 8);
    buf[5] = reg >> 8;
    buf[6] = reg & 0xff;
    buf[7] = val;

    ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x0000001, 0x000000b, 0x0000000, buf, 8, 1000);
    if (ret != 8) {
        fprintf(stderr, "write reg control msg returned %d, bytes: ", ret);
        print_bytes(buf, ret);
        fprintf(stderr, "\n");
    }

    return ret;
}

static int somagic_write_i2c(uint8_t dev_addr, uint8_t reg, uint8_t val)
{
    int ret;
    uint8_t buf[8];

    memcpy(buf, "\x0b\x4a\xc0\x01\x01\x01\x08\xf4", 8);

    buf[1] = dev_addr;
    buf[5] = reg;
    buf[6] = val;

    ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x0000001, 0x000000b, 0x0000000, buf, 8, 1000);
    if (ret != 8) {
        fprintf(stderr, "write_i2c returned %d, bytes: ", ret);
        print_bytes(buf, ret);
        fprintf(stderr, "\n");
    }

    return ret;
}

static int somagic_capture()
{
    int ret;
    int i = 0;

    // Buffers and transfer pointers for isochronous data.
    // Static so we don't have to worry about use-after-free during shutdown.
    static struct libusb_transfer* tfr[num_iso_transfers];
    static unsigned char isobuf[num_iso_transfers][64 * 3072];

    for (i = 0; i < num_iso_transfers; i++) {
        // Allocate only on first use; keep them around across hotplug events
        if (tfr[i] == NULL) {
            tfr[i] = libusb_alloc_transfer(64);
            if (tfr[i] == NULL) {
                fprintf(stderr, "camera: Failed to allocate USB transfer #%d: %s\n", i, strerror(errno));
                return 1;
            }
        }

        // Reinitialize transfer struct. Note that userdata is the transfer index.
        libusb_fill_iso_transfer(tfr[i], devh, 0x00000082,
            isobuf[i], 64 * 3072, 64,
            camera_usb_callback,
            (void*)(uintptr_t)i, 2000);

        libusb_set_iso_packet_lengths(tfr[i], 3072);
    }

    // Start out with no transfers
    pending_requests = 0;
    resubmit_bitmask = (1 << num_iso_transfers) - 1;
    memset(&alg1_vs, 0, sizeof alg1_vs);

    somagic_write_reg(0x1800, 0x0d);

    fprintf(stderr, "camera: Video stream started\n");

    // Keep running until we have no transfers and nothing to resubmit
    while (pending_requests > 0 || resubmit_bitmask != 0) {

        if (pending_requests == 0) {
            fprintf(stderr, "camera: buffer underrun (video processing taking too long!)\n");
        
            // Discard synchronization state
            memset(&alg1_vs, 0, sizeof alg1_vs);
        }

        while (resubmit_bitmask) {
            int i = ffs(resubmit_bitmask) - 1;
            resubmit_bitmask &= ~(1 << i);

            ret = libusb_submit_transfer(tfr[i]);
            if (ret) {
                fprintf(stderr, "camera: Failed to submit request #%d for transfer: %s\n", i, strerror(errno));
                return 1;
            }

            pending_requests++;
        }

        libusb_handle_events(NULL);
    }

    return 0;
}

static int somagic_init()
{
    int ret;
    uint8_t work;

    /* buffer for control messages */
    unsigned char buf[65535];

    signal(SIGTERM, release_usb_device);
    ret = libusb_claim_interface(devh, 0);
    if (ret) {
        perror("Failed to claim device interface");
        if (ret == LIBUSB_ERROR_BUSY) {
            fprintf(stderr, "Another program is using the device\n");
        }
        return 1;
    }

    ret = libusb_set_interface_alt_setting(devh, 0, 0);
    if (ret) {
        perror("Failed to set active alternate setting for interface");
        return 1;
    }

    ret = libusb_get_descriptor(devh, 0x0000001, 0x0000000, buf, 18);
    if (ret != 18) {
        fprintf(stderr, "1 get descriptor returned %d, bytes: ", ret);
        print_bytes(buf, ret);
        fprintf(stderr, "\n");
    }
    ret = libusb_get_descriptor(devh, 0x0000002, 0x0000000, buf, 9);
    if (ret != 9) {
        fprintf(stderr, "2 get descriptor returned %d, bytes: ", ret);
        print_bytes(buf, ret);
        fprintf(stderr, "\n");
    }
    ret = libusb_get_descriptor(devh, 0x0000002, 0x0000000, buf, 66);

    ret = libusb_release_interface(devh, 0);
    if (ret) {
        perror("Failed to release interface (before set_configuration)");
        return 1;
    }
    ret = libusb_set_configuration(devh, 0x0000001);
    if (ret) {
        perror("Failed to set active device configuration");
        return 1;
    }
    ret = libusb_claim_interface(devh, 0);
    if (ret) {
        perror("Failed to claim device interface (after set_configuration)");
        return 1;
    }
    ret = libusb_set_interface_alt_setting(devh, 0, 0);
    if (ret) {
        perror("Failed to set active alternate setting for interface (after set_configuration)");
        return 1;
    }
    ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE + LIBUSB_ENDPOINT_IN, 0x0000001, 0x0000001, 0x0000000, buf, 2, 1000);
    if (ret != 2) {
        fprintf(stderr, "5 control msg returned %d, bytes: ", ret);
        print_bytes(buf, ret);
        fprintf(stderr, "\n");
    }

    /*
     * AVR Documentation @ http://www.avr-asm-tutorial.net/avr_en/beginner/PDETAIL.html#IOPORTS
     *
     * Reg 0x3a should be DDRA.
     * (DDRA = PortA Data Direction Register)
     * By setting this to 0x80, we set PIN7 to output.
     *
     * I assume that this PIN is connected to the RESET pin of the
     * SAA7XXX & CS5340.
     *
     * If we leave this PIN in HIGH, or don't set it to OUTPUT
     * we can not receive Stereo Audio from the CS5340.
     *
     * Reg 0x3b should be PORTA.
     * (PortA = PortA Data Register)
     * By setting this to 0x00, we pull Pin7 LOW
     */
    somagic_write_reg(0x3a, 0x80);
    somagic_write_reg(0x3b, 0x00);

    /*
     * Reg 0x34 should be DDRC
     * Reg 0x35 should be PORTC.
     *
     * This PORT seems to only be used in the Model002!
     */
    somagic_write_reg(0x34, 0x01);
    somagic_write_reg(0x35, 0x00);
    somagic_write_reg(0x34, 0x11);
    somagic_write_reg(0x35, 0x11);

    /* SAAxxx: toggle RESET (PIN7) */
    somagic_write_reg(0x3b, 0x80);
    somagic_write_reg(0x3b, 0x00);

    /* Subaddress 0x01, Horizontal Increment delay */
    /* Recommended position */
    somagic_write_i2c(0x4a, 0x01, 0x08);

    /* Subaddress 0x02, Analog input control 1 */
    /* Analog function select FUSE = Amplifier plus anti-alias filter bypassed */
    /* Update hysteresis for 9-bit gain = Off */
    if (input_type == CVBS) {
        work = 0xc0 | cvbs_input;
    } else {
        work = 0xc0 | input_type;
    }
    somagic_write_i2c(0x4a, 0x02, work);

    /* Subaddress 0x03, Analog input control 2 */
    if (input_type != SVIDEO) {
        /* Static gain control channel 1 (GAI18), sign bit of gain control = 1 */
        /* Static gain control channel 2 (GAI28), sign bit of gain control = 1 */
        /* Gain control fix (GAFIX) = Automatic gain controlled by MODE3 to MODE0 */
        /* Automatic gain control integration (HOLDG) = AGC active */
        /* White peak off (WPOFF) = White peak off */
        /* AGC hold during vertical blanking period (VBSL) = Long vertical blanking (AGC disabled from start of pre-equalization pulses until start of active video (line 22 for 60 Hz, line 24 for 50 Hz) */
        /* Normal clamping if decoder is in unlocked state */
        somagic_write_i2c(0x4a, 0x03, 0x33);
    } else {
        /* Static gain control channel 1 (GAI18), sign bit of gain control = 1 */
        /* Static gain control channel 2 (GAI28), sign bit of gain control = 0 */
        /* Gain control fix (GAFIX) = Automatic gain controlled by MODE3 to MODE0 */
        /* Automatic gain control integration (HOLDG) = AGC active */
        /* White peak off (WPOFF) = White peak off */
        /* AGC hold during vertical blanking period (VBSL) = Long vertical blanking (AGC disabled from start of pre-equalization pulses until start of active video (line 22 for 60 Hz, line 24 for 50 Hz) */
        /* Normal clamping if decoder is in unlocked state */
        somagic_write_i2c(0x4a, 0x03, 0x31);
    }

    /* Subaddress 0x04, Gain control analog/Analog input control 3 (AICO3); static gain control channel 1 GAI1 */
    /* Gain (dB) = -3 (Note: Dependent on subaddress 0x03 GAI18 value) */
    somagic_write_i2c(0x4a, 0x04, 0x00);

    /* Subaddress 0x05, Gain control analog/Analog input control 4 (AICO4); static gain control channel 2 GAI2 */
    /* Gain (dB) = -3 (Note: Dependent on subaddress 0x03 GAI28 value) */
    somagic_write_i2c(0x4a, 0x05, 0x00);

    /* Subaddress 0x06, Horizontal sync start/begin */
    /* Delay time (step size = 8/LLC) = Recommended value for raw data type */
    somagic_write_i2c(0x4a, 0x06, 0xe9);

    /* Subaddress 0x07, Horizontal sync stop */
    /* Delay time (step size = 8/LLC) = Recommended value for raw data type */
    somagic_write_i2c(0x4a, 0x07, 0x0d);

    /* Subaddress 0x08, Sync control */
    /* Automatic field detection (AUFD) = Automatic field detection */
    /* Field selection (FSEL) = 50 Hz, 625 lines (Note: Ignored due to automatic field detection) */
    /* Forced ODD/EVEN toggle FOET = ODD/EVEN signal toggles only with interlaced source */
    /* Horizontal time constant selection = Fast locking mode (recommended setting) */
    /* Horizontal PLL (HPLL) = PLL closed */
    /* Vertical noise reduction (VNOI) = Normal mode (recommended setting) */
    somagic_write_i2c(0x4a, 0x08, 0x98);

    /* Subaddress 0x09, Luminance control */
    /* Update time interval for analog AGC value (UPTCV) = Horizontal update (once per line) */
    /* Vertical blanking luminance bypass (VBLB) = Active luminance processing */
    /* Chrominance trap bypass (BYPS) = Chrominance trap active; default for CVBS mode */
    work = ((luminance_prefilter & 0x01) << 6) | ((luminance_mode & 0x03) << 4) | (luminance_aperture & 0x03);
    if (input_type == SVIDEO) {
        /* Chrominance trap bypass (BYPS) = Chrominance trap bypassed; default for S-video mode */
        work |= 0x80;
    }
    somagic_write_i2c(0x4a, 0x09, work);

    /* Subaddress 0x0a, Luminance brightness control */
    /* Offset = 128 (ITU level) */
    somagic_write_i2c(0x4a, 0x0a, brightness);

    /* Subaddress 0x0b, Luminance contrast control */
    /* Gain = 1.0 */
    somagic_write_i2c(0x4a, 0x0b, contrast);

    /* Subaddress 0x0c, Chrominance saturation control */
    somagic_write_i2c(0x4a, 0x0c, saturation);

    /* Subaddress 0x0d, Chrominance hue control */
    somagic_write_i2c(0x4a, 0x0d, hue);

    /* Subaddress 0x0e, Chrominance control */
    /* Chrominance bandwidth (CHBW0 and CHBW1) = Nominal bandwidth (800 kHz) */
    /* Fast color time constant (FCTC) = Nominal time constant */
    /* Disable chrominance comb filter (DCCF) = Chrominance comb filter on (during lines determined by VREF = 1) */
    /* Clear DTO (CDTO) = Disabled */
    switch (tv_standard) {
    case PAL:
    case NTSC:
        work = 0x01;
        break;
    case NTSC_50:
    case PAL_60:
        work = 0x11;
        break;
    case PAL_COMBO_N:
    case NTSC_60:
        work = 0x21;
        break;
    case NTSC_N:
    case PAL_M:
        work = 0x31;
        break;
    case SECAM:
        work = 0x50;
        break;
    }
    somagic_write_i2c(0x4a, 0x0e, work);

    /* Subaddress 0x0f, Chrominance gain control */
    /* Chrominance gain value = ??? (Note: only meaningful if ACGF is off) */
    /* Automatic chrominance gain control ACGC = On */
    somagic_write_i2c(0x4a, 0x0f, 0x2a);

    /* Subaddress 0x10, Format/delay control */
    /* Output format selection (OFTS0 and OFTS1), V-flag generation in SAV/EAV-codes = V-flag in SAV/EAV is generated by VREF */
    /* Fine position of HS (HDEL0 and HDEL1) (steps in 2/LLC) = 0 */
    /* VREF pulse position and length (VRLN) = see Table 46 in SAA7113H documentation */
    /* Luminance delay compensation (steps in 2/LLC) = 0 */
    somagic_write_i2c(0x4a, 0x10, 0x40);

    /* Subaddress 0x11, Output control 1 */
    /* General purpose switch [available on pin RTS1, if control bits RTSE13 to RTSE10 (subaddress 0x12) is set to 0010] = LOW */
    /* CM99 compatibility to SAA7199 (CM99) = Default value */
    /* General purpose switch [available on pin RTS0, if control bits RTSE03 to RTSE00 (subaddress 0x12) is set to 0010] = LOW */
    /* Selection of horizontal lock indicator for RTS0 and RTS1 outputs = Standard horizontal lock indicator (low-passed) */
    /* Output enable YUV data (OEYC) = Output VPO-bus active or controlled by RTS1 */
    /* Output enable real-time (OERT) = RTS0, RTCO active, RTS1 active, if RTSE13 to RTSE10 = 0000 */
    /* YUV decoder bypassed (VIPB) = Processed data to VPO output */
    /* Color on (COLO) = Automatic color killer */
    somagic_write_i2c(0x4a, 0x11, 0x0c);

    /* Subaddress 0x12, RTS0 output control/Output control 2 */
    /* RTS1 output control = 3-state, pin RTS1 is used as DOT input */
    /* RTS0 output control = VIPB (subaddress 0x11, bit 1) = 0: reserved */
    somagic_write_i2c(0x4a, 0x12, 0x01);

    /* Subaddress 0x13, Output control 3 */
    if (input_type != SVIDEO) {
        /* Analog-to-digital converter output bits on VPO7 to VPO0 in bypass mode (VIPB = 1, used for test purposes) (ADLSB) = AD7 to AD0 (LSBs) on VPO7 to VPO0 */
        /* Selection bit for status byte functionality (OLDSB) = Default status information */
        /* Field ID polarity if selected on RTS1 or RTS0 outputs if RTSE1 and RTSE0 (subaddress 0x12) are set to 1111 = Default */
        /* Analog test select (AOSL) = AOUT connected to internal test point 1 */
        somagic_write_i2c(0x4a, 0x13, 0x80);
    } else {
        /* Analog-to-digital converter output bits on VPO7 to VPO0 in bypass mode (VIPB = 1, used for test purposes) (ADLSB) = AD8 to AD1 (MSBs) on VPO7 to VPO0 */
        /* Selection bit for status byte functionality (OLDSB) = Default status information */
        /* Field ID polarity if selected on RTS1 or RTS0 outputs if RTSE1 and RTSE0 (subaddress 0x12) are set to 1111 = Default */
        /* Analog test select (AOSL) = AOUT connected to internal test point 1 */
        somagic_write_i2c(0x4a, 0x13, 0x00);
    }

    /* Subaddress 0x15, Start of VGATE pulse (01-transition) and polarity change of FID pulse/V_GATE1_START */
    /* Note: Dependency on subaddress 0x17 value */
    /* Frame line counting = If 50Hz: 1st = 2, 2nd = 315. If 60Hz: 1st = 5, 2nd = 268. */
    somagic_write_i2c(0x4a, 0x15, 0x00);

    /* Subaddress 0x16, Stop of VGATE pulse (10-transition)/V_GATE1_STOP */
    /* Note: Dependency on subaddress 0x17 value */
    /* Frame line counting = If 50Hz: 1st = 2, 2nd = 315. If 60Hz: 1st = 5, 2nd = 268. */
    somagic_write_i2c(0x4a, 0x16, 0x00);

    /* Subaddress 0x17, VGATE MSBs/V_GATE1_MSB */
    /* VSTA8, MSB VGATE start = 0 */
    /* VSTO8, MSB VGATE stop = 0 */
    somagic_write_i2c(0x4a, 0x17, 0x00);

    /* Subaddress 0x40, AC1 */
    if (tv_standard == NTSC || tv_standard == PAL_60 || tv_standard == NTSC_60 || tv_standard == PAL_M) {
        /* Data slicer clock selection, Amplitude searching = 13.5 MHz (default) */
        /* Amplitude searching = Amplitude searching active (default) */
        /* Framing code error = One framing code error allowed */
        /* Hamming check = Hamming check for 2 bytes after framing code, dependent on data type (default) */
        /* Field size select = 60 Hz field rate */
        somagic_write_i2c(0x4a, 0x40, 0x82);
    } else {
        /* Data slicer clock selection, Amplitude searching = 13.5 MHz (default) */
        /* Amplitude searching = Amplitude searching active (default) */
        /* Framing code error = One framing code error allowed */
        /* Hamming check = Hamming check for 2 bytes after framing code, dependent on data type (default) */
        /* Field size select = 50 Hz field rate */
        somagic_write_i2c(0x4a, 0x40, 0x02);
    }

    if (input_type != SVIDEO) {
        /* LCR register 2 to 24 = Intercast, oversampled CVBS data */
        somagic_write_i2c(0x4a, 0x41, 0x77);
        somagic_write_i2c(0x4a, 0x42, 0x77);
        somagic_write_i2c(0x4a, 0x43, 0x77);
        somagic_write_i2c(0x4a, 0x44, 0x77);
        somagic_write_i2c(0x4a, 0x45, 0x77);
        somagic_write_i2c(0x4a, 0x46, 0x77);
        somagic_write_i2c(0x4a, 0x47, 0x77);
        somagic_write_i2c(0x4a, 0x48, 0x77);
        somagic_write_i2c(0x4a, 0x49, 0x77);
        somagic_write_i2c(0x4a, 0x4a, 0x77);
        somagic_write_i2c(0x4a, 0x4b, 0x77);
        somagic_write_i2c(0x4a, 0x4c, 0x77);
        somagic_write_i2c(0x4a, 0x4d, 0x77);
        somagic_write_i2c(0x4a, 0x4e, 0x77);
        somagic_write_i2c(0x4a, 0x4f, 0x77);
        somagic_write_i2c(0x4a, 0x50, 0x77);
        somagic_write_i2c(0x4a, 0x51, 0x77);
        somagic_write_i2c(0x4a, 0x52, 0x77);
        somagic_write_i2c(0x4a, 0x53, 0x77);
        somagic_write_i2c(0x4a, 0x54, 0x77);
        /* LCR register 2 to 24 = Active video, video component signal, active video region (default) */
        somagic_write_i2c(0x4a, 0x55, 0xff);
    }

    /* Subaddress 0x58, Framing code for programmable data types/FC */
    /* Slicer set, Programmable framing code = ??? */
    somagic_write_i2c(0x4a, 0x58, 0x00);

    /* Subaddress 0x59, Horizontal offset/HOFF */
    /* Slicer set, Horizontal offset = Recommended value */
    somagic_write_i2c(0x4a, 0x59, 0x54);

    /* Subaddress 0x5a: Vertical offset/VOFF */
    if (tv_standard == PAL || tv_standard == PAL_COMBO_N || tv_standard == NTSC_N || tv_standard == SECAM) {
        /* Slicer set, Vertical offset = Value for 625 lines input */
        somagic_write_i2c(0x4a, 0x5a, 0x07);
        lines_per_field = 288;
    } else {
        /* Slicer set, Vertical offset = Value for 525 lines input */
        somagic_write_i2c(0x4a, 0x5a, 0x0a);
        lines_per_field = 240;
    }

    /* Subaddress 0x5b, Field offset, MSBs for vertical and horizontal offsets/HVOFF */
    /* Slicer set, Field offset = Invert field indicator (even/odd; default) */
    somagic_write_i2c(0x4a, 0x5b, 0x83);

    /* Subaddress 0x5e, SDID codes */
    /* Slicer set, SDID codes = SDID5 to SDID0 = 0x00 (default) */
    somagic_write_i2c(0x4a, 0x5e, 0x00);

    somagic_write_reg(0x1740, 0x40);

    somagic_write_reg(0x1740, 0x00);
    usleep(250 * 1000);
    somagic_write_reg(0x1740, 0x00);

    memcpy(buf, "\x01\x05", 2);
    ret = libusb_control_transfer(devh, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_RECIPIENT_DEVICE, 0x0000001, 0x0000001, 0x0000000, buf, 2, 1000);
    if (ret != 2) {
        fprintf(stderr, "190 control msg returned %d, bytes: ", ret);
        print_bytes(buf, ret);
        fprintf(stderr, "\n");
    }
    ret = libusb_get_descriptor(devh, 0x0000002, 0x0000000, buf, 265);

    ret = libusb_set_interface_alt_setting(devh, 0, 2);
    if (ret != 0) {
        perror("Failed to activate alternate setting for interface");
        return 1;
    }

    /* Disable sound - If this line is removed, we start to receive data with the header [0xaa 0xaa 0x00 0x01] */
    somagic_write_reg(0x1740, 0x00);
    usleep(30 * 1000);

    return 0;
}

static void cameraThreadFunc(void *)
{
    /*
     * Thread runs forever, looking for devices
     */

    libusb_init(NULL);

    while (true) {
        libusb_device *dev;

        dev = find_device(VENDOR, PRODUCT_WITHOUT_FIRMWARE);
        if (dev) {
            install_firmware(dev);
            libusb_unref_device(dev);
            continue;
        }

        for (unsigned p = 0; p < PRODUCT_COUNT; p++) {
            dev = find_device(VENDOR, PRODUCT[p]);
            if (dev) {
                break;
            }
        }

        if (dev) {
            libusb_open(dev, &devh);
            if (devh) {
                if (somagic_init() == 0) {
                    somagic_capture();
                }
                libusb_release_interface(devh, 0);
                libusb_close(devh);
            } else {
                perror("Failed to open USB device");
            }
            libusb_unref_device(dev);
            continue;
        }

        // No devices yet
        sleep(1);
    }
}

namespace Camera {
    tthread::thread* start(videoCallback_t callback, void *context) {
        if (cameraThread) {
            // Only one instance supported
            return 0;
        }

        videoCallback = callback;
        videoCallbackContext = context;

        cameraThread = new tthread::thread(cameraThreadFunc, 0);

        return cameraThread;
    }
}
