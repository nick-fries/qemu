/*
 * QEMU Hyper-V VMBus synthetic video device ("synthvid")
 *
 * Implements the host (VSP) side of the Hyper-V synthetic video
 * protocol so that Windows guests bind the inbox hypervideo.sys
 * display-only driver and Linux guests bind hyperv_fb / hyperv_drm,
 * providing a graphical console without any PCI display device.
 *
 * Protocol references (all first-party):
 *  - Linux drivers/video/fbdev/hyperv_fb.c and drivers/gpu/drm/hyperv/
 *  - Microsoft OpenVMM vm/devices/uidevices/src/video/
 *
 * The guest owns the framebuffer: it announces an 8 MiB VRAM region by
 * guest physical address (VRAM_LOCATION), configures a video mode
 * (SITUATION_UPDATE) and then either sends dirty rectangles (DIRT) or
 * relies on host polling.  The host merely scans out the guest memory.
 *
 * Copyright (c) 2026 Nick Fries
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "migration/blocker.h"
#include "hw/core/qdev-properties.h"
#include "hw/display/edid.h"
#include "hw/hyperv/vmbus.h"
#include "ui/console.h"
#include "trace.h"

#define TYPE_HV_SYNTHVID "hv-synthvid"
OBJECT_DECLARE_SIMPLE_TYPE(HvSynthVid, HV_SYNTHVID)

#define HV_SYNTHVID_CLASS_GUID    "da0a7802-e377-4aac-8e77-0558eb1073f8"
/*
 * Well-known instance GUID used by Hyper-V and OpenVMM for the video
 * device; both Windows' INF and Linux key the device by it.
 */
#define HV_SYNTHVID_INSTANCE_GUID "5620e0c7-8062-4dce-aeb7-520c7ef76171"

/* Guests read the VRAM size from mmio_size_mb in the channel offer */
#define SYNTHVID_VRAM_SIZE_MB   8
#define SYNTHVID_VRAM_SIZE      (SYNTHVID_VRAM_SIZE_MB * MiB)

/* Note: minor in the high half, major in the low half */
#define SYNTHVID_VERSION(major, minor) (((minor) << 16) | (major))
#define SYNTHVID_VERSION_WIN7   SYNTHVID_VERSION(3, 0)
#define SYNTHVID_VERSION_WIN8   SYNTHVID_VERSION(3, 2)
#define SYNTHVID_VERSION_BLUE   SYNTHVID_VERSION(3, 3)
#define SYNTHVID_VERSION_WIN10  SYNTHVID_VERSION(3, 5)

#define SYNTHVID_ACCEPTED_WITH_VERSION_EXCHANGE 2

#define SYNTHVID_MAX_VIDEO_OUTPUTS 1
#define SYNTHVID_DEPTH_WIN8 32

/* Message types */
#define SYNTHVID_ERROR                          0
#define SYNTHVID_VERSION_REQUEST                1
#define SYNTHVID_VERSION_RESPONSE               2
#define SYNTHVID_VRAM_LOCATION                  3
#define SYNTHVID_VRAM_LOCATION_ACK              4
#define SYNTHVID_SITUATION_UPDATE               5
#define SYNTHVID_SITUATION_UPDATE_ACK           6
#define SYNTHVID_POINTER_POSITION               7
#define SYNTHVID_POINTER_SHAPE                  8
#define SYNTHVID_FEATURE_CHANGE                 9
#define SYNTHVID_DIRT                           10
#define SYNTHVID_BIOS_INFO_REQUEST              11
#define SYNTHVID_BIOS_INFO_RESPONSE             12
#define SYNTHVID_SUPPORTED_RESOL_REQUEST        13
#define SYNTHVID_SUPPORTED_RESOL_RESPONSE       14
#define SYNTHVID_CAPABILITY_REQUEST             15
#define SYNTHVID_CAPABILITY_RESPONSE            16

#define SYNTHVID_PIPE_MSG_DATA  1

#define SYNTHVID_CURSOR_MAX_X           96
#define SYNTHVID_CURSOR_MAX_Y           96
#define SYNTHVID_CURSOR_ARGB_PIXEL_SIZE 4
#define SYNTHVID_CURSOR_MAX_DATA \
    (SYNTHVID_CURSOR_MAX_X * SYNTHVID_CURSOR_MAX_Y * \
     SYNTHVID_CURSOR_ARGB_PIXEL_SIZE)
#define SYNTHVID_CURSOR_COMPLETE        0xff

#define SYNTHVID_EDID_BLOCK_SIZE        128

/* The guests' VMBus receive buffer; never send a larger packet */
#define SYNTHVID_MAX_PACKET_SIZE        16384

#define SYNTHVID_MAX_DIMENSION          8192

/* Full-frame refresh period when the guest is not sending DIRT */
#define SYNTHVID_DIRT_FALLBACK_NS       NANOSECONDS_PER_SECOND

/*
 * Wire format.  All messages in both directions are wrapped in a pipe
 * header (the channel is offered in named-pipe mode) followed by a
 * synthvid message header.  Everything is little-endian and packed;
 * VMBus is x86-only so, like hv-balloon, fields are accessed natively.
 */
struct QEMU_PACKED synthvid_pipe_hdr {
    uint32_t type;
    uint32_t size;              /* bytes after this header */
};

struct QEMU_PACKED synthvid_msg_hdr {
    uint32_t type;
    uint32_t size;          /* Linux: incl. header; OpenVMM: body only */
};

struct QEMU_PACKED synthvid_version_req {
    uint32_t version;
};

struct QEMU_PACKED synthvid_version_resp {
    uint32_t version;
    uint8_t is_accepted;
    uint8_t max_video_outputs;
};

struct QEMU_PACKED synthvid_vram_location {
    uint64_t user_ctx;
    uint8_t is_vram_gpa_specified;
    uint64_t vram_gpa;
};

struct QEMU_PACKED synthvid_vram_location_ack {
    uint64_t user_ctx;
};

struct QEMU_PACKED synthvid_video_output_situation {
    uint8_t active;
    uint32_t vram_offset;
    uint8_t depth_bits;
    uint32_t width_pixels;
    uint32_t height_pixels;
    uint32_t pitch_bytes;
};

struct QEMU_PACKED synthvid_situation_update {
    uint64_t user_ctx;
    uint8_t video_output_count;
    struct synthvid_video_output_situation video_output[];
};

struct QEMU_PACKED synthvid_situation_update_ack {
    uint64_t user_ctx;
};

struct QEMU_PACKED synthvid_pointer_position {
    uint8_t is_visible;
    uint8_t video_output;
    int32_t image_x;
    int32_t image_y;
};

struct QEMU_PACKED synthvid_pointer_shape {
    uint8_t part_idx;
    uint8_t is_argb;
    uint32_t width;
    uint32_t height;
    uint32_t hot_x;
    uint32_t hot_y;
    uint8_t data[];
};

struct QEMU_PACKED synthvid_feature_change {
    uint8_t is_dirt_needed;
    uint8_t is_ptr_pos_needed;
    uint8_t is_ptr_shape_needed;
    uint8_t is_situ_needed;
};

struct QEMU_PACKED synthvid_rect {
    int32_t x1, y1;             /* top-left, inclusive */
    int32_t x2, y2;             /* bottom-right, exclusive */
};

struct QEMU_PACKED synthvid_dirt {
    uint8_t video_output;
    uint8_t dirt_count;
    struct synthvid_rect rect[];
};

struct QEMU_PACKED synthvid_supported_resol_req {
    uint8_t maximum_resolution_count;
};

struct QEMU_PACKED synthvid_screen_info {
    uint16_t width;
    uint16_t height;
};

struct QEMU_PACKED synthvid_supported_resol_resp {
    uint8_t edid_block[SYNTHVID_EDID_BLOCK_SIZE];
    uint8_t resolution_count;
    uint8_t default_resolution_index;
    uint8_t is_standard;
    struct synthvid_screen_info supported_resolution[];
};

struct QEMU_PACKED synthvid_bios_info_resp {
    uint32_t stop_device_supported;
    uint8_t reserved[12];
};

struct QEMU_PACKED synthvid_capability_resp {
    uint32_t lock_on_disconnect;
    uint32_t reserved[15];
};

QEMU_BUILD_BUG_ON(sizeof(struct synthvid_pipe_hdr) != 8);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_msg_hdr) != 8);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_version_req) != 4);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_version_resp) != 6);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_vram_location) != 17);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_vram_location_ack) != 8);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_video_output_situation) != 18);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_situation_update) != 9);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_situation_update_ack) != 8);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_pointer_position) != 10);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_pointer_shape) != 18);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_feature_change) != 4);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_rect) != 16);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_dirt) != 2);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_supported_resol_req) != 1);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_screen_info) != 4);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_supported_resol_resp) != 131);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_bios_info_resp) != 16);
QEMU_BUILD_BUG_ON(sizeof(struct synthvid_capability_resp) != 64);

/*
 * Common video modes offered in SUPPORTED_RESOL_RESPONSE, filtered at
 * run time to those fitting the 8 MiB VRAM at 32 bpp.
 */
static const struct synthvid_screen_info synthvid_modes[] = {
    {  640,  480 },
    {  800,  600 },
    { 1024,  768 },
    { 1280,  720 },
    { 1280, 1024 },
    { 1440,  900 },
    { 1600,  900 },
    { 1680, 1050 },
    { 1920, 1080 },
    { 1920, 1200 },
};

/*
 * Framing overhead (pipe header + message header) plus the largest
 * host-to-guest message body.
 */
#define SYNTHVID_TX_MAX \
    (sizeof(struct synthvid_pipe_hdr) + sizeof(struct synthvid_msg_hdr) + \
     sizeof(struct synthvid_supported_resol_resp) + \
     ARRAY_SIZE(synthvid_modes) * sizeof(struct synthvid_screen_info))

/*
 * Responses waiting for ring-buffer space.  The protocol is
 * request/response with well-behaved guests waiting for each answer,
 * so the queue depth only needs to cover one full handshake burst
 * (version response + feature change + resolutions + VRAM ack +
 * situation ack).  Overflow can only be caused by a guest that floods
 * requests without ever reading responses; in that case new messages
 * are dropped (and traced).
 */
#define SYNTHVID_TX_QUEUE_LEN   8

typedef struct HvSynthVidReq {
    VMBusChanReq vmreq;
} HvSynthVidReq;

struct HvSynthVid {
    VMBusDevice parent;

    QemuConsole *con;
    qemu_edid_info edid_info;
    uint8_t edid[SYNTHVID_EDID_BLOCK_SIZE];
    Error *migration_blocker;

    /* Negotiated protocol version, 0 when not negotiated */
    uint32_t version;

    /* Guest VRAM mapping */
    void *vram_ptr;
    uint64_t vram_gpa;
    uint64_t vram_size;

    /* Current video output situation */
    bool situ_active;
    bool scanout_valid;         /* display surface points into VRAM */
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    uint32_t vram_offset;

    /* Fallback refresh for guests which do not send DIRT */
    int64_t last_dirt_ns;
    bool invalidated;

    /* Pointer shape reassembly (possibly multi-part) */
    bool shape_pending;
    bool shape_argb;
    uint8_t shape_next_part;
    uint32_t shape_width;
    uint32_t shape_height;
    uint32_t shape_hot_x;
    uint32_t shape_hot_y;
    uint32_t shape_expected;
    uint32_t shape_recvd;
    uint32_t shape_parts;
    uint8_t shape_data[SYNTHVID_CURSOR_MAX_DATA];

    /* Pending host-to-guest messages (FIFO, see SYNTHVID_TX_QUEUE_LEN) */
    struct {
        uint32_t len;
        uint8_t buf[SYNTHVID_TX_MAX];
    } txq[SYNTHVID_TX_QUEUE_LEN];
    unsigned int txq_head;
    unsigned int txq_len;
};

OBJECT_DEFINE_SIMPLE_TYPE(HvSynthVid, hv_synthvid, HV_SYNTHVID, VMBUS_DEVICE)

/* ------------------------------------------------------------------ */
/* Send path                                                          */

static void synthvid_txq_push(HvSynthVid *s, const void *buf, uint32_t len)
{
    unsigned int tail;

    if (s->txq_len == SYNTHVID_TX_QUEUE_LEN) {
        trace_hv_synthvid_tx_queue_full(len);
        return;
    }
    tail = (s->txq_head + s->txq_len) % SYNTHVID_TX_QUEUE_LEN;
    memcpy(s->txq[tail].buf, buf, len);
    s->txq[tail].len = len;
    s->txq_len++;
}

static void synthvid_flush_txq(HvSynthVid *s, VMBusChannel *chan)
{
    while (s->txq_len) {
        uint32_t len = s->txq[s->txq_head].len;

        if (!vmbus_channel_is_open(chan) ||
            vmbus_channel_reserve(chan, 0, len) < 0) {
            /*
             * No room yet: vmbus_channel_reserve() armed pending_send_sz
             * so the guest will notify us when space frees up; retry
             * then (from the channel notify callback).
             */
            return;
        }
        vmbus_channel_send(chan, VMBUS_PACKET_DATA_INBAND, NULL, 0,
                           s->txq[s->txq_head].buf, len, false, 0);
        s->txq_head = (s->txq_head + 1) % SYNTHVID_TX_QUEUE_LEN;
        s->txq_len--;
    }
}

static void synthvid_send_msg(HvSynthVid *s, VMBusChannel *chan,
                              uint32_t type, const void *body,
                              uint32_t body_len)
{
    uint8_t buf[SYNTHVID_TX_MAX];
    struct synthvid_pipe_hdr *pipe = (struct synthvid_pipe_hdr *)buf;
    struct synthvid_msg_hdr *hdr =
        (struct synthvid_msg_hdr *)(buf + sizeof(*pipe));
    uint32_t len = sizeof(*pipe) + sizeof(*hdr) + body_len;
    int ret;

    /* Host-side invariants: all our messages are small and bounded */
    assert(len <= sizeof(buf));
    QEMU_BUILD_BUG_ON(SYNTHVID_TX_MAX > SYNTHVID_MAX_PACKET_SIZE);

    if (!vmbus_channel_is_open(chan)) {
        return;
    }

    pipe->type = SYNTHVID_PIPE_MSG_DATA;
    pipe->size = sizeof(*hdr) + body_len;
    hdr->type = type;
    /*
     * Header-inclusive size, matching what the Linux guest drivers
     * themselves send (and thus what real Hyper-V traffic looks like);
     * all known receivers ignore this field anyway.
     */
    hdr->size = sizeof(*hdr) + body_len;
    memcpy(buf + sizeof(*pipe) + sizeof(*hdr), body, body_len);

    if (s->txq_len) {
        /* Keep ordering with already queued messages */
        synthvid_txq_push(s, buf, len);
        return;
    }

    ret = vmbus_channel_reserve(chan, 0, len);
    if (ret == -ENOSPC) {
        /*
         * Never drop a response: hyperv_fb waits 10 s for the VRAM ack
         * and fails the whole device without it.  Queue and retry when
         * the guest signals free space.
         */
        trace_hv_synthvid_send_retry(type, len);
        synthvid_txq_push(s, buf, len);
        return;
    }
    if (ret < 0) {
        /* Ring unmappable; the channel is being torn down */
        return;
    }
    vmbus_channel_send(chan, VMBUS_PACKET_DATA_INBAND, NULL, 0,
                       buf, len, false, 0);
}

/* ------------------------------------------------------------------ */
/* Display                                                            */

static void synthvid_set_placeholder(HvSynthVid *s, const char *msg)
{
    DisplaySurface *surface;

    if (!s->con) {
        return;
    }
    surface = qemu_create_placeholder_surface(640, 480, msg);
    dpy_gfx_replace_surface(s->con, surface);
    s->scanout_valid = false;
}

static void synthvid_unmap_vram(HvSynthVid *s)
{
    VMBusDevice *vdev = VMBUS_DEVICE(s);

    if (!s->vram_ptr) {
        return;
    }
    if (s->scanout_valid) {
        /* Drop the surface referencing the mapping before unmapping */
        synthvid_set_placeholder(s, "Guest disabled display");
    }
    dma_memory_unmap(vdev->dma_as, s->vram_ptr, s->vram_size,
                     DMA_DIRECTION_TO_DEVICE, 0);
    s->vram_ptr = NULL;
    s->vram_size = 0;
    s->vram_gpa = 0;
}

/*
 * (Re)build the scanout surface from the current situation, or show a
 * placeholder when there is no active, displayable video output.
 *
 * The surface aliases the persistent VRAM mapping without a pixman
 * destroy callback; lifetime is managed manually instead: the surface
 * is always replaced (synthvid_set_placeholder()) before the mapping
 * is dropped, and everything runs in main-loop context.
 */
static void synthvid_apply_situation(HvSynthVid *s)
{
    DisplaySurface *surface;

    if (!s->con) {
        return;
    }
    if (!s->situ_active || !s->vram_ptr) {
        synthvid_set_placeholder(s, "Guest disabled display");
        return;
    }
    surface = qemu_create_displaysurface_from(s->width, s->height,
                                              PIXMAN_x8r8g8b8, s->pitch,
                                              (uint8_t *)s->vram_ptr +
                                              s->vram_offset);
    dpy_gfx_replace_surface(s->con, surface);
    s->scanout_valid = true;
    s->last_dirt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void synthvid_invalidate(void *opaque)
{
    HvSynthVid *s = opaque;

    s->invalidated = true;
}

static void synthvid_gfx_update(void *opaque)
{
    HvSynthVid *s = opaque;

    if (!s->scanout_valid) {
        return;
    }
    /*
     * DIRT messages normally drive updates.  If the guest has not sent
     * one for a while (guests are free to ignore FEATURE_CHANGE and
     * rely on host polling, as OpenVMM-facing Windows does), fall back
     * to full-frame refreshes; cheap, as the surface memory is the
     * guest VRAM itself.
     */
    if (s->invalidated ||
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->last_dirt_ns >
        SYNTHVID_DIRT_FALLBACK_NS) {
        s->invalidated = false;
        dpy_gfx_update_full(s->con);
    }
}

static const GraphicHwOps synthvid_gfx_ops = {
    .invalidate = synthvid_invalidate,
    .gfx_update = synthvid_gfx_update,
};

/* ------------------------------------------------------------------ */
/* Guest message handlers                                             */

static void
synthvid_handle_version_request(HvSynthVid *s, VMBusChannel *chan,
                                const struct synthvid_version_req *req)
{
    struct synthvid_version_resp resp = {
        .version = req->version,
        .max_video_outputs = SYNTHVID_MAX_VIDEO_OUTPUTS,
    };
    bool accepted;

    trace_hv_synthvid_version_request(req->version);

    switch (req->version) {
    case SYNTHVID_VERSION_WIN7:
    case SYNTHVID_VERSION_WIN8:
    case SYNTHVID_VERSION_BLUE:
    case SYNTHVID_VERSION_WIN10:
        accepted = true;
        break;
    default:
        /*
         * Reject explicitly (is_accepted = 0) so the guest's documented
         * fallthrough retry with a lower version works.
         */
        accepted = false;
        break;
    }

    resp.is_accepted = accepted ? SYNTHVID_ACCEPTED_WITH_VERSION_EXCHANGE : 0;
    s->version = accepted ? req->version : 0;

    trace_hv_synthvid_version_response(resp.version, resp.is_accepted);
    synthvid_send_msg(s, chan, SYNTHVID_VERSION_RESPONSE,
                      &resp, sizeof(resp));

    if (accepted) {
        /*
         * Unlike OpenVMM, ask for dirty rectangles and pointer
         * messages right away: without a host-initiated FEATURE_CHANGE
         * the Linux drivers never send DIRT and the console would rely
         * on polling alone.  Sent after every successful negotiation
         * so reconnects renegotiate features too.
         */
        struct synthvid_feature_change feat = {
            .is_dirt_needed = 1,
            .is_ptr_pos_needed = 1,
            .is_ptr_shape_needed = 1,
            .is_situ_needed = 1,
        };

        trace_hv_synthvid_feature_change_sent();
        synthvid_send_msg(s, chan, SYNTHVID_FEATURE_CHANGE,
                          &feat, sizeof(feat));
    }
}

static void
synthvid_handle_vram_location(HvSynthVid *s, VMBusChannel *chan,
                              const struct synthvid_vram_location *loc)
{
    VMBusDevice *vdev = VMBUS_DEVICE(s);
    struct synthvid_vram_location_ack ack = {
        .user_ctx = loc->user_ctx,
    };

    trace_hv_synthvid_vram_location(loc->vram_gpa,
                                    loc->is_vram_gpa_specified);

    synthvid_unmap_vram(s);

    if (loc->is_vram_gpa_specified) {
        dma_addr_t len = SYNTHVID_VRAM_SIZE;
        void *ptr;

        /*
         * Map the full VRAM window; the GPA may be plain guest RAM
         * (hyperv_fb Gen1 CMA path) or a VMBus MMIO-window address.
         * Only a contiguous, full-length direct mapping is usable for
         * scanout.
         */
        ptr = dma_memory_map(vdev->dma_as, loc->vram_gpa, &len,
                             DMA_DIRECTION_TO_DEVICE, MEMTXATTRS_UNSPECIFIED);
        if (!ptr || len != SYNTHVID_VRAM_SIZE) {
            trace_hv_synthvid_vram_map_failed(loc->vram_gpa,
                                              ptr ? (uint64_t)len : 0);
            if (ptr) {
                dma_memory_unmap(vdev->dma_as, ptr, len,
                                 DMA_DIRECTION_TO_DEVICE, 0);
            }
        } else {
            s->vram_ptr = ptr;
            s->vram_size = len;
            s->vram_gpa = loc->vram_gpa;
        }
    }

    /*
     * Ack unconditionally, echoing user_ctx exactly (hyperv_fb fails
     * the device on mismatch, and would time out after 10 s without
     * the ack).  An unmappable GPA was traced above and simply leaves
     * the display on the placeholder surface, keeping device state
     * consistent.
     */
    synthvid_send_msg(s, chan, SYNTHVID_VRAM_LOCATION_ACK,
                      &ack, sizeof(ack));

    if (s->situ_active) {
        /*
         * A situation can legitimately precede or survive a VRAM move;
         * bounds stay valid as the window size is fixed at 8 MiB.
         */
        synthvid_apply_situation(s);
    }
}

static void
synthvid_handle_situation_update(HvSynthVid *s, VMBusChannel *chan,
                                 const struct synthvid_situation_update *situ,
                                 uint32_t body_len)
{
    struct synthvid_situation_update_ack ack = {
        .user_ctx = situ->user_ctx,
    };
    const struct synthvid_video_output_situation *vo = &situ->video_output[0];
    bool active = false;

    if (situ->video_output_count == 0) {
        /* No outputs: treat as inactive */
        trace_hv_synthvid_situation_update(0, 0, 0, 0, 0, 0);
    } else {
        if (body_len < sizeof(*situ) + sizeof(*vo)) {
            trace_hv_synthvid_malformed_message(SYNTHVID_SITUATION_UPDATE,
                                                body_len);
            return;
        }
        if (situ->video_output_count > 1) {
            /* Everyone advertises a single output; use entry 0 */
            trace_hv_synthvid_situation_extra_outputs(situ->video_output_count);
        }

        trace_hv_synthvid_situation_update(vo->active, vo->width_pixels,
                                           vo->height_pixels, vo->pitch_bytes,
                                           vo->vram_offset, vo->depth_bits);

        active = vo->active != 0;
        if (active) {
            /* All fields are hostile; validate before use */
            if (vo->depth_bits != SYNTHVID_DEPTH_WIN8 ||
                vo->width_pixels < 1 ||
                vo->width_pixels > SYNTHVID_MAX_DIMENSION ||
                vo->height_pixels < 1 ||
                vo->height_pixels > SYNTHVID_MAX_DIMENSION ||
                vo->pitch_bytes < vo->width_pixels * 4 ||
                !s->vram_ptr ||
                (uint64_t)vo->vram_offset +
                (uint64_t)vo->pitch_bytes * vo->height_pixels >
                s->vram_size) {
                trace_hv_synthvid_situation_invalid(vo->width_pixels,
                                                    vo->height_pixels,
                                                    vo->pitch_bytes,
                                                    vo->vram_offset,
                                                    vo->depth_bits);
                active = false;
            }
        }
    }

    s->situ_active = active;
    if (active) {
        s->width = vo->width_pixels;
        s->height = vo->height_pixels;
        s->pitch = vo->pitch_bytes;
        s->vram_offset = vo->vram_offset;
    }
    synthvid_apply_situation(s);

    /* Always ack: Linux doesn't wait for it but Windows may */
    synthvid_send_msg(s, chan, SYNTHVID_SITUATION_UPDATE_ACK,
                      &ack, sizeof(ack));
}

static void
synthvid_handle_pointer_position(HvSynthVid *s,
                                 const struct synthvid_pointer_position *pos)
{
    if (!s->con) {
        return;
    }
    dpy_mouse_set(s->con, pos->image_x, pos->image_y, pos->is_visible != 0);
}

static void synthvid_shape_reset(HvSynthVid *s)
{
    s->shape_pending = false;
    s->shape_recvd = 0;
    s->shape_parts = 0;
    s->shape_next_part = 0;
}

static uint32_t synthvid_shape_data_size(bool argb, uint32_t width,
                                         uint32_t height)
{
    if (argb) {
        return width * height * SYNTHVID_CURSOR_ARGB_PIXEL_SIZE;
    }
    /* Two-plane monochrome: AND mask rows followed by XOR mask rows */
    return 2 * DIV_ROUND_UP(width, 8) * height;
}

static void synthvid_shape_complete(HvSynthVid *s)
{
    QEMUCursor *c;

    c = cursor_alloc(s->shape_width, s->shape_height);
    if (!c) {
        return;
    }
    c->hot_x = MIN(s->shape_hot_x, s->shape_width - 1);
    c->hot_y = MIN(s->shape_hot_y, s->shape_height - 1);

    if (s->shape_argb) {
        /*
         * The guest sends 0xAARRGGBB words, which is also the QEMUCursor
         * data format (see the SPICE alpha-cursor path in qxl-render.c
         * and the channel masks in ui/sdl2.c).
         */
        memcpy(c->data, s->shape_data, s->shape_expected);
    } else {
        uint32_t bpl = DIV_ROUND_UP(s->shape_width, 8);
        uint8_t *and_mask = s->shape_data;
        uint8_t *xor_mask = s->shape_data + bpl * s->shape_height;

        cursor_set_mono(c, 0xffffff, 0x000000, xor_mask, 1, and_mask);
    }

    trace_hv_synthvid_pointer_shape(s->shape_argb, s->shape_width,
                                    s->shape_height, s->shape_parts);
    if (s->con) {
        dpy_cursor_define(s->con, c);
    }
    cursor_unref(c);
}

static void
synthvid_handle_pointer_shape(HvSynthVid *s,
                              const struct synthvid_pointer_shape *shape,
                              uint32_t body_len)
{
    bool final = shape->part_idx == SYNTHVID_CURSOR_COMPLETE;
    uint32_t data_len = body_len - sizeof(*shape);
    uint32_t copy;

    if (s->shape_pending &&
        ((shape->is_argb != 0) != s->shape_argb ||
         shape->width != s->shape_width ||
         shape->height != s->shape_height ||
         (!final && shape->part_idx != s->shape_next_part))) {
        /* Inconsistent continuation; drop the partial shape */
        trace_hv_synthvid_shape_dropped(shape->part_idx, s->shape_next_part);
        synthvid_shape_reset(s);
    }

    if (!s->shape_pending) {
        if (!final && shape->part_idx != 0) {
            /* Continuation of something we never saw the start of */
            trace_hv_synthvid_shape_dropped(shape->part_idx, 0);
            return;
        }
        if (shape->width < 1 || shape->width > SYNTHVID_CURSOR_MAX_X ||
            shape->height < 1 || shape->height > SYNTHVID_CURSOR_MAX_Y) {
            trace_hv_synthvid_malformed_message(SYNTHVID_POINTER_SHAPE,
                                                body_len);
            return;
        }
        s->shape_pending = true;
        s->shape_argb = shape->is_argb != 0;
        s->shape_width = shape->width;
        s->shape_height = shape->height;
        s->shape_hot_x = shape->hot_x;
        s->shape_hot_y = shape->hot_y;
        s->shape_expected = synthvid_shape_data_size(s->shape_argb,
                                                     shape->width,
                                                     shape->height);
        s->shape_recvd = 0;
        s->shape_parts = 0;
        s->shape_next_part = 1;
    } else {
        s->shape_next_part++;
    }
    s->shape_parts++;

    /*
     * The only guest-sized copy in the device; hard-capped by
     * shape_expected <= SYNTHVID_CURSOR_MAX_DATA (96x96 ARGB).
     * data_len may include up to 7 bytes of VMBus qword padding, which
     * the MIN() discards on the final part.
     */
    copy = MIN(data_len, s->shape_expected - s->shape_recvd);
    memcpy(s->shape_data + s->shape_recvd, shape->data, copy);
    s->shape_recvd += copy;

    if (final) {
        if (s->shape_recvd >= s->shape_expected) {
            synthvid_shape_complete(s);
        } else {
            trace_hv_synthvid_shape_incomplete(s->shape_recvd,
                                               s->shape_expected);
        }
        synthvid_shape_reset(s);
    }
}

static void synthvid_handle_dirt(HvSynthVid *s,
                                 const struct synthvid_dirt *dirt,
                                 uint32_t body_len)
{
    uint32_t avail = (body_len - sizeof(*dirt)) / sizeof(struct synthvid_rect);
    uint32_t count = MIN(dirt->dirt_count, avail);
    uint32_t i;

    trace_hv_synthvid_dirt(dirt->dirt_count, count);

    s->last_dirt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!s->scanout_valid) {
        return;
    }

    for (i = 0; i < count; i++) {
        const struct synthvid_rect *r = &dirt->rect[i];
        /* Clamp hostile coordinates to the current mode; x2/y2 exclusive */
        int32_t x1 = MAX(r->x1, 0);
        int32_t y1 = MAX(r->y1, 0);
        int32_t x2 = MIN(r->x2, (int32_t)s->width);
        int32_t y2 = MIN(r->y2, (int32_t)s->height);

        if (x2 <= x1 || y2 <= y1) {
            continue;
        }
        dpy_gfx_update(s->con, x1, y1, x2 - x1, y2 - y1);
    }
}

static void synthvid_handle_bios_info_request(HvSynthVid *s,
                                              VMBusChannel *chan)
{
    struct synthvid_bios_info_resp resp = {
        .stop_device_supported = 1,
    };

    synthvid_send_msg(s, chan, SYNTHVID_BIOS_INFO_RESPONSE,
                      &resp, sizeof(resp));
}

static void
synthvid_handle_resol_request(HvSynthVid *s, VMBusChannel *chan,
                              const struct synthvid_supported_resol_req *req)
{
    uint8_t buf[sizeof(struct synthvid_supported_resol_resp) +
                ARRAY_SIZE(synthvid_modes) *
                sizeof(struct synthvid_screen_info)];
    struct synthvid_supported_resol_resp *resp =
        (struct synthvid_supported_resol_resp *)buf;
    uint32_t max = req->maximum_resolution_count;
    uint32_t count = 0;
    uint32_t def_idx = 0;
    uint32_t fallback_idx = 0;
    bool def_found = false;
    unsigned int i;

    trace_hv_synthvid_resolution_request(req->maximum_resolution_count);

    /* Always respond with at least one entry */
    max = MAX(max, 1);

    memset(buf, 0, sizeof(buf));
    for (i = 0; i < ARRAY_SIZE(synthvid_modes) && count < max; i++) {
        const struct synthvid_screen_info *mode = &synthvid_modes[i];

        if ((uint64_t)mode->width * mode->height * 4 > SYNTHVID_VRAM_SIZE) {
            continue;
        }
        if (mode->width == s->edid_info.prefx &&
            mode->height == s->edid_info.prefy) {
            def_idx = count;
            def_found = true;
        }
        if (mode->width == 1024 && mode->height == 768) {
            fallback_idx = count;
        }
        resp->supported_resolution[count] = *mode;
        count++;
    }
    if (!def_found) {
        def_idx = fallback_idx;
    }

    memcpy(resp->edid_block, s->edid, sizeof(resp->edid_block));
    resp->resolution_count = count;
    resp->default_resolution_index = def_idx;
    resp->is_standard = 1;

    synthvid_send_msg(s, chan, SYNTHVID_SUPPORTED_RESOL_RESPONSE, buf,
                      sizeof(*resp) +
                      count * sizeof(struct synthvid_screen_info));
}

static void synthvid_handle_capability_request(HvSynthVid *s,
                                               VMBusChannel *chan)
{
    struct synthvid_capability_resp resp = {
        .lock_on_disconnect = 0,
    };

    synthvid_send_msg(s, chan, SYNTHVID_CAPABILITY_RESPONSE,
                      &resp, sizeof(resp));
}

/* ------------------------------------------------------------------ */
/* Receive path                                                       */

static void synthvid_handle_message(HvSynthVid *s, VMBusChannel *chan,
                                    HvSynthVidReq *req)
{
    VMBusChanReq *vmreq = &req->vmreq;
    const struct synthvid_pipe_hdr *pipe = vmreq->msg;
    const struct synthvid_msg_hdr *hdr;
    const void *body;
    uint32_t body_len, min_len;

    /*
     * vmreq->msglen is rounded up to a qword multiple by VMBus framing,
     * so every length check below is >=, never ==.  Neither guest-
     * provided size field is trusted for bounds; pipe->size (which is
     * consistent across the known implementations, unlike hdr->size)
     * is only used, after validation, to trim the qword padding off
     * variable-length payloads.
     */
    if (vmreq->pkt_type != VMBUS_PACKET_DATA_INBAND ||
        vmreq->msglen < sizeof(*pipe) + sizeof(*hdr)) {
        trace_hv_synthvid_malformed_message(vmreq->pkt_type, vmreq->msglen);
        return;
    }
    if (pipe->type != SYNTHVID_PIPE_MSG_DATA) {
        trace_hv_synthvid_malformed_message(pipe->type, vmreq->msglen);
        return;
    }

    hdr = (const struct synthvid_msg_hdr *)(pipe + 1);
    body = hdr + 1;
    body_len = vmreq->msglen - sizeof(*pipe) - sizeof(*hdr);
    if (pipe->size >= sizeof(*hdr) &&
        pipe->size - sizeof(*hdr) <= body_len) {
        body_len = pipe->size - sizeof(*hdr);
    }

    switch (hdr->type) {
    case SYNTHVID_VERSION_REQUEST:
        min_len = sizeof(struct synthvid_version_req);
        break;
    case SYNTHVID_VRAM_LOCATION:
        min_len = sizeof(struct synthvid_vram_location);
        break;
    case SYNTHVID_SITUATION_UPDATE:
        min_len = sizeof(struct synthvid_situation_update);
        break;
    case SYNTHVID_POINTER_POSITION:
        min_len = sizeof(struct synthvid_pointer_position);
        break;
    case SYNTHVID_POINTER_SHAPE:
        min_len = sizeof(struct synthvid_pointer_shape);
        break;
    case SYNTHVID_DIRT:
        min_len = sizeof(struct synthvid_dirt);
        break;
    case SYNTHVID_BIOS_INFO_REQUEST:
        min_len = 0;
        break;
    case SYNTHVID_SUPPORTED_RESOL_REQUEST:
        min_len = sizeof(struct synthvid_supported_resol_req);
        break;
    case SYNTHVID_CAPABILITY_REQUEST:
        min_len = 0;
        break;
    default:
        trace_hv_synthvid_unknown_message(hdr->type, vmreq->msglen);
        return;
    }

    if (body_len < min_len) {
        trace_hv_synthvid_malformed_message(hdr->type, vmreq->msglen);
        return;
    }

    switch (hdr->type) {
    case SYNTHVID_VERSION_REQUEST:
        synthvid_handle_version_request(s, chan, body);
        break;
    case SYNTHVID_VRAM_LOCATION:
        synthvid_handle_vram_location(s, chan, body);
        break;
    case SYNTHVID_SITUATION_UPDATE:
        synthvid_handle_situation_update(s, chan, body, body_len);
        break;
    case SYNTHVID_POINTER_POSITION:
        synthvid_handle_pointer_position(s, body);
        break;
    case SYNTHVID_POINTER_SHAPE:
        synthvid_handle_pointer_shape(s, body, body_len);
        break;
    case SYNTHVID_DIRT:
        synthvid_handle_dirt(s, body, body_len);
        break;
    case SYNTHVID_BIOS_INFO_REQUEST:
        synthvid_handle_bios_info_request(s, chan);
        break;
    case SYNTHVID_SUPPORTED_RESOL_REQUEST:
        synthvid_handle_resol_request(s, chan, body);
        break;
    case SYNTHVID_CAPABILITY_REQUEST:
        synthvid_handle_capability_request(s, chan);
        break;
    default:
        g_assert_not_reached();
    }
}

static bool synthvid_recv_channel(HvSynthVid *s, VMBusChannel *chan)
{
    HvSynthVidReq *req;

    if (vmbus_channel_recv_start(chan)) {
        return false;
    }

    while ((req = vmbus_channel_recv_peek(chan, sizeof(*req)))) {
        synthvid_handle_message(s, chan, req);
        vmbus_free_req(req);
        vmbus_channel_recv_pop(chan);
    }

    return vmbus_channel_recv_done(chan) > 0;
}

static void synthvid_chan_notify_cb(VMBusChannel *chan)
{
    HvSynthVid *s = HV_SYNTHVID(vmbus_channel_device(chan));

    do {
        synthvid_flush_txq(s, chan);
    } while (synthvid_recv_channel(s, chan));
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */

/*
 * Full protocol-state reset: used for guest close/reopen (hibernation,
 * driver reload, Windows PnP restart) and device reset.  A reopened
 * channel renegotiates from scratch, including a fresh FEATURE_CHANGE
 * after the new VERSION_RESPONSE.
 */
static void synthvid_disconnect(HvSynthVid *s)
{
    synthvid_unmap_vram(s);
    synthvid_shape_reset(s);
    s->version = 0;
    s->situ_active = false;
    s->scanout_valid = false;
    s->width = 0;
    s->height = 0;
    s->pitch = 0;
    s->vram_offset = 0;
    s->last_dirt_ns = 0;
    s->invalidated = false;
    s->txq_head = 0;
    s->txq_len = 0;
}

static int synthvid_open_channel(VMBusChannel *chan)
{
    /* The guest speaks first (VERSION_REQUEST); nothing to do */
    return 0;
}

static void synthvid_close_channel(VMBusChannel *chan)
{
    HvSynthVid *s = HV_SYNTHVID(vmbus_channel_device(chan));

    synthvid_disconnect(s);
}

static void synthvid_vmdev_reset(VMBusDevice *vdev)
{
    HvSynthVid *s = HV_SYNTHVID(vdev);

    synthvid_disconnect(s);
    synthvid_set_placeholder(s, "Guest has not initialized the display (yet)");
}

static void synthvid_vmdev_realize(VMBusDevice *vdev, Error **errp)
{
    HvSynthVid *s = HV_SYNTHVID(vdev);

    error_setg(&s->migration_blocker,
               TYPE_HV_SYNTHVID " does not support migration yet");
    if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
        return;
    }

    qemu_edid_generate(s->edid, sizeof(s->edid), &s->edid_info);

    s->con = graphic_console_init(DEVICE(vdev), 0, &synthvid_gfx_ops, s);
}

static void synthvid_vmdev_unrealize(VMBusDevice *vdev)
{
    HvSynthVid *s = HV_SYNTHVID(vdev);

    synthvid_disconnect(s);
    migrate_del_blocker(&s->migration_blocker);
    /* Graphic consoles cannot be destroyed; the device is not hotpluggable */
}

static const Property hv_synthvid_properties[] = {
    DEFINE_PROP_UINT32("xres", HvSynthVid, edid_info.prefx, 1024),
    DEFINE_PROP_UINT32("yres", HvSynthVid, edid_info.prefy, 768),
};

static void hv_synthvid_init(Object *obj)
{
    VMBusDevice *vdev = VMBUS_DEVICE(obj);

    /*
     * Default to the well-known Hyper-V video instance GUID.  Set on
     * the instance (not on VMBusDeviceClass::instanceid, which realize
     * would enforce as immutable) so the "instanceid" property remains
     * user-overridable.
     */
    qemu_uuid_parse(HV_SYNTHVID_INSTANCE_GUID, &vdev->instanceid);
}

static void hv_synthvid_finalize(Object *obj)
{
}

static void hv_synthvid_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VMBusDeviceClass *vdc = VMBUS_DEVICE_CLASS(klass);

    qemu_uuid_parse(HV_SYNTHVID_CLASS_GUID, &vdc->classid);

    dc->desc = "Hyper-V synthetic video";
    device_class_set_props(dc, hv_synthvid_properties);
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    /* Graphic consoles cannot be torn down at runtime */
    dc->hotpluggable = false;

    /* Guests are offered a named-pipe mode channel with 8 MiB of "VRAM" */
    vdc->channel_flags = VMBUS_CHANNEL_NAMED_PIPE_MODE;
    vdc->mmio_size_mb = SYNTHVID_VRAM_SIZE_MB;

    vdc->vmdev_realize = synthvid_vmdev_realize;
    vdc->vmdev_unrealize = synthvid_vmdev_unrealize;
    vdc->vmdev_reset = synthvid_vmdev_reset;
    vdc->open_channel = synthvid_open_channel;
    vdc->close_channel = synthvid_close_channel;
    vdc->chan_notify_cb = synthvid_chan_notify_cb;
}
