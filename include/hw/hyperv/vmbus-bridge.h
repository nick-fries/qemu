/*
 * QEMU Hyper-V VMBus root bridge
 *
 * Copyright (c) 2017-2018 Virtuozzo International GmbH.
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#ifndef HW_HYPERV_VMBUS_BRIDGE_H
#define HW_HYPERV_VMBUS_BRIDGE_H

#include "hw/core/sysbus.h"
#include "hw/hyperv/vmbus.h"
#include "qom/object.h"

#define TYPE_VMBUS_BRIDGE "vmbus-bridge"

/*
 * MMIO window advertised in the VMBus ACPI _CRS for guest drivers that
 * claim framebuffer/MMIO space via vmbus_allocate_mmio() (e.g. Linux
 * hyperv_drm).  Mirrors Hyper-V Gen1, which reserves low MMIO for the
 * synthetic framebuffer at 0xf8000000.  The range lies in the 32-bit
 * PCI hole on both pc and q35 (guest RAM below 4G ends at 0xe0000000
 * resp. 0xb0000000 by default) and is backed by RAM only when an
 * hv-synthvid device maps its VRAM there.
 */
#define VMBUS_MMIO_WINDOW_BASE 0xf8000000ULL
#define VMBUS_MMIO_WINDOW_SIZE 0x800000ULL /* 8 MiB */

struct VMBusBridge {
    SysBusDevice parent_obj;

    uint8_t irq;

    VMBus *bus;
};

OBJECT_DECLARE_SIMPLE_TYPE(VMBusBridge, VMBUS_BRIDGE)

static inline VMBusBridge *vmbus_bridge_find(void)
{
    return VMBUS_BRIDGE(object_resolve_path_type("", TYPE_VMBUS_BRIDGE, NULL));
}

#endif
