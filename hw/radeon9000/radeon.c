//#include "hw/hw.h"
#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/pci/pci.h"
#include <time.h>
#include <stdint.h>

#define UNUSED(x) (void)(x)


typedef struct PCIRadeonDevState {
    PCIDevice parent_obj;

    /* for VRAM */
    MemoryRegion vram;
    /* for IO */
    MemoryRegion io;
    /* for REGS */
    MemoryRegion regs;

    /* irq used */
    qemu_irq irq;
    /* dma buf size */
    unsigned int dma_size;
    /* buffer copied with the dma operation on RAM */
    char *dma_buf;
    /* did we throw an interrupt ? */
    int threw_irq;
    /* id of the device, writable */
    int id;


} PCIRadeonDevState;

#define TYPE_PCI_RADEON_DEV "pci-radeondev"
#define PCI_RADEON_DEV(obj)     OBJECT_CHECK(PCIRadeonDevState, (obj), TYPE_PCI_RADEON_DEV)
/* sizes must be power of 2 in PCI */
#define RADEON_REGS_SIZE (64*1024)
#define RADEON_VRAM_SIZE (128*1024*1024)
#define RADEON_IO_SIZE   (4*1024)

static uint8_t vram[RADEON_VRAM_SIZE];

static void qdev_pci_radeon_dev_reset(DeviceState *dev)
{
    printf("Reset World\n");
}

static void radeon_regs_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    PCIDevice *pci_dev = (PCIDevice *) opaque;

    UNUSED(d);
    UNUSED(pci_dev);

    printf("REGS    Write Ordered, addr=%x, value=%lu, size=%d\n", (unsigned) addr, value, size);

    switch (addr) {
        default:
            printf("Io not used\n");        
    }

}

static uint64_t radeon_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    printf("REGS Read Ordered, addr =%x, size=%d\n", (unsigned) addr, size);

    UNUSED(d);

    switch (addr) {
        default:
            printf("Io not used\n");
            return 0x0;
            
    }
}

static uint64_t radeon_vram_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    printf("VRAM Read Ordered, addr =%x, size=%d\n",(unsigned)  addr, size);
    uint64_t value = 0;

    UNUSED(d);

    switch(size)
    {
        case 1:
            value = *(uint8_t*)&vram[addr];
            break;
        case 2:
            value = *(uint16_t*)&vram[addr];
            break;
        case 4:
            value = *(uint32_t*)&vram[addr];
            break;
        case 8:
            value = *(uint64_t*)&vram[addr];
            break;
    }

    return value;
}

static void radeon_vram_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    printf("VRAM write Ordered, addr=%x, value=%lu, size=%d\n",(unsigned)  addr, value, size);
    UNUSED(d);


    switch(size)
    {
        case 1:
            *(uint8_t*)&vram[addr] = value;
            break;
        case 2:
            *(uint16_t*)&vram[addr] = value;
            break;
        case 4:
            *(uint32_t*)&vram[addr] = value;
            break;
        case 8:
            *(uint64_t*)&vram[addr] = value;
            break;
    }
}

static void radeon_io_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    PCIDevice *pci_dev = (PCIDevice *) opaque;

    UNUSED(d);
    UNUSED(pci_dev);

    printf("IO Write Ordered, addr=%x, value=%lu, size=%d\n", (unsigned) addr, value, size);

    switch (addr) {
        default:
            printf("Io not used\n");        
    }

}

static uint64_t radeon_io_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    printf("IO Read Ordered, addr =%x, size=%d\n", (unsigned) addr, size);

    UNUSED(d);

    switch (addr) {
        default:
            printf("Io not used\n");
            return 0x0;
            
    }
}

static const MemoryRegionOps radeon_vram_ops = {
    .read = radeon_vram_read,
    .write = radeon_vram_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static const MemoryRegionOps radeon_regs_ops = {
    .read = radeon_regs_read,
    .write = radeon_regs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static const MemoryRegionOps radeon_io_ops = {
    .read = radeon_io_read,
    .write = radeon_io_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
};

static void radeon_io_setup(PCIRadeonDevState *d) 
{
    memory_region_init_io(&d->vram, OBJECT(d), &radeon_vram_ops, d, "radeon_vram", RADEON_VRAM_SIZE);
    memory_region_init_io(&d->io, OBJECT(d), &radeon_io_ops, d, "radeon_vram", RADEON_IO_SIZE);
    memory_region_init_io(&d->regs, OBJECT(d), &radeon_regs_ops, d, "radeon_regs", RADEON_REGS_SIZE);
}

static void pci_radeon_dev_init(PCIDevice *pci_dev, Error **errp)
{
    /* init the internal state of the device */
    PCIRadeonDevState *d = PCI_RADEON_DEV(pci_dev);
    printf("d=%lu\n", (unsigned long) &d);
    d->dma_size = 0x1ffff * sizeof(char);
    d->dma_buf = malloc(d->dma_size);
    d->id = 0x4966;
    d->threw_irq = 0;
    uint8_t *pci_conf;

    /* create the memory region representing the MMIO and PIO 
     * of the device
     */
    radeon_io_setup(d);
    /*
     * See linux device driver (Edition 3) for the definition of a bar
     * in the PCI bus.
     */
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->vram);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->io);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->regs);

    //pci_dev->has_rom = true;
    pci_dev->romfile = malloc(1024);

    strcpy(pci_dev->romfile, "ati9000.rom");

    pci_conf = pci_dev->config;
    /* also in ldd, a pci device has 4 pin for interrupt
     * here we use pin B.
     */
    pci_conf[PCI_INTERRUPT_PIN] = 0x02;

    /* this device support interrupt */
    //d->irq = pci_allocate_irq(pci_dev);

    printf("Radeon loaded\n");
}

static void pci_radeon_dev_uninit(PCIDevice *dev)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) dev;
    free(d->dma_buf);
    printf("Good bye World unloaded\n");
}

static void pci_radeon_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    k->realize = pci_radeon_dev_init;
    k->exit = pci_radeon_dev_uninit;
    /* this identify our device */
    k->vendor_id = 0x1002;
    k->device_id = 0x4966;
    k->class_id = PCI_CLASS_DISPLAY_VGA;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    k->revision  = 0x00;
    dc->desc = "PCI RADEON";
    /* qemu user things */
    //dc->props = radeon_properties;
    dc->reset = qdev_pci_radeon_dev_reset;
}

static const TypeInfo pci_radeon_info = {
    .name           = TYPE_PCI_RADEON_DEV,
    .parent         = TYPE_PCI_DEVICE,
    .instance_size  = sizeof(PCIRadeonDevState),
    .class_init     = pci_radeon_class_init,
    .interfaces = (InterfaceInfo[]) {
          { INTERFACE_CONVENTIONAL_PCI_DEVICE },
          { },
    },
};

static void pci_radeon_register_types(void) 
{
    type_register_static(&pci_radeon_info);
}

type_init(pci_radeon_register_types);