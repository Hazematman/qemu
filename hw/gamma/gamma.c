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


typedef struct PCIGammaDevState {
    PCIDevice parent_obj;

    /* for PIO */
    MemoryRegion io;
    /* for MMIO */
    MemoryRegion mmio;
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


} PCIGammaDevState;

#define TYPE_PCI_GAMMA_DEV "pci-gammadev"
#define PCI_GAMMA_DEV(obj)     OBJECT_CHECK(PCIGammaDevState, (obj), TYPE_PCI_GAMMA_DEV)
/* sizes must be power of 2 in PCI */
#define GAMMA_IO_SIZE 1<<4
#define GAMMA_MMIO_SIZE 1<<6

static void qdev_pci_gamma_dev_reset(DeviceState *dev)
{
    printf("Reset World\n");
}

static void gamma_iowrite(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    int i;
    PCIGammaDevState *d = (PCIGammaDevState *) opaque;
    PCIDevice *pci_dev = (PCIDevice *) opaque;

    printf("Write Ordered, addr=%x, value=%lu, size=%d\n", (unsigned) addr, value, size);

    switch (addr) {
        case 0:
            if (value) {
                /* throw an interrupt */
                printf("irq assert\n");
                d->threw_irq = 1;
                pci_irq_assert(pci_dev);

            } else {
                /*  ack interrupt */
                printf("irq deassert\n");
                pci_irq_deassert(pci_dev);
                d->threw_irq = 0;
            }
            break;
        case 4:
            /* throw a random DMA */
            for ( i = 0; i < d->dma_size; ++i)
                d->dma_buf[i] = rand();
            cpu_physical_memory_write(value, (void *) d->dma_buf, d->dma_size);
            break;
        default:
            printf("Io not used\n");
            
    }

}

static uint64_t gamma_ioread(void *opaque, hwaddr addr, unsigned size)
{
    PCIGammaDevState *d = (PCIGammaDevState *) opaque;
    printf("Read Ordered, addr =%x, size=%d\n", (unsigned) addr, size);

    switch (addr) {
        case 0:
            /* irq status */
            return d->threw_irq;
            break;
        default:
            printf("Io not used\n");
            return 0x0;
            
    }
}

static uint64_t gamma_mmioread(void *opaque, hwaddr addr, unsigned size)
{
    PCIGammaDevState *d = (PCIGammaDevState *) opaque;
    printf("MMIO Read Ordered, addr =%x, size=%d\n",(unsigned)  addr, size);

    switch (addr) {
        case 0:
            /* also irq status */   
            printf("irq_status\n");
            return d->threw_irq;
            break;
        case 4:
            /* Id of the device */
            printf("id\n");
            return d->id;
            break;
        default:
            printf("MMIO not used\n");
            return 0x0;
            
    }
}

static void gamma_mmiowrite(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    PCIGammaDevState *d = (PCIGammaDevState *) opaque;
    printf("MMIO write Ordered, addr=%x, value=%lu, size=%d\n",(unsigned)  addr, value, size);

    switch (addr) {
        case 4:
            /* change the id */
            d->id = value;
            break;
        default:
            printf("MMIO not writable or not used\n");
            
    }
}

static const MemoryRegionOps gamma_mmio_ops = {
    .read = gamma_mmioread,
    .write = gamma_mmiowrite,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static const MemoryRegionOps gamma_io_ops = {
    .read = gamma_ioread,
    .write = gamma_iowrite,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void gamma_io_setup(PCIGammaDevState *d) 
{
    memory_region_init_io(&d->mmio, OBJECT(d), &gamma_mmio_ops, d, "gamma_mmio", GAMMA_MMIO_SIZE);
    memory_region_init_io(&d->io, OBJECT(d), &gamma_io_ops, d, "gamma_io", GAMMA_IO_SIZE);
}

static void pci_gamma_dev_init(PCIDevice *pci_dev, Error **errp)
{
    /* init the internal state of the device */
    PCIGammaDevState *d = PCI_GAMMA_DEV(pci_dev);
    printf("d=%lu\n", (unsigned long) &d);
    d->dma_size = 0x1ffff * sizeof(char);
    d->dma_buf = malloc(d->dma_size);
    d->id = 0x1337;
    d->threw_irq = 0;
    uint8_t *pci_conf;

    /* create the memory region representing the MMIO and PIO 
     * of the device
     */
    gamma_io_setup(d);
    /*
     * See linux device driver (Edition 3) for the definition of a bar
     * in the PCI bus.
     */
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &d->io);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);

    pci_conf = pci_dev->config;
    /* also in ldd, a pci device has 4 pin for interrupt
     * here we use pin B.
     */
    pci_conf[PCI_INTERRUPT_PIN] = 0x02; 

    /* this device support interrupt */
    //d->irq = pci_allocate_irq(pci_dev);

    printf("Gamma loaded\n");
}

static void pci_gamma_dev_uninit(PCIDevice *dev)
{
    PCIGammaDevState *d = (PCIGammaDevState *) dev;
    free(d->dma_buf);
    printf("Good bye World unloaded\n");
}

static void pci_hellodev_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    k->realize = pci_gamma_dev_init;
    k->exit = pci_gamma_dev_uninit;
    /* this identify our device */
    k->vendor_id = 0x1337;
    k->device_id = 0x0001;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    k->revision  = 0x00;
    dc->desc = "PCI GAMMA";
    /* qemu user things */
    //dc->props = gamma_properties;
    dc->reset = qdev_pci_gamma_dev_reset;
}

static const TypeInfo pci_gamma_info = {
    .name           = TYPE_PCI_GAMMA_DEV,
    .parent         = TYPE_PCI_DEVICE,
    .instance_size  = sizeof(PCIGammaDevState),
    .class_init     = pci_hellodev_class_init,
    .interfaces = (InterfaceInfo[]) {
          { INTERFACE_CONVENTIONAL_PCI_DEVICE },
          { },
    },
};

static void pci_gamma_register_types(void) 
{
    type_register_static(&pci_gamma_info);
}

type_init(pci_gamma_register_types);