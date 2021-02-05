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

/*****************************************************************/
/* TODO put these in register header or include linux kernel one */
#define RADEON_CONFIG_APER_SIZE             0x0108
#define RADEON_RBBM_STATUS                  0x0e40
#       define RADEON_RBBM_FIFOCNT_MASK     0x007f
#       define RADEON_RBBM_ACTIVE_SHIFT     (1 << 31)

#define RADEON_MEM_STR_CNTL                 0x0150
#       define RADEON_MEM_PWRUP_COMPL_A     (1 <<  0)
#       define RADEON_MEM_PWRUP_COMPL_B     (1 <<  1)
#       define R300_MEM_PWRUP_COMPL_C       (1 <<  2)
#       define R300_MEM_PWRUP_COMPL_D       (1 <<  3)
#       define RADEON_MEM_PWRUP_COMPLETE    0x03
#       define R300_MEM_PWRUP_COMPLETE      0x0f

#define RADEON_SCRATCH_REG0		0x15e0
#define RADEON_SCRATCH_REG1		0x15e4
#define RADEON_SCRATCH_REG2		0x15e8
#define RADEON_SCRATCH_REG3		0x15ec
#define RADEON_SCRATCH_REG4		0x15f0
#define RADEON_SCRATCH_REG5		0x15f4

#define RADEON_HOST_PATH_CNTL               0x0130
#	define RADEON_HP_LIN_RD_CACHE_DIS   (1 << 24)
#	define RADEON_HDP_READ_BUFFER_INVALIDATE   (1 << 27)
#       define RADEON_HDP_SOFT_RESET        (1 << 26)
#       define RADEON_HDP_APER_CNTL         (1 << 23)
#define RADEON_HTOTAL_CNTL                  0x0009 /* PLL */
#       define RADEON_HTOT_CNTL_VGA_EN      (1 << 28)
#define RADEON_HTOTAL2_CNTL                 0x002e /* PLL */

#define RADEON_GEN_INT_CNTL                 0x0040
#	define RADEON_CRTC_VBLANK_MASK		(1 << 0)
#	define RADEON_FP_DETECT_MASK		(1 << 4)
#	define RADEON_CRTC2_VBLANK_MASK		(1 << 9)
#	define RADEON_FP2_DETECT_MASK		(1 << 10)
#	define RADEON_GUI_IDLE_MASK		(1 << 19)
#	define RADEON_SW_INT_ENABLE		(1 << 25)

#define RADEON_AIC_CNTL                     0x01d0
#       define RADEON_PCIGART_TRANSLATE_EN     (1 << 0)
#       define RADEON_DIS_OUT_OF_PCI_GART_ACCESS     (1 << 1)
#	define RS400_MSI_REARM	                (1 << 3) /* rs400/rs480 */
#define RADEON_AIC_LO_ADDR                  0x01dc
#define RADEON_AIC_PT_BASE		0x01d8
#define RADEON_AIC_HI_ADDR		0x01e0

#define RADEON_CP_RB_BASE                   0x0700
#define RADEON_CP_RB_CNTL                   0x0704
#	define RADEON_RB_BUFSZ_SHIFT		0
#	define RADEON_RB_BUFSZ_MASK		(0x3f << 0)
#	define RADEON_RB_BLKSZ_SHIFT		8
#	define RADEON_RB_BLKSZ_MASK		(0x3f << 8)
#	define RADEON_BUF_SWAP_32BIT		(2 << 16)
#	define RADEON_MAX_FETCH_SHIFT		18
#	define RADEON_MAX_FETCH_MASK		(0x3 << 18)
#	define RADEON_RB_NO_UPDATE		(1 << 27)
#	define RADEON_RB_RPTR_WR_ENA		(1 << 31)
#define RADEON_CP_RB_RPTR_ADDR              0x070c
#define RADEON_CP_RB_RPTR                   0x0710
#define RADEON_CP_RB_WPTR                   0x0714
#define RADEON_CP_RB_RPTR_WR                0x071c

#define RADEON_PCI_GART_PAGE                0x017c

#define RADEON_CP_IB_BASE                   0x0738
#define RADEON_CP_IB_BUFSZ                  0x073c

/***********************************************************************/

/*****************************************************************/
/* RADEON CP defines */

#define CP_PACKET_MASK      0xC0000000
#define CP_PACKET0			0x00000000
#define		PACKET0_BASE_INDEX_SHIFT	0
#define		PACKET0_BASE_INDEX_MASK		(0xffff << 0)
#define		PACKET0_COUNT_SHIFT		16
#define		PACKET0_COUNT_MASK		(0x3fff << 16)
#define CP_PACKET1			0x40000000
#define CP_PACKET2			0x80000000
#define		PACKET2_PAD_SHIFT		0
#define		PACKET2_PAD_MASK		(0x3fffffff << 0)
#define CP_PACKET3			0xC0000000
#define		PACKET3_IT_OPCODE_SHIFT		8
#define		PACKET3_IT_OPCODE_MASK		(0xff << 8)
#define		PACKET3_COUNT_SHIFT		16
#define		PACKET3_COUNT_MASK		(0x3fff << 16)

/*****************************************************************/

#define UNUSED(x) (void)(x)

/* sizes must be power of 2 in PCI */
#define RADEON_REGS_SIZE (64*1024)
#define RADEON_VRAM_SIZE (128*1024*1024)
#define RADEON_IO_SIZE   (256)

#define RADEON_PAGE_SIZE 4096
#define RADEON_PAGE_SHIFT 12

#define RADEON_CB_SIZE (16)
#define RADEON_PACKET_SIZE (RADEON_CB_SIZE*4)


typedef struct PCIRadeonDevState {
    PCIDevice parent_obj;

    QemuThread thread;

    /* for VRAM */
    MemoryRegion vram;
    /* for IO */
    MemoryRegion io;
    /* for REGS */
    MemoryRegion regs;

    /* irq used */
    qemu_irq irq;
    /* did we throw an interrupt ? */
    int threw_irq;
    /* id of the device, writable */
    int id;

    int do_dma;

    uint32_t cmds[RADEON_CB_SIZE];

    struct {
        uint32_t radeon_mem_str_cntl;
        uint32_t radeon_rbbm_status;
        uint32_t radeon_gen_int_cntl;
        uint32_t radeon_aic_cntl;
        uint32_t radeon_lo_addr;
        uint32_t radeon_hi_addr;
        uint32_t radeon_pt_base;
        dma_addr_t radeon_cp_base;
        uint32_t radeon_cp_wptr;
        uint32_t radeon_cp_rptr;
        uint32_t radeon_gart_page; // TODO this might not be right

        uint32_t radeon_ib_base;
        uint32_t radeon_ib_size;

        uint32_t radeon_scratch_reg_0;
        uint32_t radeon_scratch_reg_1;
        uint32_t radeon_scratch_reg_2;
        uint32_t radeon_scratch_reg_3;
        uint32_t radeon_scratch_reg_4;
        uint32_t radeon_scratch_reg_5;
    } dev_regs;


} PCIRadeonDevState;

#define TYPE_PCI_RADEON_DEV "pci-radeondev"
#define PCI_RADEON_DEV(obj)     OBJECT_CHECK(PCIRadeonDevState, (obj), TYPE_PCI_RADEON_DEV)

static uint8_t vram[RADEON_VRAM_SIZE];

static void qdev_pci_radeon_dev_reset(DeviceState *dev)
{
    printf("Reset World\n");
}

static void radeon_print_regs(PCIRadeonDevState *d)
{
    printf("read 0x%x write 0x%x\n", qatomic_read(&d->dev_regs.radeon_cp_rptr),
                                    qatomic_read(&d->dev_regs.radeon_cp_wptr));
}

/* Used for debugging */
static void radeon_dump_gart_table(PCIRadeonDevState *d)
{
    static uint32_t gart_entires[131072];

    int result = pci_dma_read((PCIDevice *)d, (dma_addr_t)(d->dev_regs.radeon_pt_base), &gart_entires[0], sizeof(gart_entires));
    printf("RADEON READ TABLE RESULT %d\n", result);

    for(int i=0; i < 131072; i++)
    {
        printf("%d: 0x%x\n", i, gart_entires[i]);
    }
    printf("END GART TABLE\n");

}

static uint32_t radeon_translate_addr(PCIRadeonDevState *d, uint32_t addr)
{
    // TODO I'm not sure if this address translation works in general
    // Hardcodes offset as what is used to look up the page
    uint32_t offset = addr - d->dev_regs.radeon_lo_addr;
    uint32_t gpu_page = (offset >> RADEON_PAGE_SHIFT);
    uint32_t gart_lookup_addr = d->dev_regs.radeon_pt_base + gpu_page*4;
    uint32_t gart_addr;
    int result = pci_dma_read((PCIDevice *)d, (dma_addr_t)(gart_lookup_addr), &gart_addr, sizeof(gart_addr));
    printf("Gart translation result %d %d 0x%x (gart page 0x%x)\n", result, gpu_page, gart_addr, d->dev_regs.radeon_gart_page);
    return gart_addr + (offset % RADEON_PAGE_SIZE);
}

static void radeon_dma_command_thread(PCIRadeonDevState *d)
{
    PCIDevice *dev = (PCIDevice *)d;
    UNUSED(dev);
    static uint32_t buffer[128];
    dma_addr_t addr = radeon_translate_addr(d, addr);
    size_t dma_size = d->dev_regs.radeon_cp_wptr*4;
    pci_dma_read((PCIDevice *)d, (dma_addr_t)(addr), (void *)&buffer[0], (dma_addr_t)dma_size);

#if 0
    printf("RADEON AIC 0x%x 0x%x 0x%x\n", d->dev_regs.radeon_lo_addr, d->dev_regs.radeon_hi_addr, d->dev_regs.radeon_pt_base);
    printf("RADEON DMA buffer from addr 0x%lx (0x%lx)(%d)\n", d->dev_regs.radeon_cp_base, addr, result);
    for(int i=0; i<dma_size/4; i++)
    {
        printf("\t0x%x\n", buffer[i]);
    }
    printf("DONE RADEON DMA BUFFER\n");
#endif
    UNUSED(radeon_dump_gart_table);
    return;
}

static void radeon_dma_command(PCIRadeonDevState *d)
{
    //qatomic_set(&d->do_dma, 1);
    //radeon_dma_command_thread(d);
}

static void radeon_write_reg(PCIRadeonDevState *d, uint32_t addr, uint32_t val)
{
    UNUSED(radeon_dma_command);

    switch (addr) {
        case RADEON_GEN_INT_CNTL:
            /* TODO figure out how to actually use interrupts */
            d->dev_regs.radeon_mem_str_cntl = val;
            break;
        case RADEON_AIC_CNTL:
            d->dev_regs.radeon_aic_cntl = val;
            break;
        case RADEON_AIC_LO_ADDR:
            d->dev_regs.radeon_lo_addr = val;
            break;
        case RADEON_AIC_HI_ADDR:
            d->dev_regs.radeon_hi_addr = val;
            break;
        case RADEON_AIC_PT_BASE:
            d->dev_regs.radeon_pt_base = val;
            break;
        case RADEON_CP_RB_CNTL:
            printf("RADEON Wrote RB status 0x%x\n", val);
            break;
        case RADEON_CP_RB_BASE:
            d->dev_regs.radeon_cp_base = val;
            break;
        case RADEON_CP_RB_WPTR:
            d->dev_regs.radeon_cp_wptr = val;
            radeon_print_regs(d);
            break;
        case RADEON_PCI_GART_PAGE:
            d->dev_regs.radeon_gart_page = val;
            break;
        case RADEON_CP_IB_BASE:
            d->dev_regs.radeon_ib_base = val;
            break;
        case RADEON_CP_IB_BUFSZ:
            d->dev_regs.radeon_ib_size = val;
            break;
        case RADEON_SCRATCH_REG0:
            d->dev_regs.radeon_scratch_reg_0 = val;
            break;
        case RADEON_SCRATCH_REG1:
            d->dev_regs.radeon_scratch_reg_1 = val;
            break;
        case RADEON_SCRATCH_REG2:
            d->dev_regs.radeon_scratch_reg_2 = val;
            break;
        case RADEON_SCRATCH_REG3:
            d->dev_regs.radeon_scratch_reg_3 = val;
            break;
        case RADEON_SCRATCH_REG4:
            d->dev_regs.radeon_scratch_reg_4 = val;
            break;
        case RADEON_SCRATCH_REG5:
            d->dev_regs.radeon_scratch_reg_5 = val;
            break;
        default:
            printf("WRITE IO not used addr=%x, value=%0x\n", (unsigned) addr, val);
            break;
     
    }  
}

static void radeon_process_cb(PCIRadeonDevState *d, uint32_t *buf, size_t size)
{
    static int packet_type = -1;
    static uint32_t count;
    static uint32_t cur_reg;

    for(int i=0; i < size; i++)
    {
        uint32_t val = buf[i];
        if(packet_type == -1)
        {
            switch(val & CP_PACKET_MASK)
            {
                case CP_PACKET0:
                    cur_reg = (val & PACKET0_BASE_INDEX_MASK) << 2;
                    count = ((val & PACKET0_COUNT_MASK) >> PACKET0_COUNT_SHIFT)+1;
                    packet_type = 0;
                    printf("CP0 on 0x%x\n", cur_reg);
                    break;
                case CP_PACKET1:
                    printf("Got CP_PACKET1 don't know how to handle this\n");
                    break;
                case CP_PACKET2:
                    /* This is a NOP */
                    break;
                case CP_PACKET3:
                    printf("Got CP_PACKET3 don't know how to handle this\n");
                    break;
            }
        }
        else if(packet_type == 0)
        {
            radeon_write_reg(d, cur_reg, val);
            uint32_t old_reg = cur_reg;
            cur_reg += 4;
            count -= 1;

            if(count == 0)
            {
                packet_type = -1;
            }

            /* If we are writing to RADEON_CP_IB_BUFSZ then we can start */
            /* executing the indrect buffer */
            if(old_reg == RADEON_CP_IB_BUFSZ)
            {
                uint32_t ib_size = (val << 2);
                uint32_t *ib_cmds = malloc(ib_size);
                uint32_t addr = radeon_translate_addr(d, qatomic_read(&d->dev_regs.radeon_ib_base));
                pci_dma_read((PCIDevice *)d, (dma_addr_t)(addr), (void *)ib_cmds, (ib_size));

                printf("RADEON IB \n");
                for(int i=0; i < val; i++)
                {
                    printf("\t0x%x\n", ib_cmds[i]);
                }

                /* TODO how to make this function recursive... */
                radeon_process_cb(d, ib_cmds, val);

                free(ib_cmds);
            }
        }
    }
}

static void *radeon_cp_thread(void *opaque)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    PCIDevice *pci_dev = (PCIDevice *) opaque;
    bool processing = false;

    UNUSED(pci_dev);
    UNUSED(radeon_dma_command_thread);
    while(true)
    {
        /* If we have started a DMA */
        if(!processing && qatomic_read(&d->dev_regs.radeon_cp_wptr) != qatomic_read(&d->dev_regs.radeon_cp_rptr))
        { 
            //radeon_dma_command_thread(d);
            /* Read 16 words from the command buffer */
            uint32_t cmd_buffer_size = qatomic_read(&d->dev_regs.radeon_cp_wptr) - qatomic_read(&d->dev_regs.radeon_cp_rptr);
            if((cmd_buffer_size % RADEON_CB_SIZE) == 0)
            {
                uint32_t addr = radeon_translate_addr(d, qatomic_read(&d->dev_regs.radeon_cp_base)+qatomic_read(&d->dev_regs.radeon_cp_rptr)*4);
                pci_dma_read((PCIDevice *)d, (dma_addr_t)(addr), (void *)&d->cmds[0], (dma_addr_t)RADEON_PACKET_SIZE);
                processing = true;
                qatomic_set(&d->dev_regs.radeon_cp_rptr, qatomic_read(&d->dev_regs.radeon_cp_rptr)+RADEON_CB_SIZE);
                printf("Do radeon CMD\n");
            }
            else
            {
                printf("CMD is not on RADEON_CB_SIZE word offset\n");
            }
        }

        if(processing)
        {
            printf("RADEON CMD BUF\n");
            for(int i=0; i < RADEON_CB_SIZE; i++)
            {
                printf("\t0x%x\n", d->cmds[i]);
            }
            radeon_process_cb(d, d->cmds, RADEON_CB_SIZE);
            processing = false;

        }
    }

    return NULL;
}

static void radeon_regs_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    PCIDevice *pci_dev = (PCIDevice *) opaque;

    UNUSED(d);
    UNUSED(pci_dev);

    radeon_write_reg(d, addr, value);
}

static uint64_t radeon_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) opaque;
    //printf("REGS Read Ordered, addr =%x, size=%d\n", (unsigned) addr, size);

    UNUSED(d);

    switch (addr) {
        case RADEON_CONFIG_APER_SIZE:
            return RADEON_VRAM_SIZE;
            break;
        case RADEON_RBBM_STATUS:
            return d->dev_regs.radeon_rbbm_status;
            break;
        case RADEON_MEM_STR_CNTL:
            return d->dev_regs.radeon_mem_str_cntl;
            break;
        case RADEON_HOST_PATH_CNTL:
            /* No idea how this register works, `r100_get_accessible_vram` */
            /* tries to read it to determine if the aper size needs to be */
            /* multiplied by 2, leave it as 0x0 for now so that the radeon */
            /* driver treats the aper size to just be `RADEON_CONFIG_APER_SIZE` */
            /* Which will be equal to RADEON_VRAM_SIZE in bytes */ 
            return 0x0;
            break;
        case RADEON_AIC_CNTL:
            return d->dev_regs.radeon_aic_cntl;
            break;
        case RADEON_GEN_INT_CNTL:
            /* TODO figure out how to actually use interrupts */
            return d->dev_regs.radeon_mem_str_cntl;
            break;
        case RADEON_SCRATCH_REG0:
            return d->dev_regs.radeon_scratch_reg_0;
            break;
        case RADEON_SCRATCH_REG1:
            return d->dev_regs.radeon_scratch_reg_1;
            break;
        case RADEON_SCRATCH_REG2:
            return d->dev_regs.radeon_scratch_reg_2;
            break;
        case RADEON_SCRATCH_REG3:
            return d->dev_regs.radeon_scratch_reg_3;
            break;
        case RADEON_SCRATCH_REG4:
            return d->dev_regs.radeon_scratch_reg_4;
            break;
        case RADEON_SCRATCH_REG5:
            return d->dev_regs.radeon_scratch_reg_5;
            break;
        default:
            printf("READ IO not used addr =%x, size=%d\n", (unsigned) addr, size);
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

static void radeon_init_regs(PCIRadeonDevState *d)
{
    /* Initalize registers to their default state */
    d->dev_regs.radeon_rbbm_status = 0;
    d->dev_regs.radeon_mem_str_cntl = RADEON_MEM_PWRUP_COMPL_A | RADEON_MEM_PWRUP_COMPL_B | RADEON_MEM_PWRUP_COMPLETE;
    d->dev_regs.radeon_mem_str_cntl = 0;
}

static void radeon_io_setup(PCIRadeonDevState *d) 
{
    memory_region_init_io(&d->vram, OBJECT(d), &radeon_vram_ops, d, "radeon_vram", RADEON_VRAM_SIZE);
    memory_region_init_io(&d->io, OBJECT(d), &radeon_io_ops, d, "radeon_io", RADEON_IO_SIZE);
    memory_region_init_io(&d->regs, OBJECT(d), &radeon_regs_ops, d, "radeon_regs", RADEON_REGS_SIZE);
}

static void pci_radeon_dev_init(PCIDevice *pci_dev, Error **errp)
{
    /* init the internal state of the device */
    PCIRadeonDevState *d = PCI_RADEON_DEV(pci_dev);
    printf("d=%lu\n", (unsigned long) &d);
    d->id = 0x4966;
    d->threw_irq = 0;
    uint8_t *pci_conf;

    radeon_init_regs(d);

    /* create the memory region representing the MMIO and PIO 
     * of the device
     */
    radeon_io_setup(d);
    /*
     * See linux device driver (Edition 3) for the definition of a bar
     * in the PCI bus.
     */
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_PREFETCH, &d->vram);
    pci_register_bar(pci_dev, 1, PCI_BASE_ADDRESS_SPACE_IO, &d->io);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->regs);

    strcpy(pci_dev->name, "radeon9000");
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

    qemu_thread_create(&d->thread, "radeon-thread", radeon_cp_thread,
                       d, QEMU_THREAD_JOINABLE);


    printf("Radeon loaded\n");
}

static void pci_radeon_dev_uninit(PCIDevice *dev)
{
    PCIRadeonDevState *d = (PCIRadeonDevState *) dev;
    qemu_thread_join(&d->thread);
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
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);

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