// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2007 Oxford Semiconductor Ltd.
 *
 * A driver to interface the 934 based sata core present in OX810SE
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/libata.h>

#include <scsi/scsi_host.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_device.h>
#include <linux/io.h>

#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/reset.h>
#include <linux/gpio/consumer.h>
#include <linux/version.h>

#include "libata.h"

#define DRIVER_AUTHOR "Oxford Semiconductor Ltd."
#define DRIVER_DESC "934 SATA core controller"
#define DRIVER_NAME "oxnas-sata"

#define SCR2LINK(x) (0x20 + ((x) * 4))
#define NULLPTR(x) ((u32)(x) < PAGE_SIZE)

/* Delays/waits */
enum {
	ATOMIC_DELAY_US = 10,
	IDLE_WAIT_MS = 1000,
	IDLE_MAXIMUM_WAIT_MS = 3000,
	CORE_RESET_DELAY_MS = 10,
	SRST_WAIT_MS = 3000,
	ABORT_WAIT_MS = 5000,
};

/* Controller information */
enum {
	SATA_OXNAS_MAX_PRD = 63 /*254*/,
	SATA_OXNAS_MAX_PORTS = 2,
	/* The different Oxsemi SATA core version numbers */
	SATA_OXNAS_CORE_VERSION = 0x1F2,
	SATA_OXNAS_IRQ_FLAG = 0,
	SATA_OXNAS_HOST_FLAGS = (ATA_FLAG_SATA | ATA_FLAG_PIO_DMA |
			ATA_FLAG_NO_ATAPI /*| ATA_FLAG_NCQ*/),
	SATA_OXNAS_QUEUE_DEPTH = 32,

	SATA_OXNAS_DMA_BOUNDARY = 0xFFFFFFFF,

	PORT_SIZE = 0x10000,
	CORE_BASE = 0xE0000,
	RAID_BASE = 0xF0000,
	SATA_SIZE = PORT_SIZE * 0x10,
	SATA_DATA_SIZE = 0x1000000,
};

/*
 * SATA Port Registers
 */
enum {
	/* sata host port register offsets */
	ORB1 = 0x00,
	ORB2 = 0x04,
	ORB3 = 0x08,
	ORB4 = 0x0C,
	ORB5 = 0x10,
	MASTER_STATUS = 0x10,
	FIS_CTRL = 0x18,
	FIS_DATA = 0x1C,
	INT_STATUS = 0x30,
	INT_CLEAR = 0x30,
	INT_ENABLE = 0x34,
	INT_DISABLE = 0x38,
	SATA_VERSION = 0x3C,
	SATA_CONTROL = 0x5C,
	SATA_COMMAND = 0x60,
	HID_FEATURES = 0x64,
	PORT_CONTROL = 0x68,
	DRIVE_CONTROL = 0x6C,
	/* These registers allow access to the link layer registers
	 * that reside in a different clock domain to the processor bus
	 */
	LINK_DATA = 0x70,
	LINK_RD_ADDR = 0x74,
	LINK_WR_ADDR = 0x78,
	LINK_CONTROL = 0x7C,
};

/* sata port register bits */
enum {
	/*
	 * commands to issue in the master status to tell it to move shadow ,
	 * registers to the actual device ,
	 */
	SATA_OPCODE_MASK = 0x00000007,
	CMD_WRITE_TO_ORB_REGS_NO_COMMAND = 0x4,
	CMD_WRITE_TO_ORB_REGS = 0x2,
	CMD_SYNC_ESCAPE = 0x7,
	CMD_CORE_BUSY = BIT(7),
	CMD_DRIVE_SELECT_SHIFT = 12,
	CMD_DRIVE_SELECT_MASK = (0xf << CMD_DRIVE_SELECT_SHIFT),

	/* interrupt bits */
	INT_END_OF_CMD = BIT(0),
	INT_LINK_SERROR = BIT(1),
	INT_ERROR = BIT(2),
	INT_LINK_IRQ = BIT(3),
	INT_REG_ACCESS_ERR = BIT(7),
	INT_BIST_FIS = BIT(11),
	INT_MASKABLE = INT_END_OF_CMD | INT_LINK_SERROR | INT_ERROR |
		INT_LINK_IRQ | INT_REG_ACCESS_ERR | INT_BIST_FIS,
	INT_WANT = INT_END_OF_CMD | INT_LINK_SERROR | INT_REG_ACCESS_ERR |
		INT_ERROR,
	INT_ERRORS = INT_LINK_SERROR | INT_REG_ACCESS_ERR | INT_ERROR,
	INT_USED = INT_WANT, //INT_MASKABLE,//

	/* raw interrupt bits, unmaskable, but do not generate interrupts */
	RAW_END_OF_CMD = INT_END_OF_CMD << 16,
	RAW_LINK_SERROR = INT_LINK_SERROR << 16,
	RAW_ERROR = INT_ERROR << 16,
	RAW_LINK_IRQ = INT_LINK_IRQ << 16,
	RAW_REG_ACCESS_ERR = INT_REG_ACCESS_ERR << 16,
	RAW_BIST_FIS = INT_BIST_FIS << 16,
	RAW_WANT = INT_WANT << 16,
	RAW_ERRORS = INT_ERRORS << 16,

	/*
	 * variables to write to the device control register to set the current
	 * device, ie. master or slave.
	 */
	DR_CON_48 = 2,
	DR_CON_28 = 0,

	SATA_CTL_ERR_MASK = 0x00000016,
};

/* SATA core register offsets */
enum {
	DM_DBG1 = 0x000,
	RAID_SET = 0x004,
	DM_DBG2 = 0x008,
	DATACOUNT_PORT0 = 0x010,
	DATACOUNT_PORT1 = 0x014,
	CORE_INT_STATUS = 0x030,
	CORE_INT_CLEAR = 0x030,
	CORE_INT_ENABLE = 0x034,
	CORE_INT_DISABLE = 0x038,
	CORE_REBUILD_ENABLE = 0x050,
	CORE_FAILED_PORT_R = 0x054,
	DEVICE_CONTROL = 0x068,
	EXCESS = 0x06C,
	PORT_ERROR_MASK = 0x078,
	IDLE_STATUS = 0x07C,
	RAID_CONTROL = 0x090,
	PROC_PC = 0x100,
	CONFIG_IN = 0x3d8,
	PROC_START = 0x3f0,
	PROC_RESET = 0x3f4,
	DATA_MUX_RAM0 = 0x8000,
	DATA_MUX_RAM1 = 0xA000,
};

enum {
	/* Sata core debug1 register bits */
	CORE_PORT0_DATA_DIR_BIT = 20,
	CORE_PORT1_DATA_DIR_BIT = 21,
	CORE_PORT0_DATA_DIR = BIT(CORE_PORT0_DATA_DIR_BIT),
	CORE_PORT1_DATA_DIR = BIT(CORE_PORT1_DATA_DIR_BIT),

	/* sata core control register bits */
	SCTL_CLR_ERR = 0x00003016,
	RAID_CLR_ERR = 0x0000011e,

	/* Interrupts direct from the ports */
	NORMAL_INTS_WANTED = 0x00000303,

	/* shift these left by port number */
	COREINT_HOST = 0x00000001,
	COREINT_END = 0x00000100,
	CORERAW_HOST = COREINT_HOST << 16,
	CORERAW_END = COREINT_END << 16,

	/* Interrupts from the RAID controller only */
	RAID_INTS_WANTED = 0x00008300,

	/* The bits in the IDLE_STATUS that, when set indicate an idle core */
	IDLE_CORES = BIT(18) | BIT(19),

	/* Device Control register bits */
	DEVICE_CONTROL_DMABT = BIT(4),
	DEVICE_CONTROL_ABORT = BIT(2),
	DEVICE_CONTROL_PAD = BIT(3),
	DEVICE_CONTROL_PADPAT = BIT(16),
	DEVICE_CONTROL_PRTRST = BIT(8),
	DEVICE_CONTROL_RAMRST = BIT(12),
	DEVICE_CONTROL_ATA_ERR_OVERRIDE = BIT(28),

	/* standard HW raid flags */
	OXNASSATA_NOTRAID = 3,
	OXNASSATA_RAID1 = 1,
	OXNASSATA_RAID0 = 0,
	OXNASSATA_RAID_TWODISKS = 3,
};

enum {
	STAT_READ_VALID = BIT(21),
	STAT_CR_ACK = BIT(20),
	STAT_CR_READ = BIT(19),
	STAT_CR_WRITE = BIT(18),
	STAT_CAP_DATA = BIT(17),
	STAT_CAP_ADDR = BIT(16),

	STAT_ACK_ANY = STAT_CR_ACK | STAT_CR_READ | STAT_CR_WRITE |
		STAT_CAP_DATA | STAT_CAP_ADDR,

	CR_READ_ENABLE = BIT(16),
	CR_WRITE_ENABLE = BIT(17),
	CR_CAP_DATA = BIT(18),
};

enum {
	/* Link layer registers */
	SERROR_IRQ_MASK = 5,
};

/*
 * Structs to hold host and per-port private (specific to this driver) datas
 */
struct ox810sata_port_priv {
	struct ata_port *port;
	struct ata_queued_cmd *active_qc;

	// TODO comment
	spinlock_t scrlock;
};

struct ox810sata_host_priv {
	struct ata_host *host;
	struct ata_port *active_ap;
	bool hw_raid_active;
	void __iomem *iomap;
	dma_addr_t data_phys;
	struct platform_device *pdev;
	struct clk *clk;
	struct reset_control *rst_sata;
	struct reset_control *rst_link;
	struct reset_control *rst_phy;
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *desc;
};

/* TODO remove this from global */
static struct ox810sata_host_priv ox810sata_hd = { 0 };

/*
 * TODO remove that until HW-RAID support is added
 * A record of which drives have accumulated raid faults. A set bit indicates
 * a fault has occurred on that drive
 */
static u32 ox810sata_accumulated_RAID_faults;

static int ox810sata_port_no(struct ata_port *const ap)
{
	if (NULLPTR(ap))
		return 0;

	return ox810sata_hd.host->ports[0] == ap ? 0 : 1;
}

static int ox810sata_other_port_no(struct ata_port *const ap)
{
	if (NULLPTR(ap))
		return 1;

	return ox810sata_hd.host->ports[0] == ap ? 1 : 0;
}

/*
 * Gets the base address of the ata core from the ata_port structure. The value
 * returned will remain the same when hardware raid is active.
 *
 * @param ap pointer to the appropriate ata_port structure
 * @return the base address of the SATA core
 */
static void __iomem *ox810sata_iocore(void)
{
	return ox810sata_hd.iomap + CORE_BASE;
}

static void __iomem *ox810sata_ioport(struct ata_port *const ap)
{
	return ox810sata_hd.iomap + ox810sata_port_no(ap) * PORT_SIZE;
}

/*
 * Gets the base of the ox810 port associated with the ata-port as known
 * by lib-ata, The value returned changes to the single RAID port when
 * hardware RAID commands are active.
 *
 * @param ap pointer to the appropriate ata_port structure
 * @return the base address of the SATA core
 */
static void __iomem *ox810sata_ioportraid(struct ata_port *const ap)
{
	if (ox810sata_hd.hw_raid_active && ox810sata_hd.active_ap == ap)
		return ox810sata_hd.iomap + RAID_BASE;

	return ox810sata_ioport(ap);
}

static inline u32 ox810sata_io_read(void __iomem *io)
{
	return readl(io);
}

static inline u32 ox810sata_iocore_read(unsigned int reg)
{
	return ox810sata_io_read(ox810sata_iocore() + reg);
}

static inline u32 ox810sata_ioport_read(struct ata_port *const ap,
					unsigned int reg)
{
	return ox810sata_io_read(ox810sata_ioport(ap) + reg);
}

static inline u32 ox810sata_ioportraid_read(struct ata_port *const ap,
					    unsigned int reg)
{
	return ox810sata_io_read(ox810sata_ioportraid(ap) + reg);
}

static inline void ox810sata_io_andor(void __iomem *io, u32 andmask, u32 orval)
{
	if (andmask)
		writel((readl(io) & andmask) | orval, io);
	else
		writel(orval, io);

	// TODO comment
	wmb();
}

static inline void ox810sata_iocore_andor(unsigned int reg, u32 andmask,
					  u32 orval)
{
	ox810sata_io_andor(ox810sata_iocore() + reg, andmask, orval);
}

static inline void ox810sata_ioport_andor(struct ata_port *const ap,
					  unsigned int reg, u32 andmask,
					  u32 orval)
{
	ox810sata_io_andor(ox810sata_ioport(ap) + reg, andmask, orval);
}

static inline void ox810sata_ioportraid_andor(struct ata_port *const ap,
					      unsigned int reg, u32 andmask,
					      u32 orval)
{
	ox810sata_io_andor(ox810sata_ioportraid(ap) + reg, andmask, orval);
}

/*
 * Resetting core and clock helpers
 */
static void ox810sata_reset_assert(void)
{
	reset_control_assert(ox810sata_hd.rst_sata);
	reset_control_assert(ox810sata_hd.rst_link);
	reset_control_assert(ox810sata_hd.rst_phy);
}

static void ox810sata_reset_deassert(void)
{
	reset_control_deassert(ox810sata_hd.rst_phy);

	mdelay(1);

	reset_control_deassert(ox810sata_hd.rst_link);

	mdelay(1);

	reset_control_deassert(ox810sata_hd.rst_sata);

	mdelay(1);
}

static void ox810sata_clock_disable(void)
{
	clk_disable_unprepare(ox810sata_hd.clk);
}

static void ox810sata_clock_enable(void)
{
	clk_prepare_enable(ox810sata_hd.clk);
}

static struct ata_port *ox810sata_other_ap(struct ata_port *const ap)
{
	if (!NULLPTR(ap)) {
		int port_no = ox810sata_hd.host->n_ports > 1 ?
				      ox810sata_other_port_no(ap) :
				      0;

		return ox810sata_hd.host->ports[port_no];
	}

	return NULL;
}

static void ox810sata_dma_abort(void)
{
	const u32 mask = DEVICE_CONTROL_DMABT | DEVICE_CONTROL_ABORT;

	ox810sata_iocore_andor(DEVICE_CONTROL, ~0, mask);
	mdelay(1);
	ox810sata_iocore_andor(DEVICE_CONTROL, ~mask, 0);

	if (!NULLPTR(ox810sata_hd.chan)) {
		if (!NULLPTR(ox810sata_hd.desc)) {
			ox810sata_hd.desc->callback = NULL;
			if (dmaengine_tx_status(ox810sata_hd.chan,
						ox810sata_hd.desc->cookie,
						NULL) == DMA_IN_PROGRESS)
				dmaengine_terminate_async(ox810sata_hd.chan);
			ox810sata_hd.desc = NULL;
		}
	}
}

static struct ata_queued_cmd *ox810sata_qc_from_tag(struct ata_port *ap,
						    unsigned int tag)
{
	struct ata_queued_cmd *qc = NULL;

	if (!NULLPTR(ap)) {
		if (ata_tag_valid(tag) && !(ap->pflags & ATA_PFLAG_FROZEN)) {
			qc = ata_qc_from_tag(ap, tag);
			if (!NULLPTR(qc) && qc->tag != ap->link.active_tag)
				qc = NULL;
		}
	}

	return qc;
}

static struct ata_queued_cmd *ox810sata_active_qc(struct ata_port *const ap)
{
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;
	struct ata_queued_cmd *qc0 =
		NULLPTR(ap) ? NULL :
			      ox810sata_qc_from_tag(ap, ap->link.active_tag);
	struct ata_queued_cmd *qc = NULLPTR(pd) ? NULL : pd->active_qc;

	if (!NULLPTR(qc)) {
		if (!(qc->flags & ATA_QCFLAG_ACTIVE)) {
			qc = NULL;
		} else {
			if (qc->ap != ap) {
				ata_port_err(ap, "data integrity error\n");
				qc = NULL;
			}
		}
	}
	if (!NULLPTR(qc) && !NULLPTR(qc0) && qc != qc0)
		ata_port_warn(ap, "issued and active qcs not match\n");

	return qc;
}

static void ox810sata_send_control_fis(struct ata_port *const ap, const u32 cmd)
{
	ox810sata_ioportraid_andor(ap, SATA_COMMAND, ~SATA_OPCODE_MASK, cmd);
}

/* clears errors */
static void ox810sata_cs_error_clear(struct ata_port *const ap)
{
	if (!NULLPTR(ap))
		ox810sata_ioportraid_andor(ap, SATA_COMMAND, SATA_CTL_ERR_MASK,
					   0);
}

static void ox810sata_sctl_error_clear(struct ata_port *const ap)
{
	if (ox810sata_hd.hw_raid_active)
		ox810sata_ioportraid_andor(ap, SATA_CONTROL, RAID_CLR_ERR, 0);
	if (!NULLPTR(ap))
		ox810sata_ioportraid_andor(ap, SATA_CONTROL, SCTL_CLR_ERR, 0);
}

/*
 * Clears the error caused by the core's registers being accessed when the
 * core is busy.
 */
static inline void ox810sata_reg_access_error_clear(struct ata_port *ap)
{
	if (!NULLPTR(ap))
		ox810sata_ioportraid_andor(ap, INT_STATUS, INT_REG_ACCESS_ERR,
					   0);
}

/*
 * ox810sata_irq_clear is called during probe just before the interrupt handler is
 * registered, to be sure hardware is quiet. It clears and masks interrupt bits
 * in the SATA core.
 *
 * @param ap hardware with the registers in
 */
static void ox810sata_irq_clear(struct ata_port *const ap)
{
	ox810sata_ioportraid_andor(ap, INT_CLEAR, 0, INT_USED);
}

/*
 * turn off the interrupts from the ata drive
 * clear any pending interrupts.
 *
 * @param ap Hardware with the registers in
 */
static void ox810sata_irq_off(struct ata_port *const ap)
{
	// disable End of command interrupt
	ox810sata_ioportraid_andor(ap, INT_DISABLE, 0, INT_USED);

	// Clear pending interrupts
	ox810sata_irq_clear(ap);
}

/*
 * turn on the interrupts from the ata drive
 * wait for idle, clear any pending interrupts.
 *
 * @param ap Hardware with the registers in
 */
static void ox810sata_irq_on(struct ata_port *const ap)
{
	// Clear pending interrupts
	ox810sata_irq_clear(ap);

	if (ox810sata_hd.hw_raid_active) {
		// set interrupt mode for raid controller interrupts only
		ox810sata_iocore_andor(CORE_INT_ENABLE, 0, RAID_INTS_WANTED);
	} else {
		// set normal interrupt scheme
		ox810sata_iocore_andor(CORE_INT_ENABLE, 0, NORMAL_INTS_WANTED);
	}

	// enable End of command interrupt
	ox810sata_ioportraid_andor(ap, INT_ENABLE, 0, INT_USED);
}

static void ox810sata_link_wait_ready(struct ata_port *const ap)
{
	int patience;

	for (patience = 0x1000000; patience > 0; patience--) {
		if (ox810sata_ioport_read(ap, LINK_CONTROL) & 1UL)
			break;
	}
}

/*
 * allows access to the link layer registers
 * @param link_reg the link layer register to access (oxsemi indexing ie
 *        00 = static config, 04 = phy ctrl)
 */
static u32 ox810sata_link_read(struct ata_port *ap, unsigned int link_reg,
			       spinlock_t *lock)
{
	u32 result;
	unsigned long flags = 0;

	if (!NULLPTR(lock) && !in_irq())
		spin_lock_irqsave(lock, flags);

	ox810sata_link_wait_ready(ap);

	/* accessed twice as a workaround for a bug in the SATA abp bridge
	 * hardware (bug 6828)
	 */
	ox810sata_ioport_andor(ap, LINK_RD_ADDR, 0, link_reg);

	// TODO comment
	wmb();
	(void)ox810sata_ioport_read(ap, LINK_RD_ADDR);

	ox810sata_link_wait_ready(ap);

	result = ox810sata_ioport_read(ap, LINK_DATA);

	if (!NULLPTR(lock) && !in_irq())
		spin_unlock_irqrestore(lock, flags);

	return result;
}

/*
 *  Read standard SATA phy registers. Currently only used if
 * ->phy_reset hook called the sata_phy_reset() helper function.
 *
 * These registers are in another clock domain to the processor, access is via
 * some bridging registers
 *
 * @param ap hardware with the registers in
 * @param sc_reg the SATA PHY register
 * @return the value in the register
 */
static u32 ox810sata_scr_read_port(struct ata_port *ap, unsigned int sc_reg)
{
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;
	spinlock_t *lock = NULLPTR(pd) ? NULL : &pd->scrlock;

	return ox810sata_link_read(ap, SCR2LINK(sc_reg), lock);
}

/*
 * @return true if the port has a cable connected
 */
static u32 ox810sata_check_link(struct ata_port *ap)
{
	/* Check for the cable present indicated by SCR status bit-0 set */
	return ox810sata_scr_read_port(ap, SCR_STATUS) & 0x1;
}

/*
 * Reads the Status ATA shadow register from hardware. Due to a fault with PIO
 * transfers, it sometimes necessary to mask out the DRQ bit
 * @param ap hardware with the registers in
 * @return The status register
 */
static u8 ox810sata_check_status(struct ata_port *ap)
{
	u8 status = ox810sata_ioportraid_read(ap, ORB2) >> 24;

	// check for the drive going missing indicated by SCR status bits 0-3 = 0
	u32 reg = ox810sata_check_link(ap);

	if (ox810sata_hd.hw_raid_active)
		reg |= ox810sata_check_link(ox810sata_other_ap(ap));

	if (!reg)
		status |= ATA_DF | ATA_ERR;

	return status;
}

static bool ox810sata_qc_data_protocol(struct ata_queued_cmd *const qc)
{
	return NULLPTR(qc) ? false :
			     (ata_is_data(qc->tf.protocol) ||
			      (qc->flags & ATA_QCFLAG_DMAMAP));
}

static void ox810sata_qc_complete(struct ata_port *const ap,
				  const enum ata_completion_errors ac_err)
{
	struct ata_queued_cmd *qc;
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;
	unsigned long flags = 0;

	spin_lock_irqsave(ap->lock, flags);
	qc = NULLPTR(ap) ? NULL : ox810sata_active_qc(ap);
	if (!NULLPTR(qc)) {
		if (!NULLPTR(pd))
			pd->active_qc = NULL;
		qc->err_mask = ac_err_mask(ox810sata_check_status(ap)) | ac_err;
		ata_qc_complete(qc);
	}
	spin_unlock_irqrestore(ap->lock, flags);
}

static int ox810sata_dma_alloc(void)
{
	int rc = 0;

	if (NULLPTR(ox810sata_hd.chan) || IS_ERR(ox810sata_hd.chan))
		ox810sata_hd.chan =
			dma_request_chan(&ox810sata_hd.pdev->dev, "sgdma");
	if (NULLPTR(ox810sata_hd.chan)) {
		pr_err("failed to obtain DMA channel\n");
		rc = -ENXIO;
	} else if (IS_ERR(ox810sata_hd.chan)) {
		rc = PTR_ERR(ox810sata_hd.chan);
		if (rc == -EPROBE_DEFER)
			pr_warn("waiting for DMA to initialize\n");
		else
			pr_err("error %d allocating DMA channel\n", rc);
		ox810sata_hd.chan = NULL;
	} else {
		ox810sata_dma_abort();
	}

	return rc;
}

static void ox810sata_dma_free(void)
{
	ox810sata_dma_abort();
	if (!NULLPTR(ox810sata_hd.chan)) {
		dmaengine_synchronize(ox810sata_hd.chan);
		dma_release_channel(ox810sata_hd.chan);
		ox810sata_hd.chan = NULL;
	}
}

static bool ox810sata_dma_busy_check(struct ata_port *const ap)
{
	struct ata_queued_cmd *qc =
		NULLPTR(ap) ? NULL : ox810sata_active_qc(ap);
	bool use_dma = NULLPTR(qc) ? true : ox810sata_qc_data_protocol(qc);

	if (NULLPTR(ox810sata_hd.chan) || !use_dma)
		return false;

	return NULLPTR(ox810sata_hd.desc) ?
		       false :
		       dmaengine_tx_status(ox810sata_hd.chan,
					   ox810sata_hd.desc->cookie,
					   NULL) == DMA_IN_PROGRESS;
}

static inline bool ox810sata_core_busy_check(void)
{
	ox810sata_iocore_andor(IDLE_STATUS, 0, 0);

	return (~ox810sata_iocore_read(IDLE_STATUS)) & IDLE_CORES;
}

/*
 * @return true if idle or false if still busy after timeout
 */
static bool ox810sata_core_idle_wait(void)
{
	int q = (IDLE_WAIT_MS * 1000) / ATOMIC_DELAY_US;

	while (ox810sata_core_busy_check()) {
		udelay(ATOMIC_DELAY_US - 1);
		if (--q <= 0)
			return false;
	}

	return true;
}

static void ox810sata_core_reset_start(void)
{
	// Dirty hack for core to operate normally
	// without additional resets or power triggering
	ox810sata_iocore_andor(DEVICE_CONTROL, 0, ~0);
	mdelay(CORE_RESET_DELAY_MS);
	ox810sata_iocore_andor(DEVICE_CONTROL, 0, 0);
	mdelay(CORE_RESET_DELAY_MS);
	ox810sata_iocore_andor(DEVICE_CONTROL, 0, ~0);
	mdelay(CORE_RESET_DELAY_MS);

	// reset Controller, Link and PHY
	ox810sata_reset_assert();
	mdelay(CORE_RESET_DELAY_MS);
}

/*
 * Turns on the cores clock and resets it
 */
static void ox810sata_core_reset(void)
{
	int i;

	for (i = 0; i < 9; i++) {
		ox810sata_core_reset_start();

		// un-reset the PHY, then Link and Controller
		ox810sata_reset_deassert();
		mdelay(CORE_RESET_DELAY_MS);

		if (!ox810sata_core_busy_check())
			break;
	}

	ox810sata_iocore_andor(DEVICE_CONTROL, 0, 0);

	// disable padding
	ox810sata_iocore_andor(DEVICE_CONTROL, ~DEVICE_CONTROL_PAD,
			       DEVICE_CONTROL_PADPAT);
}

static void ox810sata_srst_send(struct ata_port *const ap)
{
	if (NULLPTR(ap))
		return;

	ap->last_ctl = ap->ctl;
	ap->ctl &= ~ATA_SRST;

	// write values to registers
	ox810sata_ioportraid_andor(ap, ORB1, 0, 0);
	ox810sata_ioportraid_andor(ap, ORB2, 0, 0);
	ox810sata_ioportraid_andor(ap, ORB3, 0, 0);
	ox810sata_ioportraid_andor(ap, ORB4, 0, ap->ctl << 24);
	// command the core to send a control FIS
	ox810sata_send_control_fis(ap, CMD_WRITE_TO_ORB_REGS_NO_COMMAND);
	mdelay(1);

	// write value to register
	ox810sata_ioportraid_andor(ap, ORB4, 0, (ap->ctl | ATA_SRST) << 24);

	// command the core to send a control FIS
	ox810sata_send_control_fis(ap, CMD_WRITE_TO_ORB_REGS_NO_COMMAND);
	mdelay(1);

	// write value to register
	ox810sata_ioportraid_andor(ap, ORB4, 0, ap->ctl << 24);

	// command the core to send a control FIS
	ox810sata_send_control_fis(ap, CMD_WRITE_TO_ORB_REGS_NO_COMMAND);
	mdelay(ATA_WAIT_AFTER_RESET);
}

static int ox810sata_scr_read(struct ata_link *link, unsigned int sc_reg,
			      u32 *val)
{
	*val = ox810sata_scr_read_port(link->ap, sc_reg);
	return 0;
}

/*
 * allows access to the link layer registers
 * @param link_reg the link layer register to access (oxsemi indexing ie
 *        00 = static config, 04 = phy ctrl)
 */
static void ox810sata_link_write(struct ata_port *ap, unsigned int link_reg,
				 u32 val, spinlock_t *lock)
{
	unsigned long flags = 0;

	if (!NULLPTR(lock) && !in_irq())
		spin_lock_irqsave(lock, flags);

	ox810sata_link_wait_ready(ap);

	ox810sata_ioport_andor(ap, LINK_DATA, 0, val);

	// TODO comment
	wmb();
	/* accessed twice as a workaround for a bug in the SATA abp bridge
	 * hardware (bug 6828)
	 */
	ox810sata_ioport_andor(ap, LINK_WR_ADDR, 0, link_reg);

	// TODO comment
	wmb();
	(void)ox810sata_ioport_read(ap, LINK_WR_ADDR);

	ox810sata_link_wait_ready(ap);

	if (!NULLPTR(lock) && !in_irq())
		spin_unlock_irqrestore(lock, flags);
}

/*
 *  Write standard SATA phy registers. Currently only used if
 * phy_reset hook called the sata_phy_reset() helper function.
 *
 * These registers are in another clock domain to the processor, access is via
 * some bridging registers
 *
 * @param ap hardware with the registers in
 * @param sc_reg the SATA PHY register
 * @param val the value to write into the register
 */
static void ox810sata_scr_write_port(struct ata_port *ap, unsigned int sc_reg,
				     u32 val)
{
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;
	spinlock_t *lock = NULLPTR(pd) ? NULL : &pd->scrlock;

	ox810sata_link_write(ap, SCR2LINK(sc_reg), val, lock);
}

static int ox810sata_scr_write(struct ata_link *link, unsigned int sc_reg,
			       u32 val)
{
	ox810sata_scr_write_port(link->ap, sc_reg, val);
	return 0;
}

/*
 * sends a sync-escape if there is a link present
 */
static bool ox810sata_sync_escape_send(struct ata_port *const ap)
{
	bool reset = false;

	// read the SSTATUS register and only send a sync escape if there is a link active
	if (ox810sata_check_link(ap)) {
		ox810sata_ioport_andor(ap, SATA_COMMAND, ~SATA_OPCODE_MASK,
				       CMD_SYNC_ESCAPE);
		reset = true;
	}

	return reset;
}

static void ox810sata_phy_error_clear(struct ata_port *const ap)
{
	// clear phy/link errors
	ox810sata_scr_write_port(ap, SCR_ERROR, ~0);
}

static void ox810sata_errors_clear(struct ata_port *const ap)
{
	ox810sata_reg_access_error_clear(ap);
	ox810sata_cs_error_clear(ap);
	ox810sata_sctl_error_clear(ap);
	ox810sata_phy_error_clear(ap);
}

static bool ox810sata_link_hard_reset(struct ata_port *ap)
{
	int tries = 3;
	int rc;

	do {
		const unsigned long *timing =
			sata_ehc_deb_timing(&ap->link.eh_context);

		ox810sata_errors_clear(ap);
		rc = sata_link_hardreset(&ap->link, timing,
					 msecs_to_jiffies(IDLE_WAIT_MS), NULL,
					 NULL);
	} while ((!ox810sata_check_link(ap) || rc) && tries-- > 0);

	if (rc)
		ata_port_warn(ap, "link reset fail\n");

	return rc == 0;
}

/*
 * @param ap ata port
 */
static bool ox810sata_cleanup(struct ata_port *const ap)
{
	bool both = false;

	ox810sata_irq_off(ap);

	// abort DMA
	ox810sata_errors_clear(ap);
	ox810sata_dma_abort();
	if (ox810sata_core_idle_wait())
		goto cleanup_exit;

	// link hard reset
	if (ox810sata_link_hard_reset(ap))
		goto cleanup_exit;

	if (ox810sata_core_idle_wait())
		goto cleanup_exit;

	// send sync escape code
	ox810sata_errors_clear(ap);
	if (ox810sata_sync_escape_send(ap)) {
		if (ox810sata_core_idle_wait())
			goto cleanup_exit;
		ox810sata_errors_clear(ap);
	}

	// SRST
	ox810sata_srst_send(ap);
	if (ox810sata_core_idle_wait())
		goto cleanup_exit;

	/* Perform any SATA core re-initialisation after reset */
	/* post reset init needs to be called for both ports as there's one reset for both ports */

	// core not recovering, reset it (with both ports)
	both = true;
	ox810sata_core_reset();

cleanup_exit:
	if (both)
		ata_port_warn(ap, "core reset!\n");

	ox810sata_errors_clear(ap);
	ox810sata_irq_on(ap);
	return both;
}

/*
 * Called after IDENTIFY [PACKET] DEVICE is issued to each device found.
 * Typically used to apply device-specific fixups prior to issue of
 * SET FEATURES - XFER MODE, and prior to operation.
 * @param port The port to configure
 * @param pdev The hardware associated with controlling the port
 */
static void ox810sata_dev_config(struct ata_device *const pdev)
{
	struct ata_port *ap = pdev->link->ap;
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;

	pr_info("\n");

	(void)ox810sata_core_idle_wait();

	// turn on phy error detection by removing the masks
	ox810sata_link_write(ap, 0x0C, 0x00030003,
			     NULLPTR(pd) ? NULL : &pd->scrlock);

	// tune for sata compatibility
	ox810sata_scr_write_port(ap, 0x10, 0x00002988);
	ox810sata_scr_write_port(ap, 0x14, 0x00055629);

	// enable hotplug event detection
	ox810sata_scr_write_port(ap, SERROR_IRQ_MASK, 0x03feffff);
	ox810sata_scr_write_port(ap, SCR_ACTIVE, ~0 & ~(1 << 26) & ~(1 << 16));

	/* Set the bits to put the interface into 28 or 48-bit node */
	ox810sata_ioport_andor(ap, DRIVE_CONTROL,
			       /* mask out the pair of bits associaed with each port */
			       ~(3 << (ox810sata_port_no(ap) * 2)),
			       /* set the mode pair associated with each port */
			       ((pdev->flags & ATA_DFLAG_LBA48) ? DR_CON_48 : DR_CON_28)
				<< (ox810sata_port_no(ap) * 2));

	/* if this is an ATA-6 disk, put the port into ATA-5 auto translate mode */
	if (pdev->flags & ATA_DFLAG_LBA48)
		ox810sata_ioport_andor(ap, PORT_CONTROL, ~0, 0x2);

	ox810sata_phy_error_clear(ap);
}

/*
 * Output the taskfile for diagnostic reasons, it will always appear in the
 * debug output as if it's a task file being written.
 * @param tf The taskfile to output
 */
static void tfdump(const struct ata_taskfile *tf, bool ld)
{
	if (tf->flags & ATA_TFLAG_LBA48) {
		pr_info("{%s} cmd=0x%X ft=0x%X%X LBA48=0x%02X%02X%02X%02X%02X%02X nsect=0x%02X%02X ctl=0x%02X dev=0x%X\n",
			ld ? "load" : "read", tf->command,

			tf->hob_feature, tf->feature,

			tf->hob_lbah, tf->hob_lbam, tf->hob_lbal, tf->lbah,
			tf->lbam, tf->lbal,

			tf->hob_nsect, tf->nsect, tf->ctl, tf->device);
	} else {
		pr_info("{%s} cmd=0x%X ft=0x%X LBA28=0x%01X%02X%02X%02X nsect=0x%02X ctl=0x%02X dev=0x%X\n",
			ld ? "load" : "read", tf->command,

			tf->feature,

			tf->device & 0x0f, tf->lbah, tf->lbam, tf->lbal,

			tf->nsect, tf->ctl, tf->device);
	}
}

/*
 * called to write a taskfile into the ORB registers
 * @param ap hardware with the registers in
 * @param tf taskfile to write to the registers
 */
static void ox810sata_tf_load(struct ata_port *ap,
			      const struct ata_taskfile *tf)
{
	u32 orb1 = 0;
	u32 orb2 = 0;
	u32 orb3 = 0;
	u32 orb4 = 0;
	unsigned int is_addr = tf->flags & ATA_TFLAG_ISADDR;

	pr_info("\n");

	/* if the control register has changed, write it */
	if (tf->ctl != ap->last_ctl) {
		orb4 = tf->ctl << 24;
		/* write value to register */
		ox810sata_ioportraid_andor(ap, ORB4, 0, orb4);
		ap->last_ctl = tf->ctl;
		ata_wait_idle(ap);
	}

	/* check if the ctl register has interrupts disabled or enabled and
	 * modify the interrupt enable registers on the ata core as required
	 */
	if (tf->ctl & ATA_NIEN) {
		ata_port_warn(ap, "NIEN!\n");
		//        ox810sata_irq_off(ap);
		//    } else {
		//        ox810sata_irq_on(ap);
	}

	orb2 |= (tf->command) << 24;

	/* write 48 or 28 bit tf parameters */
	if (is_addr) {
		/* set LBA bit as it's an address */
		orb1 |= (tf->device & ATA_LBA) << 24;

		if (tf->flags & ATA_TFLAG_LBA48) {
			orb1 |= ATA_LBA << 24;
			orb2 |= (tf->hob_nsect) << 8;
			orb3 |= (tf->hob_lbal) << 24;
			orb4 |= (tf->hob_lbam) << 0;
			orb4 |= (tf->hob_lbah) << 8;
			orb4 |= (tf->hob_feature) << 16;
		} else {
			orb3 |= (tf->device & 0xf) << 24;
		}

		/* write 28-bit lba */
		orb2 |= (tf->nsect) << 0;
		orb2 |= (tf->feature) << 16;
		orb3 |= (tf->lbal) << 0;
		orb3 |= (tf->lbam) << 8;
		orb3 |= (tf->lbah) << 16;
		orb4 |= (tf->ctl) << 24;
	}

	if (tf->flags & ATA_TFLAG_DEVICE)
		orb1 |= (tf->device) << 24;

	/* write values to registers */
	ox810sata_ioportraid_andor(ap, ORB1, 0, orb1);
	ox810sata_ioportraid_andor(ap, ORB2, 0, orb2);
	ox810sata_ioportraid_andor(ap, ORB3, 0, orb3);
	ox810sata_ioportraid_andor(ap, ORB4, 0, orb4);

	ata_wait_idle(ap);

	tfdump(tf, true);
}

/*
 * Called to read the hardware registers / DMA buffers, to
 * obtain the current set of taskfile register values.
 * @param ap hardware with the registers in
 * @param tf taskfile to read the registers into
 */
static void ox810sata_tf_read(struct ata_port *ap, struct ata_taskfile *tf)
{
	/* read the orb registers */
	u32 orb1, orb2, orb3, orb4;

	ata_wait_idle(ap);

	orb1 = ox810sata_ioportraid_read(ap, ORB1);
	orb2 = ox810sata_ioportraid_read(ap, ORB2);
	orb3 = ox810sata_ioportraid_read(ap, ORB3);
	orb4 = ox810sata_ioportraid_read(ap, ORB4);

	pr_info("\n");

	/* read common 28/48 bit tf parameters */
	tf->device = (orb1 >> 24);
	tf->nsect = (orb2 >> 0);
	tf->feature = (orb2 >> 16);
	tf->command = ox810sata_check_status(ap);

	/* read 48 or 28 bit tf parameters */
	if (tf->flags & ATA_TFLAG_LBA48) {
		tf->hob_nsect = (orb2 >> 8);

		tf->lbal = (orb3 >> 0);
		tf->lbam = (orb3 >> 8);
		tf->lbah = (orb3 >> 16);
		tf->hob_lbal = (orb3 >> 24);

		tf->hob_lbam = (orb4 >> 0);
		tf->hob_lbah = (orb4 >> 8);
		/* feature ext and control are write only */
	} else {
		/* read 28-bit lba */
		tf->lbal = (orb3 >> 0);
		tf->lbam = (orb3 >> 8);
		tf->lbah = (orb3 >> 16);
	}

	tfdump(tf, false);
}

static void ox810sata_dma_callback(void *arg)
{
	struct ata_port *ap = (void *)arg;

	if (!NULLPTR(ox810sata_hd.desc)) {
		ox810sata_hd.desc->callback = NULL;
		ox810sata_hd.desc = NULL;
	}

	ox810sata_qc_complete(ap, AC_ERR_OK);
}

static int ox810sata_qc_defer(struct ata_queued_cmd *qc)
{
	int ret;

	if (ox810sata_core_busy_check() || ox810sata_dma_busy_check(qc->ap))
		return ATA_DEFER_LINK;

	ret = ata_std_qc_defer(qc);
	if (ret)
		return ret;

	pr_debug("ata%u: tag#%u -------\n", qc->ap->print_id, qc->tag);

	return 0;
}

/*
 * Prepare as much as possible for a command without involving anything that is
 * shared between ports.
 */
static enum ata_completion_errors ox810sata_qc_prep(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = NULLPTR(qc) ? NULL : qc->ap;
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;
	bool raid_reg = false; // default to no raid
	int tries = 1;
	bool port_fail, other_port_fail = false;

	pr_debug("ata%u: tag#%u\n", ap->print_id, qc->tag);

	pd->active_qc = NULL;

	// If it is an internal cmd then there was no qc_defer
	if (ata_tag_internal(qc->tag)) {
		tries = 3;
		while ((!ox810sata_core_idle_wait() ||
			ox810sata_dma_busy_check(qc->ap)) &&
		       tries-- > 0) {
			if (ox810sata_cleanup(ap))
				break;
		}
	}

	// get raid settings from the bio if they exist
	if (qc->scsicmd && qc->scsicmd->request && qc->scsicmd->request->bio) {
		if (ox810sata_hd.hw_raid_active != raid_reg) {
			pr_info("hardware RAID %s",
				raid_reg ? "activated" : "deactivated");
			ox810sata_hd.hw_raid_active = raid_reg;
		}
	}

	ox810sata_irq_on(ap);

	tries = 1;
	while (1) {
		// check for failed ports prior to issuing raid-ed commands
		port_fail = ox810sata_check_link(ap) ? false : true;
		other_port_fail = false;
		if (ox810sata_hd.hw_raid_active)
			other_port_fail =
				!ox810sata_check_link(ox810sata_other_ap(ap));

		ox810sata_accumulated_RAID_faults |=
			port_fail ? 1UL << ox810sata_port_no(ap) : 0;
		ox810sata_accumulated_RAID_faults |=
			other_port_fail ? 1UL << ox810sata_other_port_no(ap) :
					  0;

		if (tries-- == 0) {
			if (port_fail || other_port_fail)
				return AC_ERR_ATA_BUS;
			break;
		}
		if (!port_fail && !other_port_fail)
			break;

		if (ox810sata_cleanup(ap))
			break;
	}

	if (ox810sata_hd.hw_raid_active) {
		ox810sata_hd.active_ap = ap;

		// at the moment we only do raid-1
		ox810sata_iocore_andor(RAID_CONTROL, 0, OXNASSATA_RAID1);
		ox810sata_iocore_andor(RAID_SET, 0, OXNASSATA_RAID_TWODISKS);
	} else {
		// Set the RAID controller hardware to idle
		ox810sata_iocore_andor(RAID_CONTROL, 0, OXNASSATA_NOTRAID);
	}

	(void)ox810sata_core_idle_wait();

	ox810sata_tf_load(ap, &qc->tf);

	return ox810sata_qc_data_protocol(qc) ? ata_bmdma_qc_prep(qc) :
						AC_ERR_OK;
}

/*
 * qc_issue is used to make a command active, once the hardware and S/G tables
 * have been prepared. IDE BMDMA drivers use the helper function
 * ata_qc_issue_prot() for taskfile protocol-based dispatch. More advanced drivers
 * roll their own ->qc_issue implementation, using this as the "issue new ATA
 * command to hardware" hook.
 * @param qc the queued command to issue
 */
static unsigned int ox810sata_qc_issue(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = NULLPTR(qc) ? NULL : qc->ap;
	struct ox810sata_port_priv *pd = NULLPTR(ap) ? NULL : ap->private_data;

	(void)ox810sata_core_idle_wait();

	if (!NULLPTR(pd))
		pd->active_qc = qc;

	if (!NULLPTR(ox810sata_hd.chan)) {
		if (ox810sata_qc_data_protocol(qc) && !NULLPTR(qc->sg)) {
			struct dma_slave_config sconf;

			if (qc->dma_dir == DMA_FROM_DEVICE) {
				sconf.src_addr = ox810sata_hd.data_phys;
				sconf.direction = DMA_DEV_TO_MEM;
			} else {
				sconf.dst_addr = ox810sata_hd.data_phys;
				sconf.direction = DMA_MEM_TO_DEV;
			}
			dmaengine_slave_config(ox810sata_hd.chan, &sconf);
			ox810sata_hd.desc = dmaengine_prep_slave_sg(ox810sata_hd.chan, qc->sg,
								    qc->n_elem,
								    qc->dma_dir,
								    DMA_PREP_INTERRUPT |
								    DMA_CTRL_ACK);
			if (!NULLPTR(ox810sata_hd.desc)) {
				ox810sata_hd.desc->callback =
					ox810sata_dma_callback;
				ox810sata_hd.desc->callback_param = ap;
				dmaengine_submit(ox810sata_hd.desc);
			}
			dma_async_issue_pending(ox810sata_hd.chan);
		} else {
			if (!NULLPTR(ox810sata_hd.desc)) {
				ox810sata_hd.desc->callback = NULL;
				ox810sata_hd.desc = NULL;
			}
		}
	}

	// Command that the orb registers get written to drive
	ox810sata_send_control_fis(ap, CMD_WRITE_TO_ORB_REGS);

	return AC_ERR_OK;
}

static bool ox810sata_qc_fill_rtf(struct ata_queued_cmd *qc)
{
	(void)ox810sata_core_idle_wait();

	ox810sata_tf_read(qc->ap, &qc->result_tf);
	return true;
}

/*
 * irq_handler is the interrupt handling routine registered with the system,
 * by libata.
 */
static irqreturn_t ox810sata_irq_handler(int irq, void *dev_instance)
{
	int cnr, port_no;
	irqreturn_t ret = IRQ_NONE;

	for (port_no = 0; port_no < SATA_OXNAS_MAX_PORTS; port_no++) {
		struct ata_port *ap =
			((struct ata_host *)dev_instance)->ports[port_no];
		u32 int_status = 0;

		if (NULLPTR(ap))
			continue;

		// check the ISR for the port to see if it created the interrupt
		cnr = 0;
		while (cnr < 3) {
			u32 i = ox810sata_ioportraid_read(ap, INT_STATUS) &
				INT_USED;

			if (!i)
				break;

			// Clear and mask pending interrupts
			ox810sata_ioportraid_andor(ap, INT_CLEAR, 0, i);
			ox810sata_ioportraid_andor(ap, INT_DISABLE, 0, i);

			// store interrupt status for the bottom end
			int_status |= i;
			cnr++;
		}

		if (cnr) {
			if (int_status & INT_REG_ACCESS_ERR)
				ata_port_err(ap, "irq#%u INT_REG_ACCESS_ERR\n", irq);
			else
				pr_debug("irq#%d status=0x%08X cnr=%d\n", irq,
					 int_status, cnr);

			if (!ox810sata_dma_busy_check(ap)) {
				if (int_status & INT_END_OF_CMD)
					ox810sata_qc_complete(ap, AC_ERR_OK);
			}

			if (int_status & (INT_LINK_SERROR | INT_LINK_IRQ)) {
				u32 serror =
					ox810sata_scr_read_port(ap, SCR_ERROR);

				if (serror &
				    (SERR_DEV_XCHG | SERR_PHYRDY_CHG)) {
					ata_port_info(ap, "hotplug event\n");
					ap->link.eh_info.action |= ATA_EH_RESET;
					ata_ehi_hotplugged(&ap->link.eh_info);
					ata_port_freeze(ap);
				}
			}

			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

/*
 * port_start() is called just after the data structures for each port are
 * initialized. Typically this is used to alloc per-port DMA buffers, tables
 * rings, enable DMA engines and similar tasks.
 *
 * @return 0 = success
 * @param ap hardware with the registers in
 */
static int ox810sata_port_start(struct ata_port *ap)
{
	struct ox810sata_port_priv *pd;
	int rc;

	rc = ata_bmdma_port_start(ap);
	if (rc)
		return rc;

	/* allocate port private data memory and attach to port */
	pd = devm_kzalloc(&ox810sata_hd.pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (NULLPTR(pd))
		return -ENOMEM;

	ap->private_data = pd;
	ap->print_id = ap->port_no + 1;
	pd->port = ap;
	pd->scrlock = __SPIN_LOCK_UNLOCKED(pd->scrlock);

	ata_port_info(ap, "port%u started\n", ap->port_no);

	// Additional cleanup/reset(s) to workaround for
	// core not responding when issuing first queued command
	(void)ox810sata_cleanup(ap);

	return 0;
}

/*
 * port_stop() is called after ->host_stop(). It's sole function is to
 * release DMA/memory resources, now that they are no longer actively being
 * used.
 */
static void ox810sata_port_stop(struct ata_port *ap)
{
	ox810sata_qc_complete(ap, AC_ERR_OK);
	(void)ox810sata_cleanup(ap);
	ox810sata_irq_off(ap);
}

static void ox810sata_error_handler(struct ata_port *ap)
{
	(void)ox810sata_cleanup(ap);

	ata_std_error_handler(ap);
}

static void ox810sata_post_internal_cmd(struct ata_queued_cmd *qc)
{
	pr_debug("ata%u: tag#%u\n", qc->ap->print_id, qc->tag);

	if (qc->flags & ATA_QCFLAG_FAILED)
		ox810sata_cleanup(qc->ap);
}

static int ox810sata_check_ready(struct ata_link *link)
{
	u8 status = ox810sata_check_status(link->ap);

	return ata_check_ready(status);
}

static int ox810sata_softreset(struct ata_link *link, unsigned int *class,
			       unsigned long deadline)
{
	int rc;
	struct ata_port *ap = link->ap;
	struct ata_taskfile tf;

	if (ata_link_offline(link)) {
		pr_warn("PHY reports no device\n");
		if (!NULLPTR(class))
			*class = ATA_DEV_NONE;
		return 0;
	}

	ox810sata_srst_send(ap);

	rc = ata_wait_ready(link, deadline, ox810sata_check_ready);

	// if link is occupied, -ENODEV too is an error
	if (rc && (rc != -ENODEV || sata_scr_valid(link))) {
		ata_port_err(ap, "SRST failed (errno=%d)\n", rc);
		return rc;
	}

	// determine by signature whether we have ATA or ATAPI devices
	ox810sata_tf_read(ap, &tf);
	if (!NULLPTR(class)) {
		*class = ata_dev_classify(&tf);

		if (*class == ATA_DEV_UNKNOWN)
			*class = ATA_DEV_NONE;

		pr_info("class=%u\n", *class);
	}

	return 0;
}

static struct ata_port_operations ox810sata_port_ops = {
	.inherits = &sata_port_ops,
	.qc_defer = ox810sata_qc_defer,
	.qc_prep = ox810sata_qc_prep,
	.qc_issue = ox810sata_qc_issue,
	.qc_fill_rtf = ox810sata_qc_fill_rtf,
	.sff_check_status = ox810sata_check_status,
	.softreset = ox810sata_softreset,
	.dev_config = ox810sata_dev_config,
	.scr_read = ox810sata_scr_read,
	.scr_write = ox810sata_scr_write,
	.port_start = ox810sata_port_start,
	.port_stop = ox810sata_port_stop,
	.error_handler = ox810sata_error_handler,
	.post_internal_cmd = ox810sata_post_internal_cmd,
};

/* the scsi_host_template structure describes the basic capabilities of libata
 * and our 921 core to the SCSI framework, it contains the addresses of functions
 * in the libata library that handle top level comands from the SCSI library
 */
static struct scsi_host_template ox810sata_sht = {
	ATA_NCQ_SHT(DRIVER_NAME),
	.can_queue = SATA_OXNAS_QUEUE_DEPTH,
	.sg_tablesize = SATA_OXNAS_MAX_PRD,
	.dma_boundary = SATA_OXNAS_DMA_BOUNDARY,
	.unchecked_isa_dma = 0,
};

/*
 * port capabilities for the ox810 sata ports.
 */
static const struct ata_port_info ox810sata_port_info = {
	.flags = SATA_OXNAS_HOST_FLAGS,
	.pio_mask = ATA_PIO4, /* pio modes 0..4 */
	.udma_mask = ATA_UDMA6, /* udma0-6 */
	.port_ops = &ox810sata_port_ops,
};

/*
 * The driver probe function.
 * Registered with the amba bus driver as a parameter of ox810sata_driver.bus
 * it will register the ata device with kernel first performing any
 * initialisation required (if the correct device is present).
 * @param pdev Pointer to the 921 device structure
 * @return 0 if no errors
 */
static int ox810sata_driver_probe(struct platform_device *pdev)
{
	u32 version;
	void __iomem *port_base = NULL, *port_end = NULL;
	const struct ata_port_info *port_info[] = { &ox810sata_port_info,
						    &ox810sata_port_info,
						    NULL };
	struct resource *res;
	int i, rc, irq = 0, n_ports = SATA_OXNAS_MAX_PORTS;

	// Get IRQ line
	if (!NULLPTR(pdev->dev.of_node))
		irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq <= 0)
		irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		pr_err("couldn't acquire IRQ line\n");
		rc = irq == 0 ? -ENXIO : irq;
		goto error_exit_with_cleanup;
	}

	// Check I/O base
	port_base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(port_base)) {
		rc = PTR_ERR(port_base);
		goto error_exit_with_cleanup;
	}
	port_end = port_base + resource_size(res) - 1;
	ox810sata_hd.iomap = port_base;
	ox810sata_hd.pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ox810sata_hd.data_phys = res->start;

	// Hold on to a DMA channel for the life of the SATA driver
	rc = ox810sata_dma_alloc();
	if (rc)
		goto error_exit_with_cleanup;

	ox810sata_hd.clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(ox810sata_hd.clk)) {
		rc = PTR_ERR(ox810sata_hd.clk);
		ox810sata_hd.clk = NULL;
		goto error_exit_with_cleanup;
	}

	ox810sata_hd.rst_sata = devm_reset_control_get(&pdev->dev, "sata");
	if (IS_ERR(ox810sata_hd.rst_sata)) {
		rc = PTR_ERR(ox810sata_hd.rst_sata);
		ox810sata_hd.rst_sata = NULL;
		goto error_exit_with_cleanup;
	}

	ox810sata_hd.rst_link = devm_reset_control_get(&pdev->dev, "link");
	if (IS_ERR(ox810sata_hd.rst_link)) {
		rc = PTR_ERR(ox810sata_hd.rst_link);
		ox810sata_hd.rst_link = NULL;
		goto error_exit_with_cleanup;
	}

	ox810sata_hd.rst_phy = devm_reset_control_get(&pdev->dev, "phy");
	if (IS_ERR(ox810sata_hd.rst_phy)) {
		rc = PTR_ERR(ox810sata_hd.rst_phy);
		ox810sata_hd.rst_phy = NULL;
		goto error_exit_with_cleanup;
	}

	ox810sata_clock_enable();
	mdelay(1);
	// reset the core
	ox810sata_core_reset();

	// Get and check number of ports
	if (!NULLPTR(pdev->dev.of_node)) {
		(void)of_property_read_u32(pdev->dev.of_node, "nr-ports",
					   &n_ports);
		if (n_ports < 1 || n_ports > SATA_OXNAS_MAX_PORTS) {
			pr_err("invalid number of ports (=%d)\n", n_ports);
			rc = -ENXIO;
			goto error_exit_with_cleanup;
		}
	}

	version = ox810sata_ioport_read(NULL, SATA_VERSION);
	if (n_ports > 1 &&
	    ox810sata_ioport_read(NULL, PORT_SIZE + SATA_VERSION) != version) {
		n_ports = 1;
		port_info[1] = NULL;
	}

	// check we support this version of the core
	switch (version) {
	case SATA_OXNAS_CORE_VERSION:
		pr_info("934 %s SATA core v%u.%02X (0x%08X), iomap=[0x%08X-0x%08X]\n",
			n_ports > 1 ? "two-ports" : "single-port", version >> 8,
			version & 0xFF, ox810sata_iocore_read(SATA_VERSION),
			(u32)port_base, (u32)port_end);
		break;
	default:
		pr_err("unknown SATA core (v%u.%02X/0x%08X, iomap=[0x%08X-0x%08X])\n",
		       version >> 8, version & 0xFF,
		       ox810sata_iocore_read(SATA_VERSION), (u32)port_base,
		       (u32)port_end);
		rc = -EINVAL;
		goto error_exit_with_cleanup;
	}

	// allocate memory and check
	ox810sata_hd.host =
		ata_host_alloc_pinfo(&pdev->dev, port_info, n_ports);
	if (!NULLPTR(ox810sata_hd.host) && !IS_ERR(ox810sata_hd.host)) {
		ox810sata_hd.host->private_data = &ox810sata_hd; // recursion
		// call ata_device_add and begin probing for drives
		rc = ata_host_activate(ox810sata_hd.host, irq,
				       ox810sata_irq_handler,
				       SATA_OXNAS_IRQ_FLAG, &ox810sata_sht);
		if (rc) {
			ox810sata_hd.host = NULL;
			goto error_exit_with_cleanup;
		}
	}
	if (IS_ERR(ox810sata_hd.host)) {
		rc = PTR_ERR(ox810sata_hd.host);
		ox810sata_hd.host = NULL;
	}
	if (NULLPTR(ox810sata_hd.host)) {
		if (!rc)
			rc = -ENOMEM;
		pr_err("couldn't create an ata host, error %d\n", rc);
		goto error_exit_with_cleanup;
	}

	// Get and enable ports power gpios if available
	for (i = 0; i < n_ports; i++) {
		struct gpio_desc *gpio_power = devm_gpiod_get_index_optional(&pdev->dev, "power",
									     i, 0);
		if (!NULLPTR(gpio_power))
			gpiod_direction_output(gpio_power, 1);
	}

	return 0;

error_exit_with_cleanup:
	if (irq > 0 && !NULLPTR(pdev->dev.of_node))
		irq_dispose_mapping(irq);
	if (!NULLPTR(ox810sata_hd.clk))
		clk_put(ox810sata_hd.clk);
	if (!NULLPTR(ox810sata_hd.rst_sata))
		reset_control_put(ox810sata_hd.rst_sata);
	if (!NULLPTR(ox810sata_hd.rst_link))
		reset_control_put(ox810sata_hd.rst_link);
	if (!NULLPTR(ox810sata_hd.rst_phy))
		reset_control_put(ox810sata_hd.rst_phy);
	if (!NULLPTR(ox810sata_hd.host))
		ata_host_detach(ox810sata_hd.host);
	ox810sata_dma_free();
	ox810sata_clock_disable();
	return rc;
}

/*
 * Called when the amba bus tells this device to remove itself.
 * @param pdev pointer to the device that needs to be shutdown
 */
static int ox810sata_driver_remove(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	unsigned int i;

	for (i = 0; i < host->n_ports; i++)
		scsi_remove_host(host->ports[i]->scsi_host);

	ox810sata_dma_free();

	// Hold reset to the SATA block
	// !!! This is needed for normal bootloader start !!!
	ox810sata_core_reset_start();

	return 0;
}

static const struct of_device_id ox810sata_match[] = {
	{
		.compatible = "plxtech,nas782x-sata",
	},
	{},
};

MODULE_DEVICE_TABLE(of, ox810sata_match);

static struct platform_driver ox810sata_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ox810sata_match,
	},
	.probe = ox810sata_driver_probe,
	.remove = ox810sata_driver_remove,
};

module_platform_driver(ox810sata_driver);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
