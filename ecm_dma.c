#include "dma_engine.h"

#include <linux/of_address.h>
#include <linux/bits.h>
#include <linux/irqreturn.h>
#include <linux/mod_devicetable.h>
#include <linux/stddef.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/compiler.h>
#include <linux/compiler_attributes.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/kern_levels.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_dma.h>
#include <linux/clk.h>
#include <linux/io-64-nonatomic-lo-hi.h>

MODULE_LICENSE("GPL v2");

/* Register/Descriptor Offsets */
#define XILINX_DMA_MM2S_CTRL_OFFSET 0x0000
#define XILINX_DMA_S2MM_CTRL_OFFSET 0x0030

/* Control Registers */
#define XILINX_DMA_REG_DMACR 0x0000
#define XILINX_DMA_DMACR_DELAY_MAX 0xff
#define XILINX_DMA_DMACR_DELAY_SHIFT 24
#define XILINX_DMA_DMACR_FRAME_COUNT_MAX 0xff
#define XILINX_DMA_DMACR_FRAME_COUNT_SHIFT 16
#define XILINX_DMA_DMACR_ERR_IRQ BIT(14)
#define XILINX_DMA_DMACR_DLY_CNT_IRQ BIT(13)
#define XILINX_DMA_DMACR_FRM_CNT_IRQ BIT(12)
#define XILINX_DMA_DMACR_MASTER_SHIFT 8
#define XILINX_DMA_DMACR_FSYNCSRC_SHIFT 5
#define XILINX_DMA_DMACR_FRAMECNT_EN BIT(4)
#define XILINX_DMA_DMACR_GENLOCK_EN BIT(3)
#define XILINX_DMA_DMACR_RESET BIT(2)
#define XILINX_DMA_DMACR_CIRC_EN BIT(1)
#define XILINX_DMA_DMACR_RUNSTOP BIT(0)
#define XILINX_DMA_DMACR_FSYNCSRC_MASK GENMASK(6, 5)
#define XILINX_DMA_DMACR_DELAY_MASK GENMASK(31, 24)
#define XILINX_DMA_DMACR_FRAME_COUNT_MASK GENMASK(23, 16)
#define XILINX_DMA_DMACR_MASTER_MASK GENMASK(11, 8)

#define XILINX_DMA_REG_DMASR 0x0004
#define XILINX_DMA_DMASR_EOL_LATE_ERR BIT(15)
#define XILINX_DMA_DMASR_ERR_IRQ BIT(14)
#define XILINX_DMA_DMASR_DLY_CNT_IRQ BIT(13)
#define XILINX_DMA_DMASR_FRM_CNT_IRQ BIT(12)
#define XILINX_DMA_DMASR_SOF_LATE_ERR BIT(11)
#define XILINX_DMA_DMASR_SG_DEC_ERR BIT(10)
#define XILINX_DMA_DMASR_SG_SLV_ERR BIT(9)
#define XILINX_DMA_DMASR_EOF_EARLY_ERR BIT(8)
#define XILINX_DMA_DMASR_SOF_EARLY_ERR BIT(7)
#define XILINX_DMA_DMASR_DMA_DEC_ERR BIT(6)
#define XILINX_DMA_DMASR_DMA_SLAVE_ERR BIT(5)
#define XILINX_DMA_DMASR_DMA_INT_ERR BIT(4)
#define XILINX_DMA_DMASR_SG_MASK BIT(3)
#define XILINX_DMA_DMASR_IDLE BIT(1)
#define XILINX_DMA_DMASR_HALTED BIT(0)
#define XILINX_DMA_DMASR_DELAY_MASK GENMASK(31, 24)
#define XILINX_DMA_DMASR_FRAME_COUNT_MASK GENMASK(23, 16)

#define XILINX_DMA_REG_CURDESC 0x0008
#define XILINX_DMA_REG_TAILDESC 0x0010
#define XILINX_DMA_REG_REG_INDEX 0x0014
#define XILINX_DMA_REG_FRMSTORE 0x0018
#define XILINX_DMA_REG_THRESHOLD 0x001c
#define XILINX_DMA_REG_FRMPTR_STS 0x0024
#define XILINX_DMA_REG_PARK_PTR 0x0028
#define XILINX_DMA_PARK_PTR_WR_REF_SHIFT 8
#define XILINX_DMA_PARK_PTR_WR_REF_MASK GENMASK(12, 8)
#define XILINX_DMA_PARK_PTR_RD_REF_SHIFT 0
#define XILINX_DMA_PARK_PTR_RD_REF_MASK GENMASK(4, 0)
#define XILINX_DMA_REG_VDMA_VERSION 0x002c

/* Register Direct Mode Registers */
#define XILINX_DMA_REG_VSIZE 0x0000
#define XILINX_DMA_REG_HSIZE 0x0004

#define XILINX_DMA_REG_FRMDLY_STRIDE 0x0008
#define XILINX_DMA_FRMDLY_STRIDE_FRMDLY_SHIFT 24
#define XILINX_DMA_FRMDLY_STRIDE_STRIDE_SHIFT 0

/* HW specific definitions */
#define XILINX_MCDMA_MAX_CHANS_PER_DEVICE 0x20
#define XILINX_DMA_MAX_CHANS_PER_DEVICE 0x2
#define XILINX_CDMA_MAX_CHANS_PER_DEVICE 0x1

#define XILINX_DMA_DMAXR_ALL_IRQ_MASK                                  \
	(XILINX_DMA_DMASR_FRM_CNT_IRQ | XILINX_DMA_DMASR_DLY_CNT_IRQ | \
	 XILINX_DMA_DMASR_ERR_IRQ)

#define XILINX_DMA_DMASR_ALL_ERR_MASK                                      \
	(XILINX_DMA_DMASR_EOL_LATE_ERR | XILINX_DMA_DMASR_SOF_LATE_ERR |   \
	 XILINX_DMA_DMASR_SG_DEC_ERR | XILINX_DMA_DMASR_SG_SLV_ERR |       \
	 XILINX_DMA_DMASR_EOF_EARLY_ERR | XILINX_DMA_DMASR_SOF_EARLY_ERR | \
	 XILINX_DMA_DMASR_DMA_DEC_ERR | XILINX_DMA_DMASR_DMA_SLAVE_ERR |   \
	 XILINX_DMA_DMASR_DMA_INT_ERR)

/*
 * Recoverable errors are DMA Internal error, SOF Early, EOF Early
 * and SOF Late. They are only recoverable when C_FLUSH_ON_FSYNC
 * is enabled in the h/w system.
 */
#define XILINX_DMA_DMASR_ERR_RECOVER_MASK                                 \
	(XILINX_DMA_DMASR_SOF_LATE_ERR | XILINX_DMA_DMASR_EOF_EARLY_ERR | \
	 XILINX_DMA_DMASR_SOF_EARLY_ERR | XILINX_DMA_DMASR_DMA_INT_ERR)

/* Axi VDMA Flush on Fsync bits */
#define XILINX_DMA_FLUSH_S2MM 3
#define XILINX_DMA_FLUSH_MM2S 2
#define XILINX_DMA_FLUSH_BOTH 1

/* Delay loop counter to prevent hardware failure */
#define XILINX_DMA_LOOP_COUNT 1000000

/* AXI DMA Specific Registers/Offsets */
#define XILINX_DMA_REG_SRCDSTADDR 0x18
#define XILINX_DMA_REG_BTT 0x28

/* AXI DMA Specific Masks/Bit fields */
#define XILINX_DMA_MAX_TRANS_LEN_MIN 8
#define XILINX_DMA_MAX_TRANS_LEN_MAX 23
#define XILINX_DMA_V2_MAX_TRANS_LEN_MAX 26
#define XILINX_DMA_CR_COALESCE_MAX GENMASK(23, 16)
#define XILINX_DMA_CR_CYCLIC_BD_EN_MASK BIT(4)
#define XILINX_DMA_CR_COALESCE_SHIFT 16
#define XILINX_DMA_BD_SOP BIT(27)
#define XILINX_DMA_BD_EOP BIT(26)
#define XILINX_DMA_COALESCE_MAX 255
#define XILINX_DMA_NUM_DESCS 255
#define XILINX_DMA_NUM_APP_WORDS 5

#define xilinx_prep_dma_addr_t(addr) \
	((dma_addr_t)((u64)addr##_##msb << 32 | (addr)))

/**
 * @brief 
 * struct xilinx_axidma_desc_hw - Hardware Descriptor for AXI DMA
 *
 * @next_desc: Next Descriptor Pointer @0x00
 * @next_desc_msb: MSB of Next Descriptor Pointer @0x04
 * @buf_addr: Buffer address @0x08
 * @buf_addr_msb: MSB of Buffer address @0x0C
 * @reserved1: Reserved @0x10
 * @reserved2: Reserved @0x14
 * @control: Control field @0x18
 * @status: Status field @0x1C
 * @app: APP Fields @0x20 - 0x30
 */
struct xilinx_axidma_desc_hw {
	u32 next_desc;
	u32 next_desc_msb;
	u32 buf_addr;
	u32 buf_addr_msb;
	u32 reserved[2];
	u32 control;
	u32 status;
	u32 app[XILINX_DMA_NUM_APP_WORDS];
} __aligned(64);

/**
 * @brief 
 * struct xilinx_axidma_tx_segment - Descriptor segment for AXI DMA
 *
 * @hw: Hardware descriptor
 * @node: Node in the descriptor segments list
 * @phys: Physical address of the descriptor
 */
struct xilinx_axidma_tx_segment {
	struct xilinx_axidma_desc_hw hw;
	struct list_head node;
	dma_addr_t phys;
} __aligned(64);

/**
 * @brief 
 * struct xilinx_dma_tx_descriptor - Per Transaction structure
 * 
 * @async_tx: Async transaction descriptor
 * @segments: TX segments list
 * @node: Node in the channel descriptors list
 * @cyclic: Check for cyclic transfers.
 * @err: Whether the descriptor has an error.
 * @residue: Residue of the completed descriptor
 */
struct xilinx_dma_tx_descriptor {
	struct dma_async_tx_descriptor async_tx;
	struct list_head segments;
	struct list_head node;
	bool cyclic;
	bool err;
	u32 residue;
};

/**
 * @brief 
 * struct xilinx_dma_chan - Driver specific DMA channel structure
 *
 * @xdev: Driver specific device structure
 * @ctrl_offset: Control registers offset
 * @desc_offset: TX descriptor registers offset
 * @lock: Descriptor operation lock
 * @pending_list: Descriptors waiting
 * @active_list: Descriptors ready to submit
 * @done_list: Complete descriptors
 * @free_seg_list: Free descriptors
 * @common: DMA common channel
 * @desc_pool: Descriptors pool
 * @dev: The dma device
 * @irq: Channel IRQ
 * @id: Channel ID
 * @direction: Transfer direction
 * @num_frms: Number of frames
 * @has_sg: Support scatter transfers
 * @cyclic: Check for cyclic transfers.
 * @genlock: Support genlock mode
 * @err: Channel has errors
 * @idle: Check for channel idle
 * @terminating: Check for channel being synchronized by user
 * @tasklet: Cleanup work after irq
 * @config: Device configuration info
 * @flush_on_fsync: Flush on Frame sync
 * @desc_pendingcount: Descriptor pending count
 * @ext_addr: Indicates 64 bit addressing is supported by dma channel
 * @desc_submitcount: Descriptor h/w submitted count
 * @seg_v: Statically allocated segments base
 * @seg_p: Physical allocated segments base
 * @cyclic_seg_v: Statically allocated segment base for cyclic transfers
 * @cyclic_seg_p: Physical allocated segments base for cyclic dma
 * @start_transfer: Differentiate b/w DMA IP's transfer
 * @stop_transfer: Differentiate b/w DMA IP's quiesce
 * @tdest: TDEST value for CHANNEL ID
 * @has_vflip: S2MM vertical flip
 *
 * @note!: remove tdest signal, and some other field for vdma/mcdma
 */
struct xilinx_dma_chan {
	struct xilinx_dma_device *xdev;
	u32 ctrl_offset;
	u32 desc_offset;
	spinlock_t lock;
	struct list_head pending_list;
	struct list_head active_list;
	struct list_head done_list;
	struct list_head free_seg_list;
	struct dma_chan common;
	// struct dma_pool *desc_pool;
	struct device *dev;
	int irq;
	int id;
	enum dma_transfer_direction direction;
	int num_frms;
	bool has_sg;
	bool cyclic;
	bool genlock;
	bool err;
	bool idle;
	bool terminating;
	struct tasklet_struct tasklet;
	bool flush_on_fsync;
	u32 desc_pendingcount;
	bool ext_addr;
	u32 desc_submitcount;
	struct xilinx_axidma_tx_segment *seg_v;
	dma_addr_t seg_p;
	struct xilinx_axidma_tx_segment *cyclic_seg_v;
	dma_addr_t cyclic_seg_p;
	void (*start_transfer)(struct xilinx_dma_chan *chan);
	int (*stop_transfer)(struct xilinx_dma_chan *chan);
	u16 tdest;
	// bool has_vflip;
};

/**
 * @brief 
 * struct xilinx_dma_device - Driver specific DMA device structure
 *
 * @regs: I/O mapped base address
 * @dev: Device structure
 * @common: DMA common device
 * @chan: Driver specific DMA channel
 * @flush_on_fsync: Flush on frame sync
 * @ext_addr: Indicates 64 bit addressing is supported by dma channel
 * @pdev: Platform device structure
 * @dma_config: DMA configuration
 * @axi_clk: DMA axi4-lite interrace clock
 * @tx_clk: DMA mm2s clock
 * @rx_clk: DMA s2mm clock
 * @sg_clk: DMA scatter gather engine clock
 * @s2mm_chan_id: DMA s2mm channel id
 * @mm2s_chan_id: DMA mm2s channel id
 * @max_buffer_len: Maximum buffer length
 */
struct xilinx_dma_device {
	void __iomem *regs;
	struct device *dev;
	struct dma_device common;
	struct xilinx_dma_chan *chan[XILINX_DMA_MAX_CHANS_PER_DEVICE];
	u32 flush_on_fsync;
	bool ext_addr;
	struct platform_device *pdev;
	// const struct xilinx_dma_config *dma_config; no need for dma, we have only one type of dma
	struct clk *axi_clk;
	struct clk *tx_clk;
	struct clk *rx_clk;
	struct clk *sg_clk;
	u32 s2mm_chan_id;
	u32 mm2s_chan_id;
	u32 max_buffer_len;
	bool is_coherent;
};

/* MACROs */
#define to_xilinx_chan(chan) container_of(chan, struct xilinx_dma_chan, common)
#define to_dma_tx_descriptor(tx) \
	container_of(tx, struct xilinx_dma_tx_descriptor, async_tx)
#define xilinx_dma_poll_timeout(chan, reg, val, cond, sleep_us, timeout_us)  \
	readl_poll_timeout_atomic(                                           \
		(chan->xdev->regs + chan->ctrl_offset + reg), (val), (cond), \
		(sleep_us), (timeout_us))

static inline u32 dma_read(struct xilinx_dma_chan *chan, u32 reg)
{
	return ioread32(chan->xdev->regs + reg);
}

static inline void dma_write(struct xilinx_dma_chan *chan, u32 reg, u32 val)
{
	iowrite32(val, chan->xdev->regs + reg);
}

static inline u32 dma_ctrl_read(struct xilinx_dma_chan *chan, u32 reg)
{
	return dma_read(chan, chan->ctrl_offset + reg);
}

static inline void dma_ctrl_write(struct xilinx_dma_chan *chan, u32 reg,
				  u32 val)
{
	dma_write(chan, chan->ctrl_offset + reg, val);
}

static inline void dma_ctrl_clr(struct xilinx_dma_chan *chan, u32 reg, u32 clr)
{
	dma_ctrl_write(chan, reg, dma_read(chan, reg) & ~clr);
}

static inline void dma_ctrl_set(struct xilinx_dma_chan *chan, u32 reg, u32 set)
{
	dma_ctrl_write(chan, reg, dma_read(chan, reg) | set);
}

static inline void dma_writeq(struct xilinx_dma_chan *chan, u32 reg, u64 val)
{
	lo_hi_writeq(val, chan->xdev->regs + chan->ctrl_offset + reg);
}

static inline void xilinx_write(struct xilinx_dma_chan *chan, u32 reg,
				dma_addr_t addr)
{
	if (chan->ext_addr)
		dma_writeq(chan, reg, addr);
	else
		dma_ctrl_write(chan, reg, addr);
}

static inline void xilinx_axidma_buf(struct xilinx_dma_chan *chan,
				     struct xilinx_axidma_desc_hw *hw,
				     dma_addr_t buf_addr, size_t sg_used,
				     size_t period_len)
{
	if (unlikely(chan->ext_addr)) {
		hw->buf_addr = lower_32_bits(buf_addr + sg_used + period_len);
		hw->buf_addr_msb =
			upper_32_bits(buf_addr + sg_used + period_len);
	} else {
		hw->buf_addr = buf_addr + sg_used + period_len;
	}
}

/**
 * @brief Allocate transaction segment
 * 
 * @param chan 
 * @return struct xilinx_axidma_tx_segment* , the allocated segment on success, NULL on failure
 */
static struct xilinx_axidma_tx_segment *
xilinx_axidma_alloc_tx_segment(struct xilinx_dma_chan *chan)
{
	struct xilinx_axidma_tx_segment *segment = NULL;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	if (!list_empty(&chan->free_seg_list)) {
		segment = list_first_entry(&chan->free_seg_list,
					   struct xilinx_axidma_tx_segment,
					   node);
		// remove the segment from the free list
		list_del(&segment->node);
	}
	spin_unlock_irqrestore(&chan->lock, flags);

	if (!segment)
		dev_dbg(chan->dev, "No free segment available\n");

	return segment;
}

static void xilinx_dma_clean_hw_desc(struct xilinx_axidma_desc_hw *hw)
{
	u32 next_desc = hw->next_desc;
	u32 next_desc_msb = hw->next_desc_msb;

	memset(hw, 0, sizeof(struct xilinx_axidma_desc_hw));

	hw->next_desc = next_desc;
	hw->next_desc_msb = next_desc_msb;
}

/**
 * @brief Free transaction segment
 * 
 * @param chan 
 * @param segment 
 */
static void xilinx_dma_free_tx_segment(struct xilinx_dma_chan *chan,
				       struct xilinx_axidma_tx_segment *segment)
{
	xilinx_dma_clean_hw_desc(&segment->hw);
	list_add_tail(&segment->node, &chan->free_seg_list);
}

/**
 * @brief Allocate transcation descriptor
 * 
 * @param chan 
 * @return struct xilinx_dma_tx_descriptor* 
 */
static struct xilinx_dma_tx_descriptor *
xilinx_dma_alloc_tx_descriptor(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_tx_descriptor *desc;
	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;
	INIT_LIST_HEAD(&desc->segments);
	return desc;
}

/**
 * @brief Free transcation descriptor
 * 
 * @param chan 
 * @param desc 
 */
static void xilinx_dma_free_tx_descriptor(struct xilinx_dma_chan *chan,
					  struct xilinx_dma_tx_descriptor *desc)
{
	struct xilinx_axidma_tx_segment *segment, *next;
	if (!desc)
		return;

	list_for_each_entry_safe(segment, next, &desc->segments, node) {
		list_del(&segment->node);
		xilinx_dma_free_tx_segment(chan, segment);
	}
	kfree(desc);
}

/**
 * @brief Free all the descriptors in one of the descriptor lists
 * 
 * @param chan 
 * @param list 
 */
static void xilinx_dma_free_desc_list(struct xilinx_dma_chan *chan,
				      struct list_head *list)
{
	struct xilinx_dma_tx_descriptor *desc, *next;
	list_for_each_entry_safe(desc, next, list, node) {
		list_del(&desc->node);
		xilinx_dma_free_tx_descriptor(chan, desc);
	}
}

/**
 * @brief Free all the descriptors in the channel
 * 
 * @param chan 
 */
static void xilinx_dma_free_descriptors(struct xilinx_dma_chan *chan)
{
	unsigned long flags;
	spin_lock_irqsave(&chan->lock, flags);

	xilinx_dma_free_desc_list(chan, &chan->pending_list);
	xilinx_dma_free_desc_list(chan, &chan->done_list);
	xilinx_dma_free_desc_list(chan, &chan->active_list);

	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * @brief Free channel resources
 * 
 * @param dchan 
 */
static void xilinx_dma_free_chan_resources(struct dma_chan *dma_chan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dma_chan);
	unsigned long flags;

	dev_dbg(chan->dev, "Freeing channel resources\n");

	xilinx_dma_free_descriptors(chan);

	spin_lock_irqsave(&chan->lock, flags);
	INIT_LIST_HEAD(&chan->free_seg_list);
	spin_unlock_irqrestore(&chan->lock, flags);

	/* Free memory that is allocated for BDs */
	/* sizeof(struct xilinx_axidma_tx_segment) * XILINX_DMA_NUM_DESCS */
	dma_free_coherent(chan->dev,
			  sizeof(*chan->seg_v) * XILINX_DMA_NUM_DESCS,
			  chan->seg_v, chan->seg_p);

	/* Free memory that is allocated for cyclic DMA Mode */
	// TODO!(pan): check if the BD_NUM in cyclic mode is only 1
	dma_free_coherent(chan->dev, sizeof(*chan->cyclic_seg_v),
			  chan->cyclic_seg_v, chan->cyclic_seg_p);
}

/**
 * @brief Compute residue for a given descriptor
 * 
 * @param chan 
 * @param desc 
 * @return u32 
 */
static u32 xilinx_dma_get_residue(struct xilinx_dma_chan *chan,
				  struct xilinx_dma_tx_descriptor *desc)
{
	struct xilinx_axidma_tx_segment *segment = NULL;
	struct xilinx_axidma_desc_hw *hw = NULL;
	struct list_head *entry = NULL;
	u32 residue = 0;

	list_for_each(entry, &desc->segments) {
		segment = list_entry(entry, struct xilinx_axidma_tx_segment,
				     node);
		hw = &segment->hw;
		residue += (hw->control - hw->status) &
			   chan->xdev->max_buffer_len;
	}
	return residue;
}

/**
 * @brief Cyclic DMA mode callback
 * 
 * @param chan 
 * @param desc 
 * @param flags 
 */
static void xilinx_dma_chan_handle_cyclic(struct xilinx_dma_chan *chan,
					  struct xilinx_dma_tx_descriptor *desc,
					  unsigned long *flags)
{
	dma_async_tx_callback callback;
	void *callback_param;

	callback = desc->async_tx.callback;
	callback_param = desc->async_tx.callback_param;
	if (callback) {
		spin_unlock_irqrestore(&chan->lock, *flags);
		callback(callback_param);
		spin_lock_irqsave(&chan->lock, *flags);
	}
}

/**
 * @brief Clean channel descriptors
 * if the DMA Cycle mode is enable, just do the callback to handle, ignore the completion
 * If the DMA Cycle mode is disable, just free the descriptor
 * 
 * @param chan 
 */
static void xilinx_dma_chan_desc_cleanup(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_tx_descriptor *desc, *next;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	list_for_each_entry_safe(desc, next, &chan->done_list, node) {
		struct dmaengine_result result;
		if (desc->cyclic) {
			/** 
			 * we don't free the descriptor in cyclic mode, 
			 * but we need to remove it from the done_list 
			 * and add it back to the active_list 
			 */

			// TODO(pan): check if there is any side effect
			list_move_tail(&desc->node, &chan->active_list);

			xilinx_dma_chan_handle_cyclic(chan, desc, &flags);
			break;
		}
		if (unlikely(desc->err)) {
			if (chan->direction == DMA_DEV_TO_MEM)
				result.result = DMA_TRANS_READ_FAILED;
			else
				result.result = DMA_TRANS_WRITE_FAILED;
		} else {
			result.result = DMA_TRANS_NOERROR;
		}

		/* Run the link descriptor callback function */
		result.residue = desc->residue;
		spin_unlock_irqrestore(&chan->lock, flags);
		dmaengine_desc_get_callback_invoke(&desc->async_tx, &result);

		/** 
		 * Remove from the list of running transactions , after the callback we may need the
		 * metadata of the descriptor, so we can't free it until the callback is done
		 */
		list_del(&desc->node);

		spin_lock_irqsave(&chan->lock, flags);

		/* Run any dependencies, then free the descriptor */
		dma_run_dependencies(&desc->async_tx);
		xilinx_dma_free_tx_descriptor(chan, desc);

		/**
		 * While we ran a callback the user called a terminate function,
		 * which takes care of cleaning up any remaining descriptors.
		 */

		if (chan->terminating)
			break;
	}
	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * @brief Schedule completion tasklet
 *
 * @param t 
 */
static void xilinx_dma_do_tasklet(struct tasklet_struct *t)
{
	struct xilinx_dma_chan *chan = from_tasklet(chan, t, tasklet);
	xilinx_dma_chan_desc_cleanup(chan);
}

/**
 * @brief Allocate channel resources 
 * 
 * @param dma_chan 
 * @return int 
 */
static int xilinx_dma_alloc_chan_resources(struct dma_chan *dma_chan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dma_chan);
	int i;

	/* allocate the buffer descriptors */
	chan->seg_v = dma_alloc_coherent(
		chan->dev, sizeof(*chan->seg_v) * XILINX_DMA_NUM_DESCS,
		&chan->seg_p, GFP_KERNEL);
	if (!chan->seg_v) {
		dev_err(chan->dev, "unable to allocate channel %d descriptors",
			chan->id);
		return -ENOMEM;
	}

	/**
	 * @brief For Cyclic DMA mode, we need to program the tail descriptor
	 * register with a value which is not a part of the BD chain.
	 * so allocating a desc segment during channel allocation for programming  
	 * tail descriptor
	 */
	chan->cyclic_seg_v =
		dma_alloc_coherent(chan->dev, sizeof(*chan->cyclic_seg_v),
				   &chan->cyclic_seg_p, GFP_KERNEL);

	if (!chan->cyclic_seg_v) {
		dev_err(chan->dev, "unable to allocate channel %d cyclic desc",
			chan->id);
		dma_free_coherent(chan->dev,
				  sizeof(*chan->seg_v) * XILINX_DMA_NUM_DESCS,
				  chan->seg_v, chan->seg_p);
		return -ENOMEM;
	}
	chan->cyclic_seg_v->phys = chan->cyclic_seg_p;
	for (i = 0; i < XILINX_DMA_NUM_DESCS; i++) {
		// chan->seg_v[i].phys = chan->seg_p + i * sizeof(*chan->seg_v);
		chan->seg_v[i].hw.next_desc = lower_32_bits(
			chan->seg_p + sizeof(*chan->seg_v) *
					      ((i + 1) * XILINX_DMA_NUM_DESCS));
		chan->seg_v[i].hw.next_desc_msb = upper_32_bits(
			chan->seg_p + sizeof(*chan->seg_v) *
					      ((i + 1) * XILINX_DMA_NUM_DESCS));
		chan->seg_v[i].phys = chan->seg_p + i * sizeof(*chan->seg_v);
		list_add_tail(&chan->seg_v[i].node, &chan->free_seg_list);
	}

	dma_cookie_init(dma_chan);

	/**
	 * @brief For AXI DMA resetting once channel will reset the
	 * other channel as well so enable the interrupts here.
	 */
	dma_ctrl_set(chan, XILINX_DMA_REG_DMACR, XILINX_DMA_DMAXR_ALL_IRQ_MASK);

	return 0;
}

/**
 * @brief Calculate the number of bytes to copy
 * 
 * @param chan 
 * @param size 
 * @param done 
 * @return int 
 */
static int xilinx_dma_calc_copysize(struct xilinx_dma_chan *chan, int size,
				    int done)
{
	size_t copy;
	copy = min_t(size_t, size - done, chan->xdev->max_buffer_len);

	if ((copy + done < size) && chan->xdev->common.copy_align) {
		copy = rounddown(copy, (1 << chan->xdev->common.copy_align));
	}
	return copy;
}

/**
 * @brief Get DMA transaction status
 * 
 * @param dma_chan 
 * @param cookie 
 * @param tx_state 
 * @return enum dma_status 
 */
static enum dma_status xilinx_dma_tx_status(struct dma_chan *dma_chan,
					    dma_cookie_t cookie,
					    struct dma_tx_state *tx_state)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dma_chan);
	struct xilinx_dma_tx_descriptor *desc;
	enum dma_status ret;
	unsigned long flags;
	u32 residue = 0;

	ret = dma_cookie_status(dma_chan, cookie, tx_state);
	if (ret == DMA_COMPLETE || !tx_state)
		return ret;

	spin_lock_irqsave(&chan->lock, flags);
	if (!list_empty(&chan->active_list)) {
		desc = list_last_entry(&chan->active_list,
				       struct xilinx_dma_tx_descriptor, node);
		if (chan->has_sg) {
			residue = xilinx_dma_get_residue(chan, desc);
		}
	}
	spin_unlock_irqrestore(&chan->lock, flags);

	dma_set_residue(tx_state, residue);

	return ret;
}

/**
 * @brief xilinx_dma_stop_transfer, Halt DMA channel
 * 
 * @param chan 
 * @return int 
 */
static int xilinx_dma_stop_transfer(struct xilinx_dma_chan *chan)
{
	u32 val;

	dma_ctrl_clr(chan, XILINX_DMA_REG_DMACR, XILINX_DMA_DMACR_RUNSTOP);

	/* wait for the hardware to halt */
	return xilinx_dma_poll_timeout(chan, XILINX_DMA_REG_DMASR, val,
				       val & XILINX_DMA_DMASR_HALTED, 0,
				       XILINX_DMA_LOOP_COUNT);
}

/**
 * @brief Start DMA channel
 * 
 * @param chan 
 */
static void xilinx_dma_start(struct xilinx_dma_chan *chan)
{
	int err;
	u32 val;

	dma_ctrl_set(chan, XILINX_DMA_REG_DMACR, XILINX_DMA_DMACR_RUNSTOP);

	err = xilinx_dma_poll_timeout(chan, XILINX_DMA_REG_DMASR, val,
				      !(val & XILINX_DMA_DMASR_IDLE), 0,
				      XILINX_DMA_LOOP_COUNT);
	if (err) {
		dev_err(chan->dev, "Cannot start channel %p: %x\n", chan,
			dma_ctrl_read(chan, XILINX_DMA_REG_DMASR));
		chan->err = true;
	}
}

/**
 * @brief xilinx_dma_start_transfer, Start DMA channel
 * 
 * @param chan, Driver specific DMA channel structure
 */
static void xilinx_dma_start_transfer(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_tx_descriptor *head_desc, *tail_desc;
	struct xilinx_axidma_tx_segment *tail_segment;
	u32 reg;

	if (chan->err)
		return;

	if (list_empty(&chan->pending_list)) {
		// FIXME: just for debug, remove it later
		return;
	}

	dev_info(chan->dev,
		 "No pending descriptors for channel %p, is idle %s\n", chan,
		 chan->idle ? "true" : "false");

	if (!chan->idle)
		return;

	head_desc = list_first_entry(&chan->pending_list,
				     struct xilinx_dma_tx_descriptor, node);
	tail_desc = list_last_entry(&chan->pending_list,
				    struct xilinx_dma_tx_descriptor, node);
	tail_segment = list_last_entry(&tail_desc->segments,
				       struct xilinx_axidma_tx_segment, node);
	reg = dma_ctrl_read(chan, XILINX_DMA_REG_DMACR);

	if (chan->desc_pendingcount <= XILINX_DMA_COALESCE_MAX) {
		reg &= ~XILINX_DMA_COALESCE_MAX;
		reg |= chan->desc_pendingcount << XILINX_DMA_CR_COALESCE_SHIFT;
		dma_ctrl_write(chan, XILINX_DMA_REG_DMACR, reg);
	}

	if (chan->has_sg)
		xilinx_write(chan, XILINX_DMA_REG_CURDESC,
			     head_desc->async_tx.phys);

	xilinx_dma_start(chan);

	if (chan->err)
		return;

	/* start transfer */

	if (chan->has_sg) {
		if (chan->cyclic)
			// to start cyclic transfer, set he tail descriptor to a detached descriptor
			xilinx_write(chan, XILINX_DMA_REG_TAILDESC,
				     chan->cyclic_seg_v->phys);
		else {
			// FIXME: just for debugging, we'll remove this later
			dev_info(
				chan->dev,
				"head segment phys: %x, tail segment phys: %x\n",
				head_desc->async_tx.phys, tail_segment->phys);

			xilinx_write(chan, XILINX_DMA_REG_TAILDESC,
				     tail_segment->phys);
		}
	} else {
		struct xilinx_axidma_tx_segment *segment;
		struct xilinx_axidma_desc_hw *hw;

		segment = list_first_entry(&head_desc->segments,
					   struct xilinx_axidma_tx_segment,
					   node);

		hw = &segment->hw;
		xilinx_write(chan, XILINX_DMA_REG_SRCDSTADDR,
			     xilinx_prep_dma_addr_t(hw->buf_addr));

		dma_ctrl_write(chan, XILINX_DMA_REG_BTT,
			       hw->control & chan->xdev->max_buffer_len);
	}

	list_splice_tail_init(&chan->pending_list, &chan->active_list);
	chan->desc_pendingcount = 0;
	chan->idle = false;
}

/**
 * @brief Issue pending transactions
 * 
 * @param dchan 
 */
static void xilinx_dma_issue_pending(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);
	chan->start_transfer(chan);
	spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * xilinx_dma_device_config - Configure the DMA channel
 * @dchan: DMA channel
 * @config: channel configuration
 */
static int xilinx_dma_device_config(struct dma_chan *dchan,
				    struct dma_slave_config *config)
{
	return 0;
}

/**
 * @brief Mark the active descriptor as completed 
 * 
 * CONTEXT: hardirq
 */
static void xilinx_dma_complete_descriptor(struct xilinx_dma_chan *chan)
{
	struct xilinx_dma_tx_descriptor *desc, *_desc;

	/* this function was invoked with lock held */
	if (list_empty(&chan->active_list))
		return;

	list_for_each_entry_safe(desc, _desc, &chan->active_list, node) {
		if (chan->has_sg)
			desc->residue = xilinx_dma_get_residue(chan, desc);
		else
			desc->residue = 0;
		desc->err = chan->err;

		list_del(&desc->node);
		if (!desc->cyclic)
			dma_cookie_complete(&desc->async_tx);
		list_add_tail(&desc->node, &chan->done_list);
	}
}

/**
 * @brief  Reset DMA channel
 * 
 * @param chan 
 * @return int 
 */
static int xilinx_dma_reset(struct xilinx_dma_chan *chan)
{
	int err;
	u32 tmp;

	dma_ctrl_set(chan, XILINX_DMA_REG_DMACR, XILINX_DMA_DMACR_RESET);

	/* wait for the hardware to finish reset */
	err = xilinx_dma_poll_timeout(chan, XILINX_DMA_REG_DMACR, tmp,
				      !(tmp & XILINX_DMA_DMACR_RESET), 0,
				      XILINX_DMA_LOOP_COUNT);
	if (err) {
		dev_err(chan->dev, "reset timeout, cr %x, sr %x\n",
			dma_ctrl_read(chan, XILINX_DMA_REG_DMACR),
			dma_ctrl_read(chan, XILINX_DMA_REG_DMASR));
		return -ETIMEDOUT;
	}

	chan->err = false;
	chan->idle = true;
	chan->desc_pendingcount = 0;
	chan->desc_submitcount = 0;

	return err;
}

/**
 * @brief  Reset DMA channel and enable irq
 * 
 * @param chan 
 * @return int , 0 on success
 */
static int xilinx_dma_chan_reset(struct xilinx_dma_chan *chan)
{
	int err;
	err = xilinx_dma_reset(chan);
	if (err)
		return err;

	dma_ctrl_set(chan, XILINX_DMA_REG_DMACR, XILINX_DMA_DMAXR_ALL_IRQ_MASK);

	return 0;
}

/**
 * @brief DMA interrupt handler
 * 
 * @param irq 
 * @param data 
 * @return irqreturn_t  IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t xilinx_dma_irq_handler(int irq, void *data)
{
	struct xilinx_dma_chan *chan = data;
	u32 status;

	/* Read status and ack the interrupts */
	status = dma_ctrl_read(chan, XILINX_DMA_REG_DMASR);
	if (!(status & XILINX_DMA_DMAXR_ALL_IRQ_MASK))
		return IRQ_NONE;

	dma_ctrl_write(chan, XILINX_DMA_REG_DMASR,
		       status & XILINX_DMA_DMAXR_ALL_IRQ_MASK);

	if (status & XILINX_DMA_DMASR_ERR_IRQ) {
		/*
		 * An error occurred. If C_FLUSH_ON_FSYNC is enabled and the
		 * error is recoverable, ignore it. Otherwise flag the error.
		 *
		 * Only recoverable errors can be cleared in the DMASR register,
		 * make sure not to write to other error bits to 1.
		 */
		u32 errors = status & XILINX_DMA_DMASR_ALL_ERR_MASK;
		dma_ctrl_write(chan, XILINX_DMA_REG_DMASR,
			       errors & XILINX_DMA_DMASR_ERR_RECOVER_MASK);
		if (!chan->flush_on_fsync ||
		    (errors & ~XILINX_DMA_DMASR_ERR_RECOVER_MASK)) {
			dev_err(chan->dev,
				"Channel %p has errors %x, cdr %x tdr %x\n",
				chan, errors,
				dma_ctrl_read(chan, XILINX_DMA_REG_CURDESC),
				dma_ctrl_read(chan, XILINX_DMA_REG_TAILDESC));
			chan->err = true;
		}
	}

	if (status & XILINX_DMA_DMASR_DLY_CNT_IRQ) {
		/* Device takes too long */
		dev_dbg(chan->dev, "Inter-packed latency too long\n");
	}

	// bit 12, ioc_irq, interrupt on complete
	if (status & XILINX_DMA_DMASR_FRM_CNT_IRQ) {
		spin_lock(&chan->lock);
		xilinx_dma_complete_descriptor(chan);
		chan->idle = true;
		chan->start_transfer(chan);
		spin_unlock(&chan->lock);
	}

	/**
	 * @brief schedule tasklet in high priority, cause during
	 * the tasklet we would do the callback and finish the bd's completion
	 * while at the master thread we're polling the bd's completion,
	 * so we need to make sure the tasklet as high priority as possible
	 */

	tasklet_hi_schedule(&chan->tasklet);
	return IRQ_HANDLED;
}

/**
 * @brief Queuing descriptor
 * 
 * @param chan 
 * @param desc 
 */
static void append_desc_queue(struct xilinx_dma_chan *chan,
			      struct xilinx_dma_tx_descriptor *desc)
{
	struct xilinx_dma_tx_descriptor *tail_desc;
	struct xilinx_axidma_tx_segment *tail_segment;

	if (list_empty(&chan->pending_list))
		goto append;

	tail_desc = list_last_entry(&chan->pending_list,
				    struct xilinx_dma_tx_descriptor, node);
	tail_segment = list_last_entry(&tail_desc->segments,
				       struct xilinx_axidma_tx_segment, node);
	tail_segment->hw.next_desc = cpu_to_le32(desc->async_tx.phys);
append:
	list_add_tail(&desc->node, &chan->pending_list);
	chan->desc_pendingcount++;
}

/**
 * @brief Submit DMA transaction
 * 
 * @param tx 
 * @return dma_cookie_t 
 */
static dma_cookie_t xilinx_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct xilinx_dma_tx_descriptor *desc = to_dma_tx_descriptor(tx);
	struct xilinx_dma_chan *chan = to_xilinx_chan(tx->chan);
	dma_cookie_t cookie;
	unsigned long flags;
	int err;

	if (chan->cyclic) {
		xilinx_dma_free_tx_descriptor(chan, desc);
		return -EBUSY;
	}

	if (chan->err) {
		err = xilinx_dma_chan_reset(chan);
		if (err < 0)
			return err;
	}

	spin_lock_irqsave(&chan->lock, flags);

	cookie = dma_cookie_assign(tx);

	append_desc_queue(chan, desc);

	if (desc->cyclic)
		chan->cyclic = true;
	chan->terminating = false;

	spin_unlock_irqrestore(&chan->lock, flags);

	return cookie;
}

// static void
static void *xilinx_get_metadata_ptr(struct dma_async_tx_descriptor *desc,
				     size_t *payload_len, size_t *max_len)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(desc->chan);
	struct xilinx_dma_tx_descriptor *cur_desc;
	struct xilinx_axidma_tx_segment *segment;

	if (!chan->has_sg)
		return NULL;

	// APP WORDs,
	*payload_len = XILINX_DMA_NUM_APP_WORDS;
	*max_len = XILINX_DMA_NUM_APP_WORDS;

	cur_desc = list_first_entry(&chan->done_list,
				    struct xilinx_dma_tx_descriptor, node);
	segment = list_first_entry(&cur_desc->segments,
				   struct xilinx_axidma_tx_segment, node);

	return segment->hw.app;
}

static struct dma_descriptor_metadata_ops xilinx_dma_desc_metadata_ops = {
	// DESC_METADATA_ENGINE, attach not implemented
	.attach = NULL,
	.get_ptr = xilinx_get_metadata_ptr,
	.set_len = NULL,
};

static struct dma_async_tx_descriptor *
xilinx_dma_prep_slave_sg(struct dma_chan *dchan, struct scatterlist *sgl,
			 unsigned int sg_len,
			 enum dma_transfer_direction direction,
			 unsigned long flags, void *context)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	struct xilinx_dma_tx_descriptor *desc;
	struct xilinx_axidma_tx_segment *segment = NULL;
	u32 *app_w = (u32 *)context;
	struct scatterlist *sg;
	size_t copy;
	size_t sg_used;
	unsigned int i;

	if (!is_slave_direction(direction))
		return NULL;

	desc = xilinx_dma_alloc_tx_descriptor(chan);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->async_tx, &chan->common);
	desc->async_tx.tx_submit = xilinx_dma_tx_submit;

	for_each_sg(sgl, sg, sg_len, i) {
		sg_used = 0;

		while (sg_used < sg_dma_len(sg)) {
			struct xilinx_axidma_desc_hw *hw;

			segment = xilinx_axidma_alloc_tx_segment(chan);
			if (!segment)
				goto error;
			copy = xilinx_dma_calc_copysize(chan, sg_dma_len(sg),
							sg_used);
			hw = &segment->hw;
			xilinx_axidma_buf(chan, hw, sg_dma_address(sg), sg_used,
					  0);
			hw->control = copy;

			if (chan->direction == DMA_MEM_TO_DEV) {
				if (app_w)
					memcpy(hw->app, app_w,
					       sizeof(u32) *
						       XILINX_DMA_NUM_APP_WORDS);
			}

			sg_used += copy;

			list_add_tail(&segment->node, &desc->segments);
		}
	}

	segment = list_first_entry(&desc->segments,
				   struct xilinx_axidma_tx_segment, node);
	desc->async_tx.phys = segment->phys;

	if (chan->direction == DMA_MEM_TO_DEV) {
		segment->hw.control |= XILINX_DMA_BD_SOP;
		segment = list_last_entry(
			&desc->segments, struct xilinx_axidma_tx_segment, node);
		segment->hw.control |= XILINX_DMA_BD_EOP;

	} else if (chan->direction == DMA_DEV_TO_MEM) {
		// NOTE!: metadata for
		desc->async_tx.desc_metadata_mode = DESC_METADATA_ENGINE;
		desc->async_tx.metadata_ops = &xilinx_dma_desc_metadata_ops;
	}

	return &desc->async_tx;
error:
	xilinx_dma_free_tx_descriptor(chan, desc);
	return NULL;
}

/**
 * @brief prepare descriptors for a DMA_SLAVE transaction
 * 
 * @param dchan: DMA Channel 
 * @param buf_addr: Physical address of the buffer
 * @param buf_len: Total length of the cyclic buffer
 * @param period_len: length of individual cyclic buffer
 * @param direction: DMA direction
 * @param flags: transfer ack flags
 * @return struct dma_async_tx_descriptor* 
 */
static struct dma_async_tx_descriptor *
xilinx_dma_prep_dma_cyclic(struct dma_chan *dchan, dma_addr_t buf_addr,
			   size_t buf_len, size_t period_len,
			   enum dma_transfer_direction direction,
			   unsigned long flags)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	struct xilinx_dma_tx_descriptor *desc;
	struct xilinx_axidma_tx_segment *segment, *head_segment, *prev = NULL;

	size_t copy, sg_used;
	unsigned int num_periods;
	int i;
	u32 reg;

	if (!period_len)
		return NULL;

	num_periods = buf_len / period_len;

	if (!num_periods)
		return NULL;

	if (!is_slave_direction(direction))
		return NULL;

	desc = xilinx_dma_alloc_tx_descriptor(chan);
	if (!desc)
		return NULL;

	chan->direction = direction;
	dma_async_tx_descriptor_init(&desc->async_tx, &chan->common);
	desc->async_tx.tx_submit = xilinx_dma_tx_submit;

	for (i = 0; i < num_periods; ++i) {
		sg_used = 0;

		while (sg_used < period_len) {
			struct xilinx_axidma_desc_hw *hw;

			segment = xilinx_axidma_alloc_tx_segment(chan);
			if (!segment)
				goto error;
			copy = xilinx_dma_calc_copysize(chan, period_len,
							sg_used);
			hw = &segment->hw;
			xilinx_axidma_buf(chan, hw, buf_addr, sg_used,
					  period_len * i);
			hw->control = copy;
			if (prev)
				prev->hw.next_desc = segment->phys;

			prev = segment;
			sg_used += copy;

			list_add_tail(&segment->node, &desc->segments);
		}
	}

	head_segment = list_first_entry(&desc->segments,
					struct xilinx_axidma_tx_segment, node);
	desc->async_tx.phys = head_segment->phys;
	desc->cyclic = true;
	reg = dma_ctrl_read(chan, XILINX_DMA_REG_DMACR);
	reg |= XILINX_DMA_CR_CYCLIC_BD_EN_MASK;
	dma_ctrl_write(chan, XILINX_DMA_REG_DMACR, reg);

	segment = list_last_entry(&desc->segments,
				  struct xilinx_axidma_tx_segment, node);
	segment->hw.next_desc = cpu_to_le32(
		head_segment->phys); // last segment points to first segment

	// FIXME: take period length into account
	if (direction == DMA_MEM_TO_DEV) {
		head_segment->hw.control |= XILINX_DMA_BD_SOP;
		segment->hw.control |= XILINX_DMA_BD_EOP;
	}
	return &desc->async_tx;
error:
	xilinx_dma_free_tx_descriptor(chan, desc);
	return NULL;
}

/**
 * @brief Halt the channel and free descriptors
 * 
 * @param dchan 
 * @return int 
 */
static int xilinx_dma_terminate_all(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	u32 reg;
	int err;

	if (!chan->cyclic) {
		err = chan->stop_transfer(chan);
		if (err) {
			dev_err(chan->dev, "Cannot stop channel %p: %x\n", chan,
				dma_ctrl_read(chan, XILINX_DMA_REG_DMASR));
			chan->err = true;
		}
	}

	xilinx_dma_chan_reset(chan);
	/* Remove and free all of the descriptors in the lists */
	chan->terminating = true;
	xilinx_dma_free_descriptors(chan);
	chan->idle = true;

	if (chan->cyclic) {
		reg = dma_ctrl_read(chan, XILINX_DMA_REG_DMACR);
		reg &= ~XILINX_DMA_CR_CYCLIC_BD_EN_MASK;
		dma_ctrl_write(chan, XILINX_DMA_REG_DMACR, reg);
		chan->cyclic = false;
	}

	return 0;
}

static void xilinx_dma_synchronize(struct dma_chan *dchan)
{
	struct xilinx_dma_chan *chan = to_xilinx_chan(dchan);
	tasklet_kill(&chan->tasklet);
}

/* -----------------------------------------------------------------------------
 * Probe and remove
 */

/**
 * xilinx_dma_chan_remove - Per Channel remove function
 * @chan: Driver specific DMA channel
 */
static void xilinx_dma_chan_remove(struct xilinx_dma_chan *chan)
{
	/* Disable all interrupts */
	dma_ctrl_clr(chan, XILINX_DMA_REG_DMACR, XILINX_DMA_DMAXR_ALL_IRQ_MASK);

	if (chan->irq > 0)
		free_irq(chan->irq, chan);

	tasklet_kill(&chan->tasklet);

	list_del(&chan->common.device_node);
}

static int axidma_clk_init(struct platform_device *pdev, struct clk **axi_clk,
			   struct clk **tx_clk, struct clk **rx_clk,
			   struct clk **sg_clk)
{
	int err;

	*axi_clk = devm_clk_get(&pdev->dev, "s_axi_lite_aclk");
	if (IS_ERR(*axi_clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(*axi_clk),
				     "failed to get axi_clock\n");
	*tx_clk = devm_clk_get(&pdev->dev, "m_axi_mm2s_aclk");
	if (IS_ERR(*tx_clk))
		*tx_clk = NULL;

	*rx_clk = devm_clk_get(&pdev->dev, "m_axi_s2mm_aclk");
	if (IS_ERR(*rx_clk))
		*rx_clk = NULL;

	*sg_clk = devm_clk_get(&pdev->dev, "m_axi_sg_aclk");
	if (IS_ERR(*sg_clk))
		*sg_clk = NULL;

	err = clk_prepare_enable(*axi_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable axi_clk (%d)\n", err);
		return err;
	}

	err = clk_prepare_enable(*tx_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable tx_clk (%d)\n", err);
		goto err_disable_axiclk;
	}

	err = clk_prepare_enable(*rx_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable rx_clk (%d)\n", err);
		goto err_disable_txclk;
	}

	err = clk_prepare_enable(*sg_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable sg_clk (%d)\n", err);
		goto err_disable_rxclk;
	}

	return 0;

err_disable_rxclk:
	clk_disable_unprepare(*rx_clk);
err_disable_txclk:
	clk_disable_unprepare(*tx_clk);
err_disable_axiclk:
	clk_disable_unprepare(*axi_clk);

	return err;
}

static void xdma_disable_all_clks(struct xilinx_dma_device *xdev)
{
	clk_disable_unprepare(xdev->rx_clk);
	clk_disable_unprepare(xdev->tx_clk);

	clk_disable_unprepare(xdev->sg_clk);
	clk_disable_unprepare(xdev->axi_clk);
}

/**
 * xilinx_dma_chan_probe - Per Channel Probing
 * It get channel features from the device tree entry and
 * initialize special channel handling routines
 *
 * @xdev: Driver specific device structure
 * @node: Device node
 *
 * Return: '0' on success and failure value on error
 */
static int xilinx_dma_chan_probe(struct xilinx_dma_device *xdev,
				 struct device_node *node)
{
	struct xilinx_dma_chan *chan;
	bool has_dre = false;
	u32 value, width;
	int err;

	/* Allocate and initialize the channel structure */
	chan = devm_kzalloc(xdev->dev, sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan->dev = xdev->dev;
	chan->xdev = xdev;
	chan->desc_pendingcount = 0x0;
	chan->ext_addr = xdev->ext_addr;
	/* This variable ensures that descriptors are not
	 * Submitted when dma engine is in progress. This variable is
	 * Added to avoid polling for a bit in the status register to
	 * Know dma state in the driver hot path.
	 */
	chan->idle = true;

	spin_lock_init(&chan->lock);
	INIT_LIST_HEAD(&chan->pending_list);
	INIT_LIST_HEAD(&chan->done_list);
	INIT_LIST_HEAD(&chan->active_list);
	INIT_LIST_HEAD(&chan->free_seg_list);

	/* Retrieve the channel properties from the device tree */
	has_dre = of_property_read_bool(node, "xlnx,include-dre");

	chan->genlock = of_property_read_bool(node, "xlnx,genlock-mode");

	err = of_property_read_u32(node, "xlnx,datawidth", &value);
	if (err) {
		dev_err(xdev->dev, "missing xlnx,datawidth property\n");
		return err;
	}
	width = value >> 3; /* Convert bits to bytes */

	/* If data width is greater than 8 bytes, DRE is not in hw */
	if (width > 8)
		has_dre = false;

	if (!has_dre)
		xdev->common.copy_align =
			(enum dmaengine_alignment)fls(width - 1);

	if (of_device_is_compatible(node, "xlnx,axi-dma-mm2s-channel")) {
		chan->direction = DMA_MEM_TO_DEV;
		chan->id = xdev->mm2s_chan_id++;
		chan->tdest = chan->id;
		chan->ctrl_offset = XILINX_DMA_MM2S_CTRL_OFFSET;
	} else if (of_device_is_compatible(node, "xlnx,axi-dma-s2mm-channel")) {
		chan->direction = DMA_DEV_TO_MEM;
		chan->id = xdev->s2mm_chan_id++;
		chan->tdest = chan->id - XILINX_DMA_MAX_CHANS_PER_DEVICE / 2;
		chan->ctrl_offset = XILINX_DMA_S2MM_CTRL_OFFSET;
	} else {
		dev_err(xdev->dev, "Invalid channel compatible node\n");
		return -EINVAL;
	}

	/* Request the interrupt */
	chan->irq = of_irq_get(node, chan->tdest);
	if (chan->irq < 0)
		return dev_err_probe(xdev->dev, chan->irq,
				     "failed to get irq\n");
	err = request_irq(chan->irq, xilinx_dma_irq_handler, IRQF_SHARED,
			  "xilinx-dma-controller", chan);
	if (err) {
		dev_err(xdev->dev, "unable to request IRQ %d\n", chan->irq);
		return err;
	}

	chan->start_transfer = xilinx_dma_start_transfer;
	chan->stop_transfer = xilinx_dma_stop_transfer;
	/* check if SG is enabled (only for AXIDMA, AXIMCDMA, and CDMA) */
	if (dma_ctrl_read(chan, XILINX_DMA_REG_DMASR) &
	    XILINX_DMA_DMASR_SG_MASK)
		chan->has_sg = true;
	dev_dbg(chan->dev, "ch %d: SG %s\n", chan->id,
		chan->has_sg ? "enabled" : "disabled");

	/* Initialize the tasklet */
	tasklet_setup(&chan->tasklet, xilinx_dma_do_tasklet);

	/*
	 * Initialize the DMA channel and add it to the DMA engine channels
	 * list.
	 */
	chan->common.device = &xdev->common;

	list_add_tail(&chan->common.device_node, &xdev->common.channels);
	xdev->chan[chan->id] = chan;

	/* Reset the channel */
	err = xilinx_dma_chan_reset(chan);
	if (err < 0) {
		dev_err(xdev->dev, "Reset channel failed\n");
		return err;
	}

	return 0;
}

/**
 * xilinx_dma_child_probe - Per child node probe
 * It get number of dma-channels per child node from
 * device-tree and initializes all the channels.
 *
 * @xdev: Driver specific device structure
 * @node: Device node
 *
 * Return: '0' on success and failure value on error.
 */
static int xilinx_dma_child_probe(struct xilinx_dma_device *xdev,
				  struct device_node *node)
{
	int ret, i;
	u32 nr_channels = 1;

	ret = of_property_read_u32(node, "dma-channels", &nr_channels);

	for (i = 0; i < nr_channels; i++) {
		ret = xilinx_dma_chan_probe(xdev, node);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * of_dma_xilinx_xlate - Translation function
 * @dma_spec: Pointer to DMA specifier as found in the device tree
 * @ofdma: Pointer to DMA controller data
 *
 * Return: DMA channel pointer on success and NULL on error
 */
static struct dma_chan *of_dma_xilinx_xlate(struct of_phandle_args *dma_spec,
					    struct of_dma *ofdma)
{
	struct xilinx_dma_device *xdev = ofdma->of_dma_data;
	int chan_id = dma_spec->args[0];

	if (chan_id >= XILINX_DMA_MAX_CHANS_PER_DEVICE || !xdev->chan[chan_id])
		return NULL;

	return dma_get_slave_channel(&xdev->chan[chan_id]->common);
}

static const struct of_device_id xilinx_dma_of_ids[] = {
	{ .compatible = "icnc,axi-dma" },
	{}
};
MODULE_DEVICE_TABLE(of, xilinx_dma_of_ids);

/**
 * @brief xilinx_dma_probe - Driver probe function
 * 
 * @param pdev 
 * @return int 
 */
static int xilinx_dma_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct xilinx_dma_device *xdev;
	struct device_node *child; //, *np = pdev->dev.of_node;
	u32 addr_width, len_width;
	int i, err;

	xdev = devm_kzalloc(&pdev->dev, sizeof(*xdev), GFP_KERNEL);
	if (!xdev)
		return -ENOMEM;

	xdev->dev = &pdev->dev;

	err = axidma_clk_init(pdev, &xdev->axi_clk, &xdev->tx_clk,
			      &xdev->rx_clk, &xdev->sg_clk);
	if (err)
		return err;

	xdev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(xdev->regs))
		return PTR_ERR(xdev->regs);

	/* Retrieve the DMA Engine properties from the device tree */
	xdev->max_buffer_len = GENMASK(XILINX_DMA_MAX_TRANS_LEN_MAX - 1, 0);
	xdev->s2mm_chan_id = XILINX_DMA_MAX_CHANS_PER_DEVICE / 2;

	if (!of_property_read_u32(node, "xlnx,sg-length-width", &len_width)) {
		if (len_width < XILINX_DMA_MAX_TRANS_LEN_MIN ||
		    len_width > XILINX_DMA_V2_MAX_TRANS_LEN_MAX) {
			dev_warn(
				xdev->dev,
				"invalid xlnx,sg-length-width property value. Using default width\n");
		} else {
			if (len_width > XILINX_DMA_MAX_TRANS_LEN_MAX)
				dev_warn(
					xdev->dev,
					"Please ensure that IP supports buffer length > 23 bits\n");
			xdev->max_buffer_len = GENMASK(len_width - 1, 0);
		}
	}

	err = of_property_read_u32(node, "xlnx,addrwidth", &addr_width);
	if (err < 0)
		dev_warn(xdev->dev, "missing xlnx,addrwidth property\n");

	if (addr_width > 32)
		xdev->ext_addr = true;
	else
		xdev->ext_addr = false;

	dma_set_mask_and_coherent(xdev->dev, DMA_BIT_MASK(addr_width));

	/* Initialize the DMA engine */
	xdev->common.dev = &pdev->dev;

	INIT_LIST_HEAD(&xdev->common.channels);

	dma_cap_set(DMA_SLAVE, xdev->common.cap_mask);
	dma_cap_set(DMA_PRIVATE, xdev->common.cap_mask);

	xdev->common.device_alloc_chan_resources =
		xilinx_dma_alloc_chan_resources;
	xdev->common.device_free_chan_resources =
		xilinx_dma_free_chan_resources;
	xdev->common.device_terminate_all = xilinx_dma_terminate_all;
	xdev->common.device_synchronize = xilinx_dma_synchronize;
	xdev->common.device_tx_status = xilinx_dma_tx_status;
	xdev->common.device_issue_pending = xilinx_dma_issue_pending;
	xdev->common.device_config = xilinx_dma_device_config;

	dma_cap_set(DMA_CYCLIC, xdev->common.cap_mask);
	xdev->common.device_prep_slave_sg = xilinx_dma_prep_slave_sg;
	xdev->common.device_prep_dma_cyclic = xilinx_dma_prep_dma_cyclic;
	/* Residue calculation is supported by only AXI DMA and CDMA */
	xdev->common.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;

	platform_set_drvdata(pdev, xdev);

	/* Initialize the channels */
	for_each_child_of_node(node, child) {
		err = xilinx_dma_child_probe(xdev, child);
		dev_err(xdev->dev, "xilinx_dma_child_probe err = %d", err);
		if (err < 0)
			goto disable_clks;
	}

	/* Register the DMA engine with the core */
	err = dma_async_device_register(&xdev->common);
	if (err) {
		dev_err(xdev->dev, "failed to register the dma device\n");
		goto error;
	}

	err = of_dma_controller_register(node, of_dma_xilinx_xlate, xdev);
	if (err < 0) {
		dev_err(&pdev->dev, "Unable to register DMA to DT\n");
		dma_async_device_unregister(&xdev->common);
		goto error;
	}
	xdev->is_coherent = of_dma_is_coherent(node);

	dev_info(&pdev->dev, "iCNC's Xilinx AXI DMA Engine Driver Probed!\n");
	return 0;

disable_clks:
	xdma_disable_all_clks(xdev);
error:
	for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++)
		if (xdev->chan[i])
			xilinx_dma_chan_remove(xdev->chan[i]);
	return err;
}
/**
 * xilinx_dma_remove - Driver remove function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: Always '0'
 */
static int xilinx_dma_remove(struct platform_device *pdev)
{
	struct xilinx_dma_device *xdev = platform_get_drvdata(pdev);
	int i;

	of_dma_controller_free(pdev->dev.of_node);

	dma_async_device_unregister(&xdev->common);

	for (i = 0; i < XILINX_DMA_MAX_CHANS_PER_DEVICE; i++)
		if (xdev->chan[i])
			xilinx_dma_chan_remove(xdev->chan[i]);

	xdma_disable_all_clks(xdev);

	return 0;
}

static struct platform_driver xilinx_dma_driver = {
	.driver = {
		.name = "icnc,axi-dma",
		.of_match_table = xilinx_dma_of_ids,
	},
	.probe = xilinx_dma_probe,
	.remove = xilinx_dma_remove,
};

module_platform_driver(xilinx_dma_driver);

MODULE_AUTHOR("ICNC, Inc.");
MODULE_DESCRIPTION("DMA Proxy Prototype");
MODULE_LICENSE("GPL v2");
