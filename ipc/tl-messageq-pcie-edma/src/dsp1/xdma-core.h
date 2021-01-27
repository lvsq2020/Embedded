/*
 * xdma-core.h
 *
 *  Created on: 2018Äê2ÔÂ26ÈÕ
 *      Author: vefone
 */

#ifndef XDMA_CORE_H_
#define XDMA_CORE_H_

#include <stdint.h>

#define LS_BYTE_MASK 0x000000FFUL

/* for test purposes only, not in default IP! */
#define DESC_COUNTER 0

#define DESC_MAGIC 0xAD4B0000UL

#define MAX_EXTRA_ADJ (15)

/* obtain the 32 most significant (high) bits of a 32-bit or 64-bit address */
#define PCI_DMA_H(addr) ((addr >> 16) >> 16)
/* obtain the 32 least significant (low) bits of a 32-bit or 64-bit address */
#define PCI_DMA_L(addr) (addr & 0xffffffffUL)

#if DESC_COUNTER
    #define INSERT_DESC_COUNT(count) ((count & 0xf) << 12)
#else
    #define INSERT_DESC_COUNT(count) (0)
#endif

/**
 * Descriptor for a single contiguous memory block transfer.
 *
 * Multiple descriptors are linked by means of the next pointer. An additional
 * extra adjacent number gives the amount of extra contiguous descriptors.
 *
 * The descriptors are in root complex memory, and the bytes in the 32-bit
 * words must be in little-endian byte ordering.
 */
struct xdma_desc {
    uint32_t control;
    uint32_t bytes;              /* transfer length in bytes */
    uint32_t src_addr_lo;        /* source address (low 32-bit) */
    uint32_t src_addr_hi;        /* source address (high 32-bit) */
    uint32_t dst_addr_lo;        /* destination address (low 32-bit) */
    uint32_t dst_addr_hi;        /* destination address (high 32-bit) */
    /*
     * next descriptor in the single-linked list of descriptors;
     * this is the PCIe (bus) address of the next descriptor in the
     * root complex memory
     */
    uint32_t next_lo;            /* next desc address (low 32-bit) */
    uint32_t next_hi;            /* next desc address (high 32-bit) */
} __packed;

void xdma_desc_control(struct xdma_desc *first, uint32_t control_field);
void xdma_desc_control_clear(struct xdma_desc *first, uint32_t clear_mask);
void xdma_desc_control_set(struct xdma_desc *first, uint32_t set_mask);
struct xdma_desc *xdma_desc_alloc(int number,
                  uint64_t *desc_bus_p, struct xdma_desc **desc_last_p);
void xdma_desc_set(struct xdma_desc *desc, uint64_t rc_bus_addr,
                   uint64_t ep_addr, int len, int dir_to_dev);
void xdma_desc_set_source(struct xdma_desc *desc, uint64_t source);

#endif /* XDMA_CORE_H_ */
