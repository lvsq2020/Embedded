/*
 * xdma-core.c
 *
 *  Created on: 2018Äê2ÔÂ26ÈÕ
 *      Author: vefone
 */

#include "xdma-core.h"
#include <string.h>
#include <stdlib.h>

/* xdma_desc_control -- Set complete control field of a descriptor. */
void xdma_desc_control(struct xdma_desc *first, uint32_t control_field)
{
    /* remember magic and adjacent number */
    uint32_t control = first->control & ~(LS_BYTE_MASK);

    /* merge adjacent and control field */
    control |= control_field;
    /* write control and next_adjacent */
    first->control = control;
}

/* xdma_desc_clear -- Clear bits in control field of a descriptor. */
void xdma_desc_control_clear(struct xdma_desc *first, uint32_t clear_mask)
{
    /* remember magic and adjacent number */
    uint32_t control = first->control;

    /* merge adjacent and control field */
    control &= (~clear_mask);
    /* write control and next_adjacent */
    first->control = control;
}

/* xdma_desc_clear -- Set bits in control field of a descriptor. */
void xdma_desc_control_set(struct xdma_desc *first, uint32_t set_mask)
{
    /* remember magic and adjacent number */
    uint32_t control = first->control;

    /* merge adjacent and control field */
    control |= set_mask;
    /* write control and next_adjacent */
    first->control = control;
}

/* xdma_desc_alloc() - Allocate cache-coherent array of N descriptors.
 *
 * Allocates an array of 'number' descriptors in contiguous PCI bus addressable
 * memory. Chains the descriptors as a singly-linked list; the descriptor's
 * next * pointer specifies the bus address of the next descriptor.
 *
 *
 * @dev Pointer to pci_dev
 * @number Number of descriptors to be allocated
 * @desc_bus_p Pointer where to store the first descriptor bus address
 * @desc_last_p Pointer where to store the last descriptor virtual address,
 * or NULL.
 *
 * @return Virtual address of the first descriptor
 *
 */
struct xdma_desc *xdma_desc_alloc(int number,
                  uint64_t *desc_bus_p, struct xdma_desc **desc_last_p)
{
        struct xdma_desc *desc_virt;    /* virtual address */
        uint64_t desc_bus;            /* bus address */
        int i;
        int adj = number - 1;
        int extra_adj;
        uint32_t temp_control;

        /* allocate a set of cache-coherent contiguous pages */
        desc_virt = (struct xdma_desc *)malloc(number * sizeof(struct xdma_desc));
        if (!desc_virt)
                return NULL;
        /* get bus address of the first descriptor */
        desc_bus = *desc_bus_p;

        /* create singly-linked list for SG DMA controller */
        for (i = 0; i < number - 1; i++) {
                /* increment bus address to next in array */
                desc_bus += sizeof(struct xdma_desc);

                /* singly-linked list uses bus addresses */
                desc_virt[i].next_lo = PCI_DMA_L(desc_bus);
                desc_virt[i].next_hi = PCI_DMA_H(desc_bus);
                desc_virt[i].bytes = 0;

                /* any adjacent descriptors? */
                if (adj > 0) {
                        extra_adj = adj - 1;
                        if (extra_adj > MAX_EXTRA_ADJ)
                                extra_adj = MAX_EXTRA_ADJ;

                        adj--;
                } else {
                        extra_adj = 0;
                }

                temp_control = DESC_MAGIC | (extra_adj << 8);

                temp_control |= INSERT_DESC_COUNT(i);

                desc_virt[i].control = temp_control;
        }
        /* { i = number - 1 } */
        /* zero the last descriptor next pointer */
        desc_virt[i].next_lo = 0;
        desc_virt[i].next_hi = 0;
        desc_virt[i].bytes = 0;

        temp_control = DESC_MAGIC;

        temp_control |= INSERT_DESC_COUNT(i);

        desc_virt[i].control = temp_control;

        /* caller wants a pointer to last descriptor? */
        if (desc_last_p)
                *desc_last_p = desc_virt + i;

        /* return the virtual address of the first descriptor */
        return desc_virt;
}

/* xdma_desc_set() - Fill a descriptor with the transfer details
 *
 * @desc pointer to descriptor to be filled
 * @addr root complex address
 * @ep_addr end point address
 * @len number of bytes, must be a (non-negative) multiple of 4.
 * @dir_to_dev If non-zero, source is root complex address and destination
 * is the end point address. If zero, vice versa.
 *
 * Does not modify the next pointer
 */
void xdma_desc_set(struct xdma_desc *desc, uint64_t rc_bus_addr,
                          uint64_t ep_addr, int len, int dir_to_dev)
{
    desc->control = 0xad4b0013;

    /* transfer length */
    desc->bytes = len;
    if (dir_to_dev) {
        /* read from root complex memory (source address) */
        desc->src_addr_lo = PCI_DMA_L(rc_bus_addr);
        desc->src_addr_hi = PCI_DMA_H(rc_bus_addr);
        /* write to end point address (destination address) */
        desc->dst_addr_lo = PCI_DMA_L(ep_addr);
        desc->dst_addr_hi = PCI_DMA_H(ep_addr);
    } else {
        /* read from end point address (source address) */
        desc->src_addr_lo = PCI_DMA_L(ep_addr);
        desc->src_addr_hi = PCI_DMA_H(ep_addr);
        /* write to root complex memory (destination address) */
        desc->dst_addr_lo = PCI_DMA_L(rc_bus_addr);
        desc->dst_addr_hi = PCI_DMA_H(rc_bus_addr);
    }
}

void xdma_desc_set_source(struct xdma_desc *desc, uint64_t source)
{
    /* read from end point address (source address) */
    desc->src_addr_lo = PCI_DMA_L(source);
    desc->src_addr_hi = PCI_DMA_H(source);
}

