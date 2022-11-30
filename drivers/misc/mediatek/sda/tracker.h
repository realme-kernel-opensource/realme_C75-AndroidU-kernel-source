/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __MTK_TRACKER_H__
#define __MTK_TRACKER_H__

#define INFRA_TRACKER_BASE          (0x10314000)

#define INFRA_TRACKER_CON           (infra_tracker_base + 0x0000)
#define INFRA_TRACKER_TIMEOUT_INFO  (infra_tracker_base + 0x0028)

/* Tracker INFO */
#define AR_TRACK_LOG_OFFSET         0x0200
#define AR_TRACK_L_OFFSET           0x0400
#define AR_TRACK_H_OFFSET           0x0600
#define AW_TRACK_LOG_OFFSET         0x0800
#define AW_TRACK_L_OFFSET           0x0A00
#define AW_TRACK_H_OFFSET           0x0C00
#define W_TRACK_DATA_VALID_OFFSET   0x0020

/* Infra tracker FAXI OFFSET */
#define INFRA_VALID_S               25
#define INFRA_VALID_E               25
#define INFRA_SECURE_S              24
#define INFRA_SECURE_E              24
#define INFRA_ID_S                   8
#define INFRA_ID_E                  23

/* After SPOID0567 the BUS_DBG_AR_TRACK_LOG offset has changed */
#define BYTE_OFF                    (4)

#define AR_TRACK_L(__base, __n)     (__base + AR_TRACK_L_OFFSET + BYTE_OFF * (__n))
#define AW_TRACK_L(__base, __n)     (__base + AW_TRACK_L_OFFSET + BYTE_OFF * (__n))
#define AR_TRACK_H(__base, __n)     (__base + AR_TRACK_H_OFFSET + BYTE_OFF * (__n))
#define AW_TRACK_H(__base, __n)     (__base + AW_TRACK_H_OFFSET + BYTE_OFF * (__n))

#define BUS_DBG_AR_TRACK_LOG(__base, __n)     (__base + AR_TRACK_LOG_OFFSET + BYTE_OFF * (__n))
#define BUS_DBG_AW_TRACK_LOG(__base, __n)     (__base + AW_TRACK_LOG_OFFSET + BYTE_OFF * (__n))

#define BUS_DBG_CON_IRQ_AR_STA0     (0x00000100)
#define BUS_DBG_CON_IRQ_AW_STA0     (0x00000200)
#define BUS_DBG_CON_IRQ_AR_STA1     (0x00100000)
#define BUS_DBG_CON_IRQ_AW_STA1     (0x00200000)
#define BUS_DBG_CON_TIMEOUT            (BUS_DBG_CON_IRQ_AR_STA0|BUS_DBG_CON_IRQ_AW_STA0| \
					BUS_DBG_CON_IRQ_AR_STA1|BUS_DBG_CON_IRQ_AW_STA1)

/* INFRA BUS TRACKER */
#define INFRA_ENTRY_NUM             (32)
#define BUSTRACKER_TIMEOUT          (0x300)

#define AR_STALL_TIMEOUT            (0x00000080)
#define AW_STALL_TIMEOUT            (0x00008000)
#define AR_RESP_ERR_TYPE_OFFSET     (24)
#define AR_RESP_ERR_CHECK_OFFSET    (25)
#define AW_RESP_ERR_TYPE_OFFSET     (26)
#define AW_RESP_ERR_CHECK_OFFSET    (27)
#define AR_RESP_ERR_CHECK_MASK      (0x02000000)
#define AR_RESP_ERR_TYPE_MASK       (0x03000000)
#define AW_RESP_ERR_CHECK_MASK      (0x08000000)
#define AW_RESP_ERR_TYPE_MASK       (0x0C000000)
#define DBG_SLV_CHECK               (AR_RESP_ERR_CHECK_MASK|AW_RESP_ERR_CHECK_MASK)
#define SLV_CHECK                   (1)
#define SLV_ERR                     (2)
#define DEC_ERR                     (3)

static void __iomem *infra_tracker_base;

static inline unsigned int extract_n2mbits(unsigned int input, unsigned int n, unsigned int m)
{
	/*
	 * 1. ~0 = 1111 1111 1111 1111 1111 1111 1111 1111
	 * 2. ~0 << (m - n + 1) = 1111 1111 1111 1111 1100 0000 0000 0000
	 * // assuming we are extracting 14 bits, the +1 is added for inclusive selection
	 * 3. ~(~0 << (m - n + 1)) = 0000 0000 0000 0000 0011 1111 1111 1111
	 */
	int mask;

	if (n > m) {
		n = n + m;
		m = n - m;
		n = n - m;
	}
	mask = ~(~0 << (m - n + 1));
	return (input >> n) & mask;
}
#endif // __MTK_TRACKER_H__
