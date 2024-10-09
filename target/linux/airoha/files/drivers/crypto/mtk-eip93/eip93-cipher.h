/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2019 - 2021
 *
 * Richard van Schagen <vschagen@icloud.com>
 * Christian Marangi <ansuelsmth@gmail.com
 */
#ifndef _EIP93_CIPHER_H_
#define _EIP93_CIPHER_H_

#include "eip93-main.h"

struct mtk_crypto_ctx {
	struct mtk_device		*mtk;
	u32				flags;
	struct sa_record		*sa_record;
	u32				sa_nonce;
	int				blksize;
	dma_addr_t			sa_record_base;
	/* AEAD specific */
	unsigned int			authsize;
	unsigned int			assoclen;
	bool				set_assoc;
	enum mtk_alg_type		type;
};

struct mtk_cipher_reqctx {
	u16				desc_flags;
	u16				flags;
	unsigned int			blksize;
	unsigned int			ivsize;
	unsigned int			textsize;
	unsigned int			assoclen;
	unsigned int			authsize;
	dma_addr_t			sa_record_base;
	struct sa_state			*sa_state;
	dma_addr_t			sa_state_base;
	struct eip93_descriptor		*cdesc;
	struct scatterlist		*sg_src;
	struct scatterlist		*sg_dst;
	int				src_nents;
	int				dst_nents;
	struct sa_state			*sa_state_ctr;
	dma_addr_t			sa_state_ctr_base;
};

int check_valid_request(struct mtk_cipher_reqctx *rctx);

void mtk_unmap_dma(struct mtk_device *mtk, struct mtk_cipher_reqctx *rctx,
		   struct scatterlist *reqsrc, struct scatterlist *reqdst);

void mtk_skcipher_handle_result(struct crypto_async_request *async, int err);

int mtk_send_req(struct crypto_async_request *async,
		 const u8 *reqiv, struct mtk_cipher_reqctx *rctx);

void mtk_handle_result(struct mtk_device *mtk, struct mtk_cipher_reqctx *rctx,
		       u8 *reqiv);

#endif /* _EIP93_CIPHER_H_ */
