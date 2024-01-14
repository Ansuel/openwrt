/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */

#ifndef _DT_BINDINGS_CLOCK_AIROHA_EN7523_H_
#define _DT_BINDINGS_CLOCK_AIROHA_EN7523_H_

#define EN7523_CLK_GSW		0
#define EN7523_CLK_EMI		1
#define EN7523_CLK_BUS		2
#define EN7523_CLK_SLIC		3
#define EN7523_CLK_SPI		4
#define EN7523_CLK_NPU		5
#define EN7523_CLK_CRYPTO	6
#define EN7523_CLK_PCIE		7

#define EN7523_NUM_CLOCKS	8

u32 get_np_scu_data(u32 reg);
void set_np_scu_data(u32 reg, u32 val);

#endif /* _DT_BINDINGS_CLOCK_AIROHA_EN7523_H_ */
