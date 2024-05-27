#!/bin/sh

DIR_ETH=/sys/kernel/debug/airoha_eth
DIR_SWITCH=/sys/kernel/debug/mt7530

# ETH QDMA
echo 0x4 > $DIR_ETH/regidx
echo -e "QDMA_GLB_CFG\t\t: $(cat $DIR_ETH/qdma_regval)"

echo 0x200 > $DIR_ETH/regidx
echo -e "CPU_RXR0_DSCP_BASE\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x204 > $DIR_ETH/regidx
echo -e "CPU_RXR0_RING_SIZE\t: $(cat $DIR_ETH/qdma_regval)"

echo 0x28 > $DIR_ETH/regidx
echo -e "INT_ENABLE_31_0\t\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x2c > $DIR_ETH/regidx
echo -e "INT_ENABLE_63_32\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x30 > $DIR_ETH/regidx
echo -e "INT2_ENABLE_31_0\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x34 > $DIR_ETH/regidx
echo -e "INT2_ENABLE_63_32\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x38 > $DIR_ETH/regidx
echo -e "INT3_ENABLE_31_0\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x3c > $DIR_ETH/regidx
echo -e "INT3_ENABLE_63_32\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x44 > $DIR_ETH/regidx
echo -e "INT4_Enable_31_0\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x48 > $DIR_ETH/regidx
echo -e "INT4_ENABLE_63_32\t: $(cat $DIR_ETH/qdma_regval)"

echo 0x108 > $DIR_ETH/regidx
echo -e "CPU_TXR0_CPU_IDX\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x10c > $DIR_ETH/regidx
echo -e "CPU_TXR0_DMA_IDX\t: $(cat $DIR_ETH/qdma_regval)"

echo 0x208 > $DIR_ETH/regidx
echo -e "CPU_RXR0_CPU_IDX\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x20c > $DIR_ETH/regidx
echo -e "CPU_RXR0_DMA_IDX\t: $(cat $DIR_ETH/qdma_regval)"

echo 0x20 > $DIR_ETH/regidx
echo -e "INT_STATUS_31_0\t\t: $(cat $DIR_ETH/qdma_regval)"
echo 0x24 > $DIR_ETH/regidx
echo -e "INT_STATUS_64_32\t: $(cat $DIR_ETH/qdma_regval)"

# ETH FE
echo 0x590 > $DIR_ETH/regidx
echo -e "CDM1_RXCPU_OK_CNT\t: $(cat $DIR_ETH/fe_regval)"
echo 0x5a0 > $DIR_ETH/regidx
echo -e "CDM1_RXCPU_DROP_CNT\t: $(cat $DIR_ETH/fe_regval)"

echo 0x660 > $DIR_ETH/regidx
echo -e "GDM1_RX_ETH_PKT_CNT\t: $(cat $DIR_ETH/fe_regval)"
echo 0x648 > $DIR_ETH/regidx
echo -e "GDM1_RX_OK_CNT\t\t: $(cat $DIR_ETH/fe_regval)"
echo 0x668 > $DIR_ETH/regidx
echo -e "GDM1_RX_ETH_DROP_CNT\t: $(cat $DIR_ETH/fe_regval)"

echo 0x610 > $DIR_ETH/regidx
echo -e "GDM1_TX_ETH_PKT_CNT\t: $(cat $DIR_ETH/fe_regval)"
echo 0x604 > $DIR_ETH/regidx
echo -e "GDM1_TX_OK_CNT_L\t: $(cat $DIR_ETH/fe_regval)"
echo 0x618 > $DIR_ETH/regidx
echo -e "GDM1_TX_ETH_DROP_CNT\t: $(cat $DIR_ETH/fe_regval)"

echo 0x50 > $DIR_ETH/regidx
echo -e "CDM1_OQ_MAP_0\t\t: $(cat $DIR_ETH/fe_regval)"
echo 0x54 > $DIR_ETH/regidx
echo -e "CDM1_OQ_MAP_1\t\t: $(cat $DIR_ETH/fe_regval)"
echo 0x58 > $DIR_ETH/regidx
echo -e "CDM1_OQ_MAP_2\t\t: $(cat $DIR_ETH/fe_regval)"
echo 0x5c > $DIR_ETH/regidx
echo -e "CDM1_OQ_MAP_3\t\t: $(cat $DIR_ETH/fe_regval)"

# DSA SWITCH
echo 0x7ffc > $DIR_SWITCH/regidx
echo -e "CREV\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x30f4 > $DIR_SWITCH/regidx
echo -e "GPINT_EN\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x3110 > $DIR_SWITCH/regidx
echo -e "PINT_EN_P1\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x3610 > $DIR_SWITCH/regidx
echo -e "PINT_EN_P6\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x30f8 > $DIR_SWITCH/regidx
echo -e "GPINT_STS\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x3114 > $DIR_SWITCH/regidx
echo -e "PINT_STS_P1\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x3614 > $DIR_SWITCH/regidx
echo -e "PINT_STS_P6\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4108 > $DIR_SWITCH/regidx
echo -e "TUPC_P1\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4100 > $DIR_SWITCH/regidx
echo -e "TDPC_P1\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4104 > $DIR_SWITCH/regidx
echo -e "TFPC_P1\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4168 > $DIR_SWITCH/regidx
echo -e "RUPC_P1\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4160 > $DIR_SWITCH/regidx
echo -e "RDPC_P1\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4164 > $DIR_SWITCH/regidx
echo -e "RFPC_P1\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4208 > $DIR_SWITCH/regidx
echo -e "TUPC_P2\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4200 > $DIR_SWITCH/regidx
echo -e "TDPC_P2\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4204 > $DIR_SWITCH/regidx
echo -e "TFPC_P2\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4308 > $DIR_SWITCH/regidx
echo -e "TUPC_P3\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4300 > $DIR_SWITCH/regidx
echo -e "TDPC_P3\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4304 > $DIR_SWITCH/regidx
echo -e "TFPC_P3\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4408 > $DIR_SWITCH/regidx
echo -e "TUPC_P4\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4400 > $DIR_SWITCH/regidx
echo -e "TDPC_P4\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4404 > $DIR_SWITCH/regidx
echo -e "TFPC_P4\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4508 > $DIR_SWITCH/regidx
echo -e "TUPC_P5\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4500 > $DIR_SWITCH/regidx
echo -e "TDPC_P5\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4504 > $DIR_SWITCH/regidx
echo -e "TFPC_P5\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4608 > $DIR_SWITCH/regidx
echo -e "TUPC_P6\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4600 > $DIR_SWITCH/regidx
echo -e "TDPC_P6\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4604 > $DIR_SWITCH/regidx
echo -e "TFPC_P6\t\t\t: $(cat $DIR_SWITCH/regval)"

echo 0x4668 > $DIR_SWITCH/regidx
echo -e "RUPC_P6\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4660 > $DIR_SWITCH/regidx
echo -e "RDPC_P6\t\t\t: $(cat $DIR_SWITCH/regval)"
echo 0x4664 > $DIR_SWITCH/regidx
echo -e "RFPC_P6\t\t\t: $(cat $DIR_SWITCH/regval)"
