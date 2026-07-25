/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    MACv2/hal_mac_lld.c
 * @brief   STM32 low level MAC driver code.
 *
 * @addtogroup MAC
 * @{
 */

#include <string.h>

#include "hal.h"

#if HAL_USE_MAC || defined(__DOXYGEN__)

#include "hal_mii.h"

/* Fixes for errors in ST headers.*/
#if ETH_DMADSR_RPS_FETCHING_Pos == 12
#undef ETH_DMADSR_RPS_FETCHING_Pos
#define ETH_DMADSR_RPS_FETCHING_Pos         (8U)
#endif

#if ETH_DMADSR_RPS_WAITING_Pos == 12
#undef ETH_DMADSR_RPS_WAITING_Pos
#define ETH_DMADSR_RPS_WAITING_Pos          (9U)
#endif

#if ETH_DMADSR_RPS_SUSPENDED_Pos == 14
#undef ETH_DMADSR_RPS_SUSPENDED_Pos
#define ETH_DMADSR_RPS_SUSPENDED_Pos        (10U)
#endif

#if ETH_DMADSR_RPS_CLOSING_Pos == 12
#undef ETH_DMADSR_RPS_CLOSING_Pos
#define ETH_DMADSR_RPS_CLOSING_Pos          (10U)
#endif

#if ETH_DMADSR_RPS_TIMESTAMP_WR_Pos == 13
#undef ETH_DMADSR_RPS_TIMESTAMP_WR_Pos
#undef ETH_DMADSR_RPS_TIMESTAMP_WR_Msk
#define ETH_DMADSR_RPS_TIMESTAMP_WR_Pos     (10U)
#define ETH_DMADSR_RPS_TIMESTAMP_WR_Msk     (0x6UL << ETH_DMADSR_RPS_TIMESTAMP_WR_Pos)
#endif

#if ETH_DMADSR_RPS_TRANSFERRING_Pos == 12
#undef ETH_DMADSR_RPS_TRANSFERRING_Pos
#define ETH_DMADSR_RPS_TRANSFERRING_Pos     (10U)
#endif

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define BUFFER_SIZE ((((STM32_MAC_BUFFERS_SIZE - 1) | 3) + 1) / 4)

/* Fixing inconsistencies in ST headers.*/
#if !defined(ETH_MACMDIOAR_CR_Div124) && defined(ETH_MACMDIOAR_CR_DIV124)
#define ETH_MACMDIOAR_CR_Div124 ETH_MACMDIOAR_CR_DIV124
#endif
#if !defined(ETH_MACMDIOAR_CR_Div102) && defined(ETH_MACMDIOAR_CR_DIV102)
#define ETH_MACMDIOAR_CR_Div102 ETH_MACMDIOAR_CR_DIV102
#endif
#if !defined(ETH_MACMDIOAR_CR_Div62) && defined(ETH_MACMDIOAR_CR_DIV62)
#define ETH_MACMDIOAR_CR_Div62 ETH_MACMDIOAR_CR_DIV62
#endif
#if !defined(ETH_MACMDIOAR_CR_Div42) && defined(ETH_MACMDIOAR_CR_DIV42)
#define ETH_MACMDIOAR_CR_Div42 ETH_MACMDIOAR_CR_DIV42
#endif
#if !defined(ETH_MACMDIOAR_CR_Div26) && defined(ETH_MACMDIOAR_CR_DIV26)
#define ETH_MACMDIOAR_CR_Div26 ETH_MACMDIOAR_CR_DIV26
#endif
#if !defined(ETH_MACMDIOAR_CR_Div16) && defined(ETH_MACMDIOAR_CR_DIV16)
#define ETH_MACMDIOAR_CR_Div16 ETH_MACMDIOAR_CR_DIV16
#endif

/* MII divider optimal value.*/
#if (STM32_HCLK > 300000000)
#error "STM32_HCLK above maximum frequency for ETH operations (300MHz)"
#elif (STM32_HCLK >= 250000000)
#define MACMDIODR_CR ETH_MACMDIOAR_CR_Div124
#elif (STM32_HCLK >= 150000000)
#define MACMDIODR_CR ETH_MACMDIOAR_CR_Div102
#elif (STM32_HCLK >= 100000000)
#define MACMDIODR_CR ETH_MACMDIOAR_CR_Div62
#elif (STM32_HCLK >= 60000000)
#define MACMDIODR_CR     ETH_MACMDIOAR_CR_Div42
#elif (STM32_HCLK >= 35000000)
#define MACMDIODR_CR     ETH_MACMDIOAR_CR_Div26
#elif (STM32_HCLK >= 20000000)
#define MACMDIODR_CR     ETH_MACMDIOAR_CR_Div16
#else
#error "STM32_HCLK below minimum frequency for ETH operations (20MHz)"
#endif

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Ethernet driver 1.
 */
MACDriver ETHD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static const uint8_t default_mac_address[] = {0xAA, 0x55, 0x13,
                                              0x37, 0x01, 0x10};

#if defined(STM32_ETH_BUFFERS_EXTERN)
extern stm32_eth_rx_descriptor_t *__eth_rd;
extern stm32_eth_tx_descriptor_t *__eth_td;
extern uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
extern uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];
#else
static stm32_eth_rx_descriptor_t __eth_rd[STM32_MAC_RECEIVE_BUFFERS]
                        __attribute__((aligned(4), __section__(".eth")));
static stm32_eth_tx_descriptor_t __eth_td[STM32_MAC_TRANSMIT_BUFFERS]
                        __attribute__((aligned(4), __section__(".eth")));

static uint32_t __eth_rb[STM32_MAC_RECEIVE_BUFFERS][BUFFER_SIZE]
                        __attribute__((aligned(4), __section__(".eth")));
static uint32_t __eth_tb[STM32_MAC_TRANSMIT_BUFFERS][BUFFER_SIZE]
                        __attribute__((aligned(4), __section__(".eth")));
#endif
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Writes a PHY register.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[in] reg       register number
 * @param[in] value     new register value
 *
 * @notapi
 */
void mii_write(MACDriver *macp, uint32_t reg, uint32_t value) {

  ETH->MACMDIODR = value;
  ETH->MACMDIOAR = macp->phyaddr | (reg << ETH_MACMDIOAR_RDA_Pos) | MACMDIODR_CR |
                  ETH_MACMDIOAR_MOC_WR | ETH_MACMDIOAR_MB;
  while ((ETH->MACMDIOAR & ETH_MACMDIOAR_MB) != 0)
    ;
}

/**
 * @brief   Reads a PHY register.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[in] reg       register number
 *
 * @return              The PHY register content.
 *
 * @notapi
 */
uint32_t mii_read(MACDriver *macp, uint32_t reg) {

  ETH->MACMDIOAR = macp->phyaddr | (reg << ETH_MACMDIOAR_RDA_Pos) | MACMDIODR_CR |
                  ETH_MACMDIOAR_MOC_RD | ETH_MACMDIOAR_MB;
  while ((ETH->MACMDIOAR & ETH_MACMDIOAR_MB) != 0)
    ;
  return ETH->MACMDIODR;
}

#if !defined(BOARD_PHY_ADDRESS)
/**
 * @brief   PHY address detection.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 */
static bool mii_find_phy(MACDriver *macp) {
  uint32_t i;

#if STM32_MAC_PHY_TIMEOUT > 0
  unsigned n = STM32_MAC_PHY_TIMEOUT;
 do {
#endif
    for (i = 0U; i <= 31U; i++) {
      macp->phyaddr = i << ETH_MACMDIOAR_PA_Pos;
      ETH->MACMDIOAR = (i << ETH_MACMDIOAR_RDA_Pos) | MACMDIODR_CR;
      ETH->MACMDIODR = (i << ETH_MACMDIODR_RA_Pos) | MACMDIODR_CR;
      if ((mii_read(macp, MII_PHYSID1) == (BOARD_PHY_ID >> 16U)) &&
          ((mii_read(macp, MII_PHYSID2) & 0xFFF0U) == (BOARD_PHY_ID & 0xFFF0U))) {
        return true;
      }
    }
#if STM32_MAC_PHY_TIMEOUT > 0
    n--;
  } while (n > 0U);
#endif
 return false;
}
#endif

/**
 * @brief   MAC address setup.
 *
 * @param[in] p         pointer to a six bytes buffer containing the MAC
 *                      address
 */
static void mac_lld_set_address(const uint8_t *p) {

  /* MAC address configuration, only a single address comparator is used,
     hash table not used.*/
  ETH->MACA0HR   = ((uint32_t)p[5] << 8) |
                   ((uint32_t)p[4] << 0);
  ETH->MACA0LR   = ((uint32_t)p[3] << 24) |
                   ((uint32_t)p[2] << 16) |
                   ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[0] << 0);
  ETH->MACA1HR   = 0;
  ETH->MACA1LR   = 0;
  ETH->MACA2HR   = 0;
  ETH->MACA2LR   = 0;
  ETH->MACA3HR   = 0;
  ETH->MACA3LR   = 0;
  ETH->MACHT0R   = 0;
  ETH->MACHT1R   = 0;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

OSAL_IRQ_HANDLER(STM32_ETH_HANDLER) {
  MACDriver *macp = &ETHD1;
  uint32_t dmacsr;

  OSAL_IRQ_PROLOGUE();

  dmacsr = ETH->DMACSR;
  ETH->DMACSR = dmacsr; /* Clear status bits.*/

  if ((dmacsr & (ETH_DMACSR_RI | ETH_DMACSR_TI)) != 0U) {
    if ((dmacsr & ETH_DMACSR_TI) != 0U) {
      /* Data Transmitted.*/
      __mac_tx_wakeup(macp);
    }

    if ((dmacsr & ETH_DMACSR_RI) != 0U) {
      /* Data Received.*/
      __mac_rx_wakeup(macp);
    }

    __mac_callback(macp);
  }

  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level MAC initialization.
 *
 * @notapi
 */
bool mac_lld_init(void) {
  unsigned i;

  if (__eth_rb == NULL || __eth_tb == NULL || __eth_rd == NULL || __eth_td == NULL) {
    return false;
  }

  macObjectInit(&ETHD1);
  ETHD1.link_up = false;

  /* Descriptor tables are initialized in ring mode, note that the first
     word is not initialized here but in mac_lld_start().*/
  for (i = 0; i < STM32_MAC_RECEIVE_BUFFERS; i++) {
    __eth_rd[i].rdes0 = (uint32_t)__eth_rb[i];
    __eth_rd[i].rdes1 = 0;
    __eth_rd[i].rdes2 = 0;
    __eth_rd[i].rdes3 = STM32_RDES3_OWN | STM32_RDES3_IOC | STM32_RDES3_BUF1V;
  }
  for (i = 0; i < STM32_MAC_TRANSMIT_BUFFERS; i++) {
    __eth_td[i].tdes0 = 0;
    __eth_td[i].tdes1 = 0;
    __eth_td[i].tdes2 = 0;
    __eth_td[i].tdes3 = 0;
  }

  /* Selection of the RMII or MII mode based on info exported by board.h.*/
#if defined(STM32H7XX)
  {
    uint32_t pmcr = SYSCFG->PMCR & ~SYSCFG_PMCR_EPIS_SEL_Msk;
#if defined(BOARD_PHY_RMII)
  pmcr |= SYSCFG_PMCR_EPIS_SEL_2;
#endif
  SYSCFG->PMCR = pmcr;
}

#elif defined(STM32H5XX)
  {
    uint32_t pmcr = SBS->PMCR & ~SBS_PMCR_ETH_SEL_PHY_Msk;
#if defined(BOARD_PHY_RMII)
    pmcr |= SBS_PMCR_ETH_SEL_PHY_2;
#endif
    SBS->PMCR = pmcr;
  }

#else
#error "unsupported STM32 platform for MACv2 driver"
#endif

  /* Reset of the MAC core.*/
  rccResetETH();

  /* MAC clocks temporary activation.*/
  rccEnableETH(true);

  /* PHY address setup.*/
#if defined(BOARD_PHY_ADDRESS)
  ETHD1.phyaddr = BOARD_PHY_ADDRESS << ETH_MACMDIOAR_PA_Pos;
#else
  if (!mii_find_phy(&ETHD1)) {
      rccDisableETH();
      return false;
  }
#endif

#if defined(BOARD_PHY_RESET)
  /* PHY board-specific reset procedure.*/
  BOARD_PHY_RESET();
#else
  /* PHY soft reset procedure.*/
  mii_write(&ETHD1, MII_BMCR, BMCR_RESET);
#if defined(BOARD_PHY_RESET_DELAY)
  osalSysPolledDelayX(BOARD_PHY_RESET_DELAY);
#endif
  while (mii_read(&ETHD1, MII_BMCR) & BMCR_RESET)
    ;
#endif

#if STM32_MAC_ETH1_CHANGE_PHY_STATE
  /* PHY in power down mode until the driver will be started.*/
  mii_write(&ETHD1, MII_BMCR, mii_read(&ETHD1, MII_BMCR) | BMCR_PDOWN);
#endif

  /* MAC clocks stopped again.*/
  rccDisableETH();

  return true;
}

/**
 * @brief   Configures and activates the MAC peripheral.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
void mac_lld_start(MACDriver *macp) {
  unsigned i;

  /* Resets the state of all descriptors.*/
  for (i = 0; i < STM32_MAC_RECEIVE_BUFFERS; i++) {
    __eth_rd[i].rdes3 = STM32_RDES3_OWN | STM32_RDES3_IOC | STM32_RDES3_BUF1V;
  }
  for (i = 0; i < STM32_MAC_TRANSMIT_BUFFERS; i++) {
    __eth_td[i].tdes3 = 0U;
  }

  /* MAC clocks activation and commanded reset procedure.*/
  rccEnableETH(true);

  /* ISR vector enabled.*/
  nvicEnableVector(STM32_ETH_NUMBER, STM32_MAC_ETH1_IRQ_PRIORITY);

#if STM32_MAC_ETH1_CHANGE_PHY_STATE
  /* PHY in power up mode.*/
  mii_write(macp, MII_BMCR, mii_read(macp, MII_BMCR) & ~BMCR_PDOWN);
#endif

  ETH->DMAMR |= ETH_DMAMR_SWR;
  while (ETH->DMAMR & ETH_DMAMR_SWR)
    ;

  /* MAC configuration.*/
  ETH->MACCR   = ETH_MACCR_DO;
  ETH->MACPFR  = 0U;
  ETH->MACTFCR = 0U;
  ETH->MACRFCR = 0U;
  ETH->MACVTR  = 0U;

  /* MAC address setup.*/
  if (macp->config->mac_address == NULL)
    mac_lld_set_address(default_mac_address);
  else
    mac_lld_set_address(macp->config->mac_address);

  /* Transmitter and receiver enabled.
     Note that the complete setup of the MAC is performed when the link
     status is detected.*/
#if STM32_MAC_IP_CHECKSUM_OFFLOAD
  ETH->MACCR |= ETH_MACCR_IPC | ETH_MACCR_RE | ETH_MACCR_TE;
#else
  ETH->MACCR |=                 ETH_MACCR_RE | ETH_MACCR_TE;
#endif

  /* MMC configuration:
     Disable all interrupts.*/
  ETH->MMCTIMR   = (1<<27) | (1<<26) | (1<<21) | (1<<15) | (1<<14);
  ETH->MMCRIMR   = (1<<27) | (1<<26) | (1<<17) | (1<<6)  | (1<<5);

  /* DMA general settings.*/
  ETH->DMASBMR   = ETH_DMASBMR_AAL;
  ETH->DMACCR    = ETH_DMACCR_DSL_0BIT;
  ETH->DMAMR     = ETH_DMAMR_INTM_0 | ETH_DMAMR_PR_8_1 | ETH_DMAMR_TXPR;  /* TX:RX 8:1 */

  /* DMA configuration:
     Descriptor rings pointers.*/
  ETH->DMACTDLAR = (uint32_t)&__eth_td[0];
  ETH->DMACTDRLR = STM32_MAC_TRANSMIT_BUFFERS-1;
  ETH->DMACRDLAR = (uint32_t)&__eth_rd[0];
  ETH->DMACRDRLR = STM32_MAC_RECEIVE_BUFFERS-1;

  /* Enabling required interrupt sources.*/
  ETH->DMACSR    = ETH_DMACSR_NIS;
  ETH->DMACIER   = ETH_DMACIER_NIE | ETH_DMACIER_RIE | ETH_DMACIER_TIE;

  /* Check because errata on some devices. There should be no need to
     disable flushing because the TXFIFO should be empty on macStart().*/
#if !defined(STM32_MAC_DISABLE_TX_FLUSH)
  /* Transmit FIFO flush.*/
  ETH->MTLTQOMR  = ETH_MTLTQOMR_FTQ;
  while (ETH->MTLTQOMR & ETH_MTLTQOMR_FTQ)
    ;
#endif

  /* DMA final configuration and start.*/
  ETH->MTLRQOMR  = ETH_MTLRQOMR_DISTCPEF | ETH_MTLRQOMR_RSF;
  ETH->MTLTQOMR  = ETH_MTLTQOMR_TSF;
  ETH->DMACTCR   = ETH_DMACTCR_ST | ETH_DMACTCR_TPBL_1PBL;
  ETH->DMACRCR   = ETH_DMACRCR_SR | ETH_DMACRCR_RPBL_1PBL |
                   (STM32_MAC_BUFFERS_SIZE << ETH_DMACRCR_RBSZ_Pos);
}

/**
 * @brief   Deactivates the MAC peripheral.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 *
 * @notapi
 */
void mac_lld_stop(MACDriver *macp) {

  if (macp->state != MAC_STOP) {
#if STM32_MAC_ETH1_CHANGE_PHY_STATE
    /* PHY in power down mode until the driver will be restarted.*/
    mii_write(macp, MII_BMCR, mii_read(macp, MII_BMCR) | BMCR_PDOWN);
#endif

    /* MAC and DMA stopped.*/
    ETH->MACCR    = 0U;
    ETH->MTLRQOMR = 0U;
    ETH->DMACIER  = 0U;
    ETH->DMACSR   = ETH->DMACSR;

    /* MAC clocks stopped.*/
    rccDisableETH();

    /* ISR vector disabled.*/
    nvicDisableVector(STM32_ETH_NUMBER);
  }
}

/**
 * @brief   Returns a transmission descriptor.
 * @details One of the available transmission descriptors is locked and
 *          returned.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[out] tdp      pointer to a @p MACTransmitDescriptor structure
 * @return              The operation status.
 * @retval MSG_OK       the descriptor has been obtained.
 * @retval MSG_TIMEOUT  descriptor not available.
 *
 * @notapi
 */
msg_t mac_lld_get_transmit_descriptor(MACDriver *macp,
                                      MACTransmitDescriptor *tdp) {
  stm32_eth_tx_descriptor_t *current_tdes;
  unsigned i;

  if (!macp->link_up)
    return MSG_TIMEOUT;

  /* Scanning for all descriptors ahead of the current tail pointer.*/
  current_tdes = (stm32_eth_tx_descriptor_t *)((uint32_t)&__eth_td[0] + ETH->DMACTDTPR);
  for (i = 0U; i < STM32_MAC_TRANSMIT_BUFFERS; i++) {

    /* Skipping descriptors that are locked and already owned by the DMA.*/
    if (((current_tdes->tdes3 & STM32_TDES3_OWN) == 0U) &&
        (current_tdes->tdes1 == 0U)) {

      /* Assigning the buffer and marking the descriptor as locked.*/
      current_tdes->tdes0 = (uint32_t)__eth_tb[current_tdes - &__eth_td[0]];
      current_tdes->tdes1 = STM32_TDES1_LOCKED;

      /* Set the buffer size and configuration.*/
      tdp->offset   = 0U;
      tdp->size     = STM32_MAC_BUFFERS_SIZE;
      tdp->physdesc = current_tdes;

      return MSG_OK;
    }

    /* Pointing on the next descriptor.*/
    current_tdes++;
    if (current_tdes >= &__eth_td[STM32_MAC_TRANSMIT_BUFFERS]) {
      current_tdes = &__eth_td[0];
    }
  }

  return MSG_TIMEOUT;
}

/**
 * @brief   Releases a transmit descriptor and starts the transmission of the
 *          enqueued data as a single frame.
 *
 * @param[in] tdp       the pointer to the @p MACTransmitDescriptor structure
 *
 * @notapi
 */
void mac_lld_release_transmit_descriptor(MACTransmitDescriptor *tdp) {
  stm32_eth_tx_descriptor_t *tdes = tdp->physdesc;

  osalDbgAssert((tdes->tdes3 & STM32_TDES3_OWN) == 0U,
              "attempt to release descriptor already owned by DMA");

  /* Give buffer back to the Ethernet DMA.*/
  tdes->tdes1 = 0U;
  tdes->tdes2 = STM32_TDES2_IOC | (tdp->offset & STM32_TDES2_B1L_MASK);
#if STM32_MAC_IP_CHECKSUM_OFFLOAD
  tdes->tdes3 = STM32_TDES3_CIC(STM32_MAC_IP_CHECKSUM_OFFLOAD) |
                STM32_TDES3_LD | STM32_TDES3_FD | STM32_TDES3_OWN;
#else
  tdes->tdes3 = STM32_TDES3_LD | STM32_TDES3_FD | STM32_TDES3_OWN;
#endif

  /* Pointing on the next descriptor.*/
  tdes = tdes + 1;
  if (tdes >= &__eth_td[STM32_MAC_TRANSMIT_BUFFERS]) {
    tdes = &__eth_td[0];
  }

  /* Wait for the write to tdes3 to go through before resuming the DMA.*/
  __DSB();

  /* Triggering the TX DMA on release.*/
  ETH->DMACTDTPR = (uint32_t)tdes - (uint32_t)&__eth_td[0];
}

/**
 * @brief   Returns a receive descriptor.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @param[out] rdp      pointer to a @p MACReceiveDescriptor structure
 * @return              The operation status.
 * @retval MSG_OK       the descriptor has been obtained.
 * @retval MSG_TIMEOUT  descriptor not available.
 *
 * @notapi
 */
msg_t mac_lld_get_receive_descriptor(MACDriver *macp,
                                     MACReceiveDescriptor *rdp) {
  stm32_eth_rx_descriptor_t *current_rdes;
  unsigned i;

  (void)macp;

  /* Scanning for all descriptors ahead of the current tail pointer.*/
  current_rdes = (stm32_eth_rx_descriptor_t *)((uint32_t)&__eth_rd[0] + ETH->DMACRDTPR);
  for (i = 0U; i < STM32_MAC_RECEIVE_BUFFERS; i++) {
    stm32_eth_rx_descriptor_t *next_rdes;

    /* Pointing on the next descriptor.*/
    next_rdes = current_rdes + 1;
    if (next_rdes >= &__eth_rd[STM32_MAC_RECEIVE_BUFFERS]) {
      next_rdes = &__eth_rd[0];
    }

    /* Is the descriptor not owned by DMA?*/
    if ((current_rdes->rdes3 & STM32_RDES3_OWN) == 0U) {

      /* Yes, checking if it is a frame to be processed.*/
      uint32_t rdes3 = current_rdes->rdes3 & (STM32_RDES3_ES |
                                              STM32_RDES3_FD |
                                              STM32_RDES3_LD);
      if ((rdes3 == (STM32_RDES3_FD | STM32_RDES3_LD)) &&
#if STM32_MAC_IP_CHECKSUM_OFFLOAD
          ((current_rdes->rdes1 & (STM32_RDES1_IPHE | STM32_RDES1_IPCE)) == 0U) &&
#endif
          ((current_rdes->rdes2 & STM32_RDES2_DAF) == 0U)) {

        /* Found a valid one.*/
        rdp->offset   = 0U;
        rdp->size     = (current_rdes->rdes3 & STM32_RDES3_PL_MASK) -2; /* Lose CRC.*/
        rdp->physdesc = current_rdes;

        /* Moving the tail pointer, this also wakes the DMA up.*/
        ETH->DMACRDTPR = (uint32_t)next_rdes - (uint32_t)&__eth_rd[0];

        return MSG_OK;
      }

      /* Other descriptor type, returning it to DMA without processing.*/
      current_rdes->rdes3 = STM32_RDES3_OWN | STM32_RDES3_IOC | STM32_RDES3_BUF1V;

      /* Pointing on the next descriptor.*/
      current_rdes = next_rdes;
    }
  }

  return MSG_TIMEOUT;
}

/*
 * @brief   Releases a receive descriptor.
 * @details The descriptor and its buffer are made available for more incoming
 *          frames.
 *
 * @param[in] rdp       the pointer to the @p MACReceiveDescriptor structure
 *
 * @notapi
 */
void mac_lld_release_receive_descriptor(MACReceiveDescriptor *rdp) {

  osalDbgAssert((rdp->physdesc->rdes3 & STM32_RDES3_OWN) == 0U,
                "attempt to release descriptor already owned by DMA");

  /* Give buffer back to the Ethernet DMA.*/
  rdp->physdesc->rdes3 = STM32_RDES3_OWN | STM32_RDES3_IOC | STM32_RDES3_BUF1V;

  /* Re-triggering the DMA, in case in case it went in suspend mode before
     a found frame was released and the ring is full.*/
  ETH->DMACRDTPR = ETH->DMACRDTPR;
}

/**
 * @brief   Updates and returns the link status.
 *
 * @param[in] macp      pointer to the @p MACDriver object
 * @return              The link status.
 * @retval true         if the link is active.
 * @retval false        if the link is down.
 *
 * @notapi
 */
bool mac_lld_poll_link_status(MACDriver *macp) {
#if STM32_MAC_PHY_LINK_TYPE == MAC_LINK_DYNAMIC
  uint32_t maccr, bmsr, bmcr;

  maccr = ETH->MACCR;

  /* PHY CR and SR registers read.*/
  (void)mii_read(macp, MII_BMSR);
  bmsr = mii_read(macp, MII_BMSR);
  bmcr = mii_read(macp, MII_BMCR);

  /* Check on auto-negotiation mode.*/
  if (bmcr & BMCR_ANENABLE) {
    uint32_t lpa;

    /* Auto-negotiation must be finished without faults and link established.*/
    if ((bmsr & (BMSR_LSTATUS | BMSR_RFAULT | BMSR_ANEGCOMPLETE)) !=
        (BMSR_LSTATUS | BMSR_ANEGCOMPLETE))
      return macp->link_up = false;

    /* Auto-negotiation enabled, checks the LPA register.*/
    lpa = mii_read(macp, MII_LPA);

    /* Check on link speed.*/
    if (lpa & (LPA_100HALF | LPA_100FULL | LPA_100BASE4))
      maccr |= ETH_MACCR_FES;
    else
      maccr &= ~ETH_MACCR_FES;

    /* Check on link mode.*/
    if (lpa & (LPA_10FULL | LPA_100FULL))
      maccr |= ETH_MACCR_DM;
    else
      maccr &= ~ETH_MACCR_DM;
  }
  else {
    /* Link must be established.*/
    if (!(bmsr & BMSR_LSTATUS))
      return macp->link_up = false;

    /* Check on link speed.*/
    if (bmcr & BMCR_SPEED100)
      maccr |= ETH_MACCR_FES;
    else
      maccr &= ~ETH_MACCR_FES;

    /* Check on link mode.*/
    if (bmcr & BMCR_FULLDPLX)
      maccr |= ETH_MACCR_DM;
    else
      maccr &= ~ETH_MACCR_DM;
  }

#elif STM32_MAC_PHY_LINK_TYPE == MAC_LINK_100_FULLDUPLEX
  uint32_t maccr = ETH->MACCR;

  maccr |= ETH_MACCR_FES;
  maccr |= ETH_MACCR_DM;

#elif STM32_MAC_PHY_LINK_TYPE == MAC_LINK_10_FULLDUPLEX
  uint32_t maccr = ETH->MACCR;

  maccr &= ~ETH_MACCR_FES;
  maccr |= ETH_MACCR_DM;
#else
#error "invalid STM32_MAC_PHY_LINK_TYPE"
#endif

  /* Changes the mode in the MAC.*/
  ETH->MACCR = maccr;

  /* Returns the link status.*/
  return macp->link_up = true;
}

/**
 * @brief   Writes to a transmit descriptor's stream.
 *
 * @param[in] tdp       pointer to a @p MACTransmitDescriptor structure
 * @param[in] buf       pointer to the buffer containing the data to be
 *                      written
 * @param[in] size      number of bytes to be written
 * @return              The number of bytes written into the descriptor's
 *                      stream, this value can be less than the amount
 *                      specified in the parameter @p size if the maximum
 *                      frame size is reached.
 *
 * @notapi
 */
size_t mac_lld_write_transmit_descriptor(MACTransmitDescriptor *tdp,
                                         uint8_t *buf,
                                         size_t size) {

  osalDbgAssert(!(tdp->physdesc->tdes3 & STM32_TDES3_OWN),
              "attempt to write descriptor already owned by DMA");

  if (size > tdp->size - tdp->offset)
    size = tdp->size - tdp->offset;

  if (size > 0) {
    memcpy((uint8_t *)(tdp->physdesc->tdes0) + tdp->offset, buf, size);
    tdp->offset += size;
  }
  return size;
}

/**
 * @brief   Reads from a receive descriptor's stream.
 *
 * @param[in] rdp       pointer to a @p MACReceiveDescriptor structure
 * @param[in] buf       pointer to the buffer that will receive the read data
 * @param[in] size      number of bytes to be read
 * @return              The number of bytes read from the descriptor's
 *                      stream, this value can be less than the amount
 *                      specified in the parameter @p size if there are
 *                      no more bytes to read.
 *
 * @notapi
 */
size_t mac_lld_read_receive_descriptor(MACReceiveDescriptor *rdp,
                                       uint8_t *buf,
                                       size_t size) {

  osalDbgAssert(!(rdp->physdesc->rdes3 & STM32_RDES3_OWN),
              "attempt to read descriptor already owned by DMA");

  if (size > rdp->size - rdp->offset)
    size = rdp->size - rdp->offset;

  if (size > 0) {
    memcpy(buf, (uint8_t *)(rdp->physdesc->rdes0) + rdp->offset, size);
    rdp->offset += size;
  }
  return size;
}

#endif /* HAL_USE_MAC */

/** @} */
