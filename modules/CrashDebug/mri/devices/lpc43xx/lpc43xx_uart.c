/* Copyright 2016 Adam Green (http://mbed.org/users/AdamGreen/)

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
/* Routines used to provide LPC43xx UART functionality to the mri debugger. */
#include <string.h>
#include <stdlib.h>
#include "platforms.h"
#include "../../architectures/armv7-m/debug_cm3.h"
#include "lpc43xx_init.h"


static const UartConfiguration g_uartConfigurations[] =
{
    {
        LPC_USART0,
        CLK_BASE_UART0,
        CLK_MX_UART0,
        CLK_APB0_UART0,
        SCU_PIN(6, 4),
        SCU_MODE_FUNC2,
        SCU_PIN(6, 5),
        SCU_MODE_FUNC2
    },
    {
        LPC_UART1,
        CLK_BASE_UART1,
        CLK_MX_UART1,
        CLK_APB0_UART1,
        SCU_PIN(5, 6),
        SCU_MODE_FUNC4,
        SCU_PIN(1, 14),
        SCU_MODE_FUNC1
    },
    {
        LPC_USART2,
        CLK_BASE_UART2,
        CLK_MX_UART2,
        CLK_APB2_UART2,
        SCU_PIN(2, 10),
        SCU_MODE_FUNC2,
        SCU_PIN(2, 11),
        SCU_MODE_FUNC2
    },
    {
        LPC_USART3,
        CLK_BASE_UART3,
        CLK_MX_UART3,
        CLK_APB2_UART3,
        SCU_PIN(2, 3),
        SCU_MODE_FUNC2,
        SCU_PIN(2, 4),
        SCU_MODE_FUNC2
    }
};

static UartConfiguration g_customUart;

typedef struct
{
    const UartConfiguration* pUart;
    int                      share;
    uint32_t                 baudRate;
} UartParameters;

typedef struct
{
    uint32_t    integerBaudRateDivisor;
    uint32_t    fractionalBaudRateDivisor;
} BaudRateDivisors;

typedef struct
{
    uint32_t    desiredRatio;
    uint32_t    mul;
    uint32_t    divAdd;
    uint32_t    closestDivisor;
    uint32_t    closestMul;
    uint32_t    closestDivAdd;
    uint32_t    closestDelta;
} CalculateDivisors;



static void     parseUartParameters(Token* pParameterTokens, UartConfiguration* pUart, UartParameters* pParameters);
static void     saveUartToBeUsedByDebugger(const UartConfiguration* pUart);
static void     setUartSharedFlag(void);
static uint32_t uint32FromString(const char* pString);
static uint32_t getDecimalDigit(char currChar);
static void     configureUartForExclusiveUseOfDebugger(UartParameters* pParameters);
static void     setUartPeripheralClockToPLL1(void);
static void     enableUartClocks(void);
static void     enableCCUClock(CCU_CLK_T clockToEnable);
static void     clearUartFractionalBaudDivisor(void);
static void     enableUartFifoAndDisableDma(void);
static void     setUartTo8N1(void);
static void     setUartBaudRate(UartParameters* pParameters);
static void     setDivisors(BaudRateDivisors* pDivisors);
static void     setDivisorLatchBit(void);
static void     clearDivisorLatchBit(void);
static void     setManualBaudFlag(void);
static BaudRateDivisors calculateBaudRateDivisors(uint32_t baudRate, uint32_t peripheralRate);
static void     initCalculateDivisorsStruct(CalculateDivisors* pThis, uint32_t baudRate, uint32_t peripheralRate);
static uint32_t fixupPeripheralRateFor16XOversampling(uint32_t actualPeripheralRate);
static int      isNoFractionalDivisorRequired(CalculateDivisors* pThis);
static BaudRateDivisors closestDivisors(CalculateDivisors* pThis);
static BaudRateDivisors calculateFractionalBaudRateDivisors(CalculateDivisors* pThis);
static void     checkTheseFractionalDivisors(CalculateDivisors* pThis);
static void     selectUartPins(void);
static void     enableUartToInterruptOnReceivedChar(void);
static void     configureNVICForUartInterrupt(void);
void __mriLpc43xxUart_Init(Token* pParameterTokens)
{
    UartParameters    parameters = {NULL, 0, 0};

    parseUartParameters(pParameterTokens, &g_customUart, &parameters);
    saveUartToBeUsedByDebugger(parameters.pUart);
    if (parameters.share)
        setUartSharedFlag();
    else
        configureUartForExclusiveUseOfDebugger(&parameters);
}

static void parseUartParameters(Token* pParameterTokens, UartConfiguration* pUart, UartParameters* pParameters)
{
    static const char baudRatePrefix[] = "MRI_UART_BAUD=";
    const char*       pMatchingPrefix = NULL;
    LPC_USART_T*      pRxUartRegisters = NULL;

    /* Parse TX pins */
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P1_13"))
    {
        /* {P1_13, UART_1, (SCU_PINIO_UART_TX | 1)}, */
        pUart->pUartRegisters = LPC_UART1;
        pUart->txPin = SCU_PIN(1, 13);
        pUart->txFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P1_15"))
    {
        /* {P1_15, UART_2, (SCU_PINIO_UART_TX | 1)}, */
        pUart->pUartRegisters = LPC_USART2;
        pUart->txPin = SCU_PIN(1, 15);
        pUart->txFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P2_0"))
    {
        /* {P2_0,  UART_0, (SCU_PINIO_UART_TX | 1)}, */
        pUart->pUartRegisters = LPC_USART0;
        pUart->txPin = SCU_PIN(2, 0);
        pUart->txFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P2_3"))
    {
        /* {P2_3,  UART_3, (SCU_PINIO_UART_TX | 2)}, */
        pUart->pUartRegisters = LPC_USART3;
        pUart->txPin = SCU_PIN(2, 3);
        pUart->txFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P2_10"))
    {
        /* {P2_10, UART_2, (SCU_PINIO_UART_TX | 2)}, */
        pUart->pUartRegisters = LPC_USART2;
        pUart->txPin = SCU_PIN(2, 10);
        pUart->txFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P3_4"))
    {
        /* {P3_4,  UART_1, (SCU_PINIO_UART_TX | 4)}, */
        pUart->pUartRegisters = LPC_UART1;
        pUart->txPin = SCU_PIN(3, 4);
        pUart->txFunction = SCU_MODE_FUNC4;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P4_1"))
    {
        /* {P4_1,  UART_3, (SCU_PINIO_UART_TX | 6)}, */
        pUart->pUartRegisters = LPC_USART3;
        pUart->txPin = SCU_PIN(4, 1);
        pUart->txFunction = SCU_MODE_FUNC6;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P5_6"))
    {
        /* {P5_6,  UART_1, (SCU_PINIO_UART_TX | 4)}, */
        pUart->pUartRegisters = LPC_UART1;
        pUart->txPin = SCU_PIN(5, 6);
        pUart->txFunction = SCU_MODE_FUNC4;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P6_4"))
    {
        /* {P6_4,  UART_0, (SCU_PINIO_UART_TX | 2)}, */
        pUart->pUartRegisters = LPC_USART0;
        pUart->txPin = SCU_PIN(6, 4);
        pUart->txFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P7_1"))
    {
        /* {P7_1,  UART_2, (SCU_PINIO_UART_TX | 6)}, */
        pUart->pUartRegisters = LPC_USART2;
        pUart->txPin = SCU_PIN(7, 1);
        pUart->txFunction = SCU_MODE_FUNC6;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P9_3"))
    {
        /* {P9_3,  UART_3, (SCU_PINIO_UART_TX | 7)}, */
        pUart->pUartRegisters = LPC_USART3;
        pUart->txPin = SCU_PIN(9, 3);
        pUart->txFunction = SCU_MODE_FUNC7;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_P9_5"))
    {
        /* {P9_5,  UART_0, (SCU_PINIO_UART_TX | 7)}, */
        pUart->pUartRegisters = LPC_USART0;
        pUart->txPin = SCU_PIN(9, 5);
        pUart->txFunction = SCU_MODE_FUNC7;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_PA_1"))
    {
        /* {PA_1,  UART_2, (SCU_PINIO_UART_TX | 3)}, */
        pUart->pUartRegisters = LPC_USART2;
        pUart->txPin = SCU_PIN(0xA, 1);
        pUart->txFunction = SCU_MODE_FUNC3;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_PC_13"))
    {
        /* {PC_13, UART_1, (SCU_PINIO_UART_TX | 2)}, */
        pUart->pUartRegisters = LPC_UART1;
        pUart->txPin = SCU_PIN(0xC, 13);
        pUart->txFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_PE_11"))
    {
        /* {PE_11, UART_1, (SCU_PINIO_UART_TX | 2)}, */
        pUart->pUartRegisters = LPC_UART1;
        pUart->txPin = SCU_PIN(0xE, 11);
        pUart->txFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_PF_2"))
    {
        /* {PF_2,  UART_3, (SCU_PINIO_UART_TX | 1)}, */
        pUart->pUartRegisters = LPC_USART3;
        pUart->txPin = SCU_PIN(0xF, 2);
        pUart->txFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_TX_PF_10"))
    {
        /* {PF_10, UART_0, (SCU_PINIO_UART_TX | 1)}, */
        pUart->pUartRegisters = LPC_USART0;
        pUart->txPin = SCU_PIN(0xF, 10);
        pUart->txFunction = SCU_MODE_FUNC1;
    }


    /* Parse RX pins */
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P1_14"))
    {
        /* {P1_14, UART_1, (SCU_PINIO_UART_RX | 1)}, */
        pRxUartRegisters = LPC_UART1;
        pUart->rxPin = SCU_PIN(1, 14);
        pUart->rxFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P1_16"))
    {
        /* {P1_16, UART_2, (SCU_PINIO_UART_RX | 1)}, */
        pRxUartRegisters = LPC_USART2;
        pUart->rxPin = SCU_PIN(1, 16);
        pUart->rxFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P2_1"))
    {
        /* {P2_1,  UART_0, (SCU_PINIO_UART_RX | 1)}, */
        pRxUartRegisters = LPC_USART0;
        pUart->rxPin = SCU_PIN(2, 1);
        pUart->rxFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P2_4"))
    {
        /* {P2_4,  UART_3, (SCU_PINIO_UART_RX | 2)}, */
        pRxUartRegisters = LPC_USART3;
        pUart->rxPin = SCU_PIN(2, 4);
        pUart->rxFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P2_11"))
    {
        /* {P2_11, UART_2, (SCU_PINIO_UART_RX | 2)}, */
        pRxUartRegisters = LPC_USART2;
        pUart->rxPin = SCU_PIN(2, 11);
        pUart->rxFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P3_5"))
    {
        /* {P3_5,  UART_1, (SCU_PINIO_UART_RX | 4)}, */
        pRxUartRegisters = LPC_UART1;
        pUart->rxPin = SCU_PIN(3, 5);
        pUart->rxFunction = SCU_MODE_FUNC4;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P4_2"))
    {
        /* {P4_2,  UART_3, (SCU_PINIO_UART_RX | 6)}, */
        pRxUartRegisters = LPC_USART3;
        pUart->rxPin = SCU_PIN(4, 2);
        pUart->rxFunction = SCU_MODE_FUNC6;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P5_7"))
    {
        /* {P5_7,  UART_1, (SCU_PINIO_UART_RX | 4)}, */
        pRxUartRegisters = LPC_UART1;
        pUart->rxPin = SCU_PIN(5, 7);
        pUart->rxFunction = SCU_MODE_FUNC4;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P6_5"))
    {
        /* {P6_5,  UART_0, (SCU_PINIO_UART_RX | 2)}, */
        pRxUartRegisters = LPC_USART0;
        pUart->rxPin = SCU_PIN(6, 5);
        pUart->rxFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P7_2"))
    {
        /* {P7_2,  UART_2, (SCU_PINIO_UART_RX | 6)}, */
        pRxUartRegisters = LPC_USART2;
        pUart->rxPin = SCU_PIN(7, 2);
        pUart->rxFunction = SCU_MODE_FUNC6;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P9_4"))
    {
        /* {P9_4,  UART_3, (SCU_PINIO_UART_RX | 7)}, */
        pRxUartRegisters = LPC_USART3;
        pUart->rxPin = SCU_PIN(9, 4);
        pUart->rxFunction = SCU_MODE_FUNC7;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_P9_6"))
    {
        /* {P9_6,  UART_0, (SCU_PINIO_UART_RX | 7)}, */
        pRxUartRegisters = LPC_USART0;
        pUart->rxPin = SCU_PIN(9, 6);
        pUart->rxFunction = SCU_MODE_FUNC7;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_PA_2"))
    {
        /* {PA_2,  UART_2, (SCU_PINIO_UART_RX | 3)}, */
        pRxUartRegisters = LPC_USART2;
        pUart->rxPin = SCU_PIN(0xA, 2);
        pUart->rxFunction = SCU_MODE_FUNC3;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_PC_14"))
    {
        /* {PC_14, UART_1, (SCU_PINIO_UART_RX | 2)}, */
        pRxUartRegisters = LPC_UART1;
        pUart->rxPin = SCU_PIN(0xC, 14);
        pUart->rxFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_PE_12"))
    {
        /* {PE_12, UART_1, (SCU_PINIO_UART_RX | 2)}, */
        pRxUartRegisters = LPC_UART1;
        pUart->rxPin = SCU_PIN(0xE, 12);
        pUart->rxFunction = SCU_MODE_FUNC2;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_PF_3"))
    {
        /* {PF_3,  UART_3, (SCU_PINIO_UART_RX | 1)}, */
        pRxUartRegisters = LPC_USART3;
        pUart->rxPin = SCU_PIN(0xF, 3);
        pUart->rxFunction = SCU_MODE_FUNC1;
    }
    if (Token_MatchingString(pParameterTokens, "MRI_UART_RX_PF_11"))
    {
        /* {PF_11, UART_0, (SCU_PINIO_UART_RX | 1)}, */
        pRxUartRegisters = LPC_USART0;
        pUart->rxPin = SCU_PIN(0xF, 11);
        pUart->rxFunction = SCU_MODE_FUNC1;
    }

    /* Remember this UART if Tx and Rx pin matched to the same UART peripherals. */
    if (pUart->pUartRegisters == pRxUartRegisters)
        pParameters->pUart = pUart;

    /* The Bambino210E only exposes a single set of pins per UART so these are specific to that board. */
    if (Token_MatchingString(pParameterTokens, "MRI_UART_MBED_USB"))
        pParameters->pUart = &g_uartConfigurations[2];
    if (Token_MatchingString(pParameterTokens, "MRI_UART_0"))
        pParameters->pUart = &g_uartConfigurations[0];
    if (Token_MatchingString(pParameterTokens, "MRI_UART_1"))
        pParameters->pUart = &g_uartConfigurations[1];
    if (Token_MatchingString(pParameterTokens, "MRI_UART_2"))
        pParameters->pUart = &g_uartConfigurations[2];
    if (Token_MatchingString(pParameterTokens, "MRI_UART_3"))
        pParameters->pUart = &g_uartConfigurations[3];

    /* Default to MRI_UART_MBED_USB if nothing else was specified. */
    if (!pParameters->pUart)
        pParameters->pUart = &g_uartConfigurations[2];

    if ((pMatchingPrefix = Token_MatchingStringPrefix(pParameterTokens, baudRatePrefix)) != NULL)
        pParameters->baudRate = uint32FromString(pMatchingPrefix + sizeof(baudRatePrefix)-1);

    if (Token_MatchingString(pParameterTokens, "MRI_UART_SHARE"))
        pParameters->share = 1;
}

static void saveUartToBeUsedByDebugger(const UartConfiguration* pUart)
{
    __mriLpc43xxState.pCurrentUart = pUart;
}

static void setUartSharedFlag(void)
{
    __mriLpc43xxState.flags |= LPC43XX_UART_FLAGS_SHARE;
}

static uint32_t uint32FromString(const char* pString)
{
    uint32_t value = 0;

    while (*pString)
    {
        uint32_t digit;

        __try
        {
            digit = getDecimalDigit(*pString++);
        }
        __catch
        {
            clearExceptionCode();
            break;
        }

        value = value * 10 + digit;
    }

    return value;
}

static uint32_t getDecimalDigit(char currChar)
{
    if (currChar >= '0' && currChar <= '9')
        return currChar - '0';
    else
        __throw_and_return(invalidDecDigitException, 0);
}

static void configureUartForExclusiveUseOfDebugger(UartParameters* pParameters)
{
    setUartPeripheralClockToPLL1();
    enableUartClocks();
    clearUartFractionalBaudDivisor();
    enableUartFifoAndDisableDma();
    setUartTo8N1();
    setUartBaudRate(pParameters);
    selectUartPins();
    enableUartToInterruptOnReceivedChar();
    Platform_CommPrepareToWaitForGdbConnection();
    configureNVICForUartInterrupt();
}

static void setUartPeripheralClockToPLL1(void)
{
    static const uint32_t autoBlockBit = 1 << 11;
    static const uint32_t pll1Bit = 0x09 << 24;

    LPC_CGU->BASE_CLK[__mriLpc43xxState.pCurrentUart->baseClock] = autoBlockBit | pll1Bit;
}

static void enableUartClocks(void)
{
    enableCCUClock(__mriLpc43xxState.pCurrentUart->registerClock);
    enableCCUClock(__mriLpc43xxState.pCurrentUart->peripheralClock);
}

static void enableCCUClock(CCU_CLK_T clockToEnable)
{
    static const uint32_t runBit = 1 << 0;

    if (clockToEnable < CLK_CCU2_START)
        LPC_CCU1->CLKCCU[clockToEnable].CFG = runBit;
    else
        LPC_CCU2->CLKCCU[clockToEnable - CLK_CCU2_START].CFG = runBit;
}

static void clearUartFractionalBaudDivisor(void)
{
    __mriLpc43xxState.pCurrentUart->pUartRegisters->FDR = 0x10;
}

static void enableUartFifoAndDisableDma(void)
{
    static const uint32_t enableFifoDisableDmaSetReceiveInterruptThresholdTo0 = 0x01;

    __mriLpc43xxState.pCurrentUart->pUartRegisters->FCR = enableFifoDisableDmaSetReceiveInterruptThresholdTo0;
}

static void setUartTo8N1(void)
{
    static const uint8_t wordLength8Bit = 0x3;
    static const uint8_t stopBit1 = 0 << 2;
    static const uint8_t disableParity = 0 << 3;
    static const uint8_t lineControlValueFor8N1 = wordLength8Bit | disableParity | stopBit1;

    __mriLpc43xxState.pCurrentUart->pUartRegisters->LCR = lineControlValueFor8N1;
}

static void setUartBaudRate(UartParameters* pParameters)
{
    BaudRateDivisors  divisors;

    if (pParameters->baudRate == 0)
        return;

    divisors = calculateBaudRateDivisors(pParameters->baudRate, SystemCoreClock);
    setDivisors(&divisors);
    setManualBaudFlag();
}

static BaudRateDivisors calculateBaudRateDivisors(uint32_t baudRate, uint32_t peripheralRate)
{
    CalculateDivisors   calcDivisors;

    initCalculateDivisorsStruct(&calcDivisors, baudRate, peripheralRate);
    if (isNoFractionalDivisorRequired(&calcDivisors))
        return closestDivisors(&calcDivisors);
    return calculateFractionalBaudRateDivisors(&calcDivisors);
}

static void initCalculateDivisorsStruct(CalculateDivisors* pThis, uint32_t baudRate, uint32_t peripheralRate)
{
    /* Calculate desired clock to baud ratio in 17.15 fixed format.
       This code can handle peripheralRates which are less <512MHz since
         2^29 / 16 = 2^29 / 2^4 = 2^25 and (2^25 << 7) fits in 32-bit value.
       If you divide by a low baud rate like 300 (>2^8) then mantissa only needs 25 - 8 = 17 bits. */
    pThis->desiredRatio = ((fixupPeripheralRateFor16XOversampling(peripheralRate) << 7) / baudRate) << 8;
    pThis->mul = 1;
    pThis->divAdd = 0;
    pThis->closestDelta = ~0U;
    pThis->closestDivisor = pThis->desiredRatio >> 15;
    pThis->closestMul = 1;
    pThis->closestDivAdd = 0;
}

static uint32_t fixupPeripheralRateFor16XOversampling(uint32_t actualPeripheralRate)
{
    return actualPeripheralRate / 16;
}

static int isNoFractionalDivisorRequired(CalculateDivisors* pThis)
{
    /* Check for no fractional bits in the 17.15 fixed format. */
    return (pThis->desiredRatio & 0x7FFF) == 0;
}

static BaudRateDivisors closestDivisors(CalculateDivisors* pThis)
{
    BaudRateDivisors divisors;

    divisors.integerBaudRateDivisor = pThis->closestDivisor;
    divisors.fractionalBaudRateDivisor = (pThis->closestMul << 4) | pThis->closestDivAdd;

    return divisors;
}

static BaudRateDivisors calculateFractionalBaudRateDivisors(CalculateDivisors* pThis)
{
    for (pThis->mul = 1 ; pThis->mul <= 15 ; pThis->mul++)
    {
        for (pThis->divAdd = 1 ; pThis->divAdd < pThis->mul ; pThis->divAdd++)
            checkTheseFractionalDivisors(pThis);
    }
    return closestDivisors(pThis);
}

static void checkTheseFractionalDivisors(CalculateDivisors* pThis)
{
    static const uint32_t fixedOne = (1 << 15);
    uint32_t              fixedScale;
    uint32_t              testDivisor;
    uint32_t              fixedRatio;
    uint32_t              fixedDelta;

    fixedScale = fixedOne + ((pThis->divAdd << 15) / pThis->mul);
    testDivisor = pThis->desiredRatio / fixedScale;
    fixedRatio = testDivisor * fixedScale;
    fixedDelta = abs(fixedRatio - pThis->desiredRatio);

    if (fixedDelta < pThis->closestDelta)
    {
        pThis->closestDelta = fixedDelta;
        pThis->closestDivisor = testDivisor;
        pThis->closestMul = pThis->mul;
        pThis->closestDivAdd = pThis->divAdd;
    }
}

static void setDivisors(BaudRateDivisors* pDivisors)
{
    LPC_USART_T* pUartRegisters = __mriLpc43xxState.pCurrentUart->pUartRegisters;

    setDivisorLatchBit();

    pUartRegisters->DLL = pDivisors->integerBaudRateDivisor & 0xFF;
    pUartRegisters->DLM = pDivisors->integerBaudRateDivisor >> 8;
    pUartRegisters->FDR = pDivisors->fractionalBaudRateDivisor;

    clearDivisorLatchBit();
}

#define LPC176x_UART_LCR_DLAB (1 << 7)

static void setDivisorLatchBit(void)
{
    __mriLpc43xxState.pCurrentUart->pUartRegisters->LCR |= LPC176x_UART_LCR_DLAB;
}

static void clearDivisorLatchBit(void)
{
    __mriLpc43xxState.pCurrentUart->pUartRegisters->LCR &= ~LPC176x_UART_LCR_DLAB;
}

static void setManualBaudFlag(void)
{
    __mriLpc43xxState.flags |= LPC43XX_UART_FLAGS_MANUAL_BAUD;

}

static void selectUartPins(void)
{
    const UartConfiguration*  pUart = __mriLpc43xxState.pCurrentUart;
    uint32_t                  txPin = pUart->txPin;
    uint32_t                  rxPin = pUart->rxPin;

    LPC_SCU->SFSP[txPin >> 16][txPin & 0xFFFF] = pUart->txFunction;
    LPC_SCU->SFSP[rxPin >> 16][rxPin & 0xFFFF] = pUart->rxFunction | SCU_PINIO_PULLNONE;
}

static void enableUartToInterruptOnReceivedChar(void)
{
    static const uint32_t baudDivisorLatchBit = (1 << 7);
    static const uint32_t enableReceiveDataInterrupt = (1 << 0);
    uint32_t              originalLCR;

    originalLCR = __mriLpc43xxState.pCurrentUart->pUartRegisters->LCR;
    __mriLpc43xxState.pCurrentUart->pUartRegisters->LCR &= ~baudDivisorLatchBit;
    __mriLpc43xxState.pCurrentUart->pUartRegisters->IER = enableReceiveDataInterrupt;
    __mriLpc43xxState.pCurrentUart->pUartRegisters->LCR = originalLCR;
}

static void configureNVICForUartInterrupt(void)
{
    IRQn_Type uart0BaseIRQ = USART0_IRQn;
    IRQn_Type currentUartIRQ;

    currentUartIRQ = (IRQn_Type)((int)uart0BaseIRQ + Platform_CommUartIndex());
    NVIC_SetPriority(currentUartIRQ, 0);
    NVIC_EnableIRQ(currentUartIRQ);
}


int Platform_CommUartIndex(void)
{
    if (__mriLpc43xxState.pCurrentUart->pUartRegisters == LPC_USART0)
        return 0;
    else if (__mriLpc43xxState.pCurrentUart->pUartRegisters == LPC_UART1)
        return 1;
    else if (__mriLpc43xxState.pCurrentUart->pUartRegisters == LPC_USART2)
        return 2;
    else if (__mriLpc43xxState.pCurrentUart->pUartRegisters == LPC_USART3)
        return 3;
    else
        return -1;
}


uint32_t Platform_CommHasReceiveData(void)
{
    static const uint8_t receiverDataReadyBit = 1 << 0;

    return __mriLpc43xxState.pCurrentUart->pUartRegisters->LSR & receiverDataReadyBit;
}


static void     waitForUartToReceiveData(void);
int Platform_CommReceiveChar(void)
{
    waitForUartToReceiveData();

    return (int)__mriLpc43xxState.pCurrentUart->pUartRegisters->RBR;
}

static void waitForUartToReceiveData(void)
{
    while (!Platform_CommHasReceiveData())
    {
    }
}

static void     waitForUartToAllowTransmit(void);
static uint32_t targetUartCanTransmit(void);
void Platform_CommSendChar(int Character)
{
    waitForUartToAllowTransmit();

    __mriLpc43xxState.pCurrentUart->pUartRegisters->THR = (uint8_t)Character;
}

static void waitForUartToAllowTransmit(void)
{
    while (!targetUartCanTransmit())
    {
    }
}

static uint32_t targetUartCanTransmit(void)
{
    static const uint8_t transmitterHoldRegisterEmptyBit = 1 << 5;

    return __mriLpc43xxState.pCurrentUart->pUartRegisters->LSR & transmitterHoldRegisterEmptyBit;
}


int Platform_CommCausedInterrupt(void)
{
    const uint32_t uart0BaseExceptionId = USART0_IRQn + 16;
    uint32_t       interruptSource = getCurrentlyExecutingExceptionNumber();
    uint32_t       currentUartExceptionId;

    currentUartExceptionId = uart0BaseExceptionId + Platform_CommUartIndex();
    return interruptSource == currentUartExceptionId;
}


void Platform_CommClearInterrupt(void)
{
    uint32_t interruptId;

    interruptId = __mriLpc43xxState.pCurrentUart->pUartRegisters->IIR;
    (void)interruptId;
}


int Platform_CommSharingWithApplication(void)
{
    return __mriLpc43xxState.flags & LPC43XX_UART_FLAGS_SHARE;
}

static int isManualBaudRate(void);
int Platform_CommShouldWaitForGdbConnect(void)
{
    return !isManualBaudRate() && !Platform_CommSharingWithApplication();
}

static int isManualBaudRate(void)
{
    return (int)(__mriLpc43xxState.flags & LPC43XX_UART_FLAGS_MANUAL_BAUD);
}


int Platform_CommIsWaitingForGdbToConnect(void)
{
    static const uint32_t autoBaudStarting = 1;

    if (!Platform_CommShouldWaitForGdbConnect())
        return 0;

    return (int)(__mriLpc43xxState.pCurrentUart->pUartRegisters->ACR & autoBaudStarting);
}


void Platform_CommPrepareToWaitForGdbConnection(void)
{
    static const uint32_t   autoBaudStart = 1;
    static const uint32_t   autoBaudModeForStartBitOnly = 1 << 1;
    static const uint32_t   autoBaudAutoRestart = 1 << 2;
    static const uint32_t   autoBaudValue = autoBaudStart | autoBaudModeForStartBitOnly | autoBaudAutoRestart;

    if (!Platform_CommShouldWaitForGdbConnect())
        return;

    __mriLpc43xxState.pCurrentUart->pUartRegisters->ACR = autoBaudValue;
}


static int hasHostSentDataInLessThan10Milliseconds(void);
static int isNoReceivedCharAndNoTimeout(void);
void Platform_CommWaitForReceiveDataToStop(void)
{
    while (hasHostSentDataInLessThan10Milliseconds())
    {
        Platform_CommReceiveChar();
    }
}

static int hasHostSentDataInLessThan10Milliseconds(void)
{
    uint32_t originalSysTickControlValue = getCurrentSysTickControlValue();
    uint32_t originalSysTickReloadValue = getCurrentSysTickReloadValue();
    start10MillisecondSysTick();

    while (isNoReceivedCharAndNoTimeout())
    {
    }

    setSysTickReloadValue(originalSysTickReloadValue);
    setSysTickControlValue(originalSysTickControlValue);

    return Platform_CommHasReceiveData();
}

static int isNoReceivedCharAndNoTimeout(void)
{
    return !Platform_CommHasReceiveData() && !has10MillisecondSysTickExpired();
}
