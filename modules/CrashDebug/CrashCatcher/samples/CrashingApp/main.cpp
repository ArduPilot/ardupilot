/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)

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
#include <mbed.h>
#include <CrashCatcher.h>


Serial g_pc(USBTX, USBRX);


// Assembly language routines defined in tests.S
extern "C" void testMspMultipleOf8(void);
extern "C" void testMspNotMultipleOf8(void);
extern "C" void testPspMultipleOf8(void);
extern "C" void testInitFPURegisters(void);
extern "C" void testBreakpoints(void);

static void enable8ByteStackAlignment();
static void crashWithFPUDisabled();
static void crashWithFPUAutoStackingDisabled();
static void crashWithFPUAutoStackEnabled();
static void crashWithFPULazyAutoStacking();


int main()
{
    char buffer[128];
    int option = -1;

    enable8ByteStackAlignment();

#if defined(TARGET_LPC1768) || defined(TARGET_LPC11U24)
    mbed_interface_disconnect();
#endif

    g_pc.baud(115200);
    while (1)
    {
        printf("\r\n\r\nSelect crash test to run\r\n");
        printf("1) MSP Rounded to multiple of 8 bytes.\r\n");
        printf("2) MSP Not Rounded to multiple of 8 bytes.\r\n");
        printf("3) PSP in use.\r\n");
        printf("4) Precise fault.\r\n");
        printf("5) Imprecise fault.\r\n");
        printf("6) Fault with FPU disabled.\r\n");
        printf("7) Fault with FPU auto-stacking disabled.\r\n");
        printf("8) Fault with FPU auto-stacking enabled.\r\n");
        printf("9) Fault with FPU lazy auto-stacking.\r\n");
        printf("10) Issue two breakpoints and return.\r\n");
        printf("Select option: ");
        fgets(buffer, sizeof(buffer), stdin);
        sscanf(buffer, "%d", &option);

        switch (option)
        {
        case 1:
            testMspMultipleOf8();
            break;
        case 2:
            testMspNotMultipleOf8();
            break;
        case 3:
            testPspMultipleOf8();
            break;
        case 4:
            CRASH_CATCHER_READ_FAULT();
            break;
        case 5:
            CRASH_CATCHER_WRITE_FAULT();
            break;
        case 6:
            crashWithFPUDisabled();
            break;
        case 7:
            crashWithFPUAutoStackingDisabled();
            break;
        case 8:
            crashWithFPUAutoStackEnabled();
            break;
        case 9:
            crashWithFPULazyAutoStacking();
            break;
        case 10:
            testBreakpoints();
            break;
        default:
            continue;
        }
    }
}

static void enable8ByteStackAlignment()
{
    SCB->CCR |= SCB_CCR_STKALIGN_Msk;
}

#if defined(TARGET_M4)

static void disableFPU()
{
    static const uint32_t FPCA = 1 << 2;
    SCB->CPACR &= ~(0xF << 20);
    __set_CONTROL(__get_CONTROL() & ~FPCA);
}

static void enableFPU()
{
    SCB->CPACR |= (0xF << 20);
}

static void crashWithFPUDisabled()
{
    disableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

static const uint32_t ASPEN = 1 << 31;
static const uint32_t LSPEN = 1 << 30;

static void crashWithFPUAutoStackingDisabled()
{
    disableFPU();
        FPU->FPCCR &= ~(ASPEN | LSPEN);
    enableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

static void crashWithFPUAutoStackEnabled()
{
    disableFPU();
        FPU->FPCCR |= ASPEN;
        FPU->FPCCR &= ~LSPEN;
    enableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

static void crashWithFPULazyAutoStacking()
{
    disableFPU();
        FPU->FPCCR |= (ASPEN | LSPEN);
    enableFPU();
    __disable_irq();
    testInitFPURegisters();
    CRASH_CATCHER_READ_FAULT();
}

#else

static void crashWithFPUDisabled()
{
    return;
}

static void crashWithFPUAutoStackingDisabled()
{
    return;
}

static void crashWithFPUAutoStackEnabled()
{
    return;
}

static void crashWithFPULazyAutoStacking()
{
    return;
}


#endif // !defined(CORTEX_M4)


// Let CrashCatcher know what RAM contents should be part of crash dump.
extern "C" const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void)
{
    static const CrashCatcherMemoryRegion regions[] = {
#if defined(TARGET_LPC1768)
                                                        {0x10000000, 0x10008000, CRASH_CATCHER_BYTE},
                                                        {0x2007C000, 0x20084000, CRASH_CATCHER_BYTE},
                                                        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
#elif defined(TARGET_LPC11U24)
                                                        {0x10000000, 0x10002000, CRASH_CATCHER_BYTE},
                                                        {0x20004000, 0x20004800, CRASH_CATCHER_BYTE},
                                                        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
#elif defined(TARGET_K64F)
                                                        {0x1FFF0000, 0x20030000, CRASH_CATCHER_BYTE},
                                                        {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
#else
    #error "Target device isn't supported."
#endif
                                                      };
    return regions;
}
