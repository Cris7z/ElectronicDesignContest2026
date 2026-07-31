#include "sys.h"

void WFI_SET(void)
{
    __ASM volatile("wfi");
}

void INTX_DISABLE(void)
{
    __ASM volatile("cpsid i");
}

void INTX_ENABLE(void)
{
    __ASM volatile("cpsie i");
}

void MSR_MSP(u32 addr) __attribute__((naked));

void MSR_MSP(u32 addr)
{
    (void)addr;
    __ASM volatile("msr msp, r0\n"
                   "bx lr\n");
}
