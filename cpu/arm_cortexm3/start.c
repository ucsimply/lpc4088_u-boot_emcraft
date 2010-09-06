#include <config.h>
#include "my_uart.h"
#include "CMSIS/a2fxxxm3.h"

extern char _data_lma_start;
extern char _data_start;
extern char _data_end;
extern char _bss_start;
extern char _bss_end;

extern char armboot_start;
unsigned int _armboot_start;

void _start(void);
void default_isr(void);

extern void start_armboot(void);

/*
 * Vectors:
 */ 

unsigned int vectors[] __attribute__((section(".vectors"))) = {

    /*
     * The first word is the stack base address (stack grows downwards)
     * Stack size is defined by reserving an area the top of RAM in u-boot.lds.
     * Supposedly, all A2F variants have at least 64K of internal Flash.
     * TO-DO: obviously, this is A2F-speficic. Should I do anything about this?
     */
    [0]		= 0x20010000,

    /* 
     * Reset entry point
     */ 
    [1]		= (unsigned int)&_start,

    /*
     * Other exceptions
     */   
    [2 ... 165]	= (unsigned int)&default_isr
};

 /* 
  * Reset entry point
  */ 

void _start(void)
{
#if !defined(CONFIG_HW_WATCHDOG)
    /* 
     * Disable WDT - unless this is done, we would have to strobe
     * the WDT every once in a while to avoid a reset.
     * Note that after the WDT is disabled, it can't be enabled
     * anymore, until a next re-boot.
     * TO-DO: this is an A2F-specific action - move it somewhere
     */
    WATCHDOG->WDOGENABLE = 0x4C6E55FA;
#else
    /*
     * Enable WDT -> period is about 40 seconds.
     */
    WATCHDOG->WDOGMVRP = 0xFFFFFFFF;
    WATCHDOG->WDOGLOAD = 0xFFFFFF00;
    WATCHDOG->WDOGCONTROL = 0;
    hw_watchdog_reset();
#endif

    /*
     * Make sure interrupts are disabled.
     * TO-DO: figure out the interrupt-handling policy in U-boot
     * TO-DO: is it affected by CONFIG_USE_IRQ?
     */
    __disable_irq();

    /*
     * Copy data and initialize BSS
     * This is in lieu of the U-boot "conventional" relocation
     * of code & data from Flash to RAM. 
     * With the A2F, we execute from NVRAM (internal Flash),
     * having relocated data to internal RAM (and having cleared the BSS
     * area in internal RAM as well)
     * TO-DO: is it A2F-specific (the fact, that we execute from NVRAM)?
     * TO-DO: if so, do I need to move this to somewhere else? 
     * Stack grows downwards; the stack base is set-up by the first
     * value in the first word in the vectors.
     * TO-DO: is the stack handling Cortex-M3 or A2F-specific?
     */
    memcpy(&_data_start, &_data_lma_start, &_data_end - &_data_start);
    memset(&_bss_start, 0, &_bss_end - &_bss_start);

    /*
     * In U-boot (armboot) lingvo, "go to the C code" - 
     * in fact, with M3, we are at the C code from the very beginning.
     * In actuality, this is the jump to the ARM generic start code.
     * ...
     * Note initialization of _armboot_start below. The ARM generic
     * code expects that this variable is set to the upper boundary of
     * the malloc pool area. 
     * For A2F, where we do not relocate the code to RAM, I set
     * the malloc pool right behind the stack. See how armboot_start
     * is defined in the CPU specific .lds file.
     * TO-DO: There is yet another complication here. The ARM generic code
     * makes the assumption that the malloc pool resides right below
     * the U-boot code, as relocated to RAM and hence _armoot_start
     * is both the base of the U-boot code and the upper boundary
     * for the malloc pool ... This is not the case for A2F, hence
     * we will have to re-set _armoot_start to the U-boot code base
     * in the CPU specific initialization code. 
     * There is the same issue for another global: monitor_flash_len
     */  
    _armboot_start = &armboot_start;
    start_armboot();
}


/*
 * Default exception handler
 */

void __attribute__((naked, noreturn)) default_isr(void);
void default_isr(void)
{
    asm("mov r0, sp; bl dump_ctx");
    for (;;) ;
}

/*
 * TO-DO: do we need this?
 */ 
static void __attribute__((used)) dump_ctx(unsigned int *ctx)
{
    static char *regs[] = {
	"R0", "R1", "R2", "R3", "R12", "LR", "PC", "PSR"
    };
    static char *exc[] = {
	0,
	0,
	"NMI",
	"HARD FAULT",
	"MEMORY MANAGEMENT",
	"BUS FAULT",
	"USAGE FAULT",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"SVCALL",
	"DEBUG MONITOR",
	"RESERVED",
	"PENDSV",
	"SYSTICK",
    };
    unsigned char vec = SCB->ICSR & 0xFF;
    int i;

    my_printf("==================================\n");
    my_printf("UNHANDLED EXCEPTION: ");
    if (vec < 16) {
	my_printf("%s\n", exc[vec]);
    } else {
	my_printf("INTISR[%d]\n", vec - 16);
    }
    for (i = 0; i < 8; i++) {
	my_printf("  %s\t= %08x", regs[i], ctx[i]);
	if (((i + 1) % 2) == 0) {
	    my_printf("\n");
	}
    }
    my_printf("==================================\n");
}

void hw_watchdog_reset(void)
{
    /* Refresh watchdog */
    WATCHDOG->WDOGREFRESH = 0xAC15DE42;
}
