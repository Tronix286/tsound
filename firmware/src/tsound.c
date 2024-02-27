/*
 * AY-3-8910 / Covox for UKNC
 * (C) Tronix, 2024
 *
 * This project uses ideas and parts of code by:
 * Ian Scott (c) (polpo) PicoGUS: https://github.com/polpo/picogus
 * pico-56 (c) 2023 Troy Schrapel: https://github.com/visrealm/pico-56
 * emu2149 v1.41 (C) 2001-2022 Mitsutaka Okazaki: https://github.com/digital-sound-antiques/emu2149
 *
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include <tusb.h>
#include "pico/multicore.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/vreg.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "mpi_io.pio.h"
#include "ymodem.h"
#include "audio/audio.h"

#define IOW_PIO_SM 0
#define IOR_PIO_SM 1
#define ADDR_MUX_PIO_SM 0   // at pio1
#define RPLY_PIO_SM 3
#define RPLY2_PIO_SM 2

#define AY38910_CLOCK     1789750 //1750000

const uint LED_PIN = PICO_DEFAULT_LED_PIN;

#define abs(x) ({ __typeof__(x) _x = (x); _x >= 0 ? _x : -_x; })

volatile uint32_t addr;
volatile uint8_t ay3_reg,ay3_dat;

int timer_cnt = 0;
int led_flashes = 0;
struct repeating_timer timer;
int cur_psg = 0;

const uint32_t addr_mux_rxempty = 1u << (PIO_FSTAT_RXEMPTY_LSB + ADDR_MUX_PIO_SM);
__force_inline bool addr_mux_has_data() {
    return !(pio1->fstat & addr_mux_rxempty);
}

const uint32_t iow_rxempty = 1u << (PIO_FSTAT_RXEMPTY_LSB + IOW_PIO_SM);
__force_inline bool iow_has_data() {
    return !(pio0->fstat & iow_rxempty);
}

const uint32_t ior_rxempty = 1u << (PIO_FSTAT_RXEMPTY_LSB + IOR_PIO_SM);
__force_inline bool ior_has_data() {
    return !(pio0->fstat & ior_rxempty);
}

bool repeating_timer_callback(struct repeating_timer *t) {
    gpio_put(LED_PIN, gpio_get(LED_PIN)^1);
    timer_cnt++;
    if (timer_cnt > led_flashes)
    {
      timer_cnt = 0;
      cancel_repeating_timer(t);
    }
    return true;
}

void led_lights(int num)
{
  led_flashes = num;
  if (timer.alarm_id == 0)
    add_repeating_timer_ms(100, repeating_timer_callback, NULL, &timer);
}

void measure_freqs(void) {
    uint f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
    uint f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
    uint f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    uint f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
    uint f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    uint f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);
    uint f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);

    printf("pll_sys  = %dkHz\n", f_pll_sys);
    printf("pll_usb  = %dkHz\n", f_pll_usb);
    printf("rosc     = %dkHz\n", f_rosc);
    printf("clk_sys  = %dkHz\n", f_clk_sys);
    printf("clk_peri = %dkHz\n", f_clk_peri);
    printf("clk_usb  = %dkHz\n", f_clk_usb);
    printf("clk_adc  = %dkHz\n", f_clk_adc);
    printf("clk_rtc  = %dkHz\n", f_clk_rtc);

    // Can't measure clk_ref / xosc as it is the ref
}

static const uint32_t IO_WAIT = 0xffffffffu;
static const uint32_t IO_END = 0x0u;

void __time_critical_func(handle_iow)(void) {
    uint32_t data = ~pio_sm_get(pio0, IOW_PIO_SM); //(iow_read & 0xFFFF);
    switch (addr) {
        // hook 177100 - port A i8255 (Covox)
        case 0xFE40:    
            pio_sm_put(pio0, IOW_PIO_SM, IO_END);   // dont RPLY because 8255 has internal RPLY circuit
            audioWriteCovox(data & 0xFF);
            led_lights(1);
            return;
            break;
        // 177360 - AY1
        case 0xFEF0:    
            pio_sm_put(pio0, IOW_PIO_SM, IO_WAIT);
            if (data & 0x10000) // byte cycle - write value to register
                audioWritePsg0(data & 0xFF);
            else                // word cycle - set psg register
                audioWritePsg0Reg(data & 0xFF);
            return;
            break;
        // 177362 - AY2
        case 0xFEF2: 
            pio_sm_put(pio0, IOW_PIO_SM, IO_WAIT);
            if (data & 0x10000) // byte cycle - write value to register
                audioWritePsg1(data & 0xFF);
            else                // word cycle - set psg register
                audioWritePsg1Reg(data & 0xFF);
            return;
            break;
        // 177364 - AY3 (reserved)
        case 0xFEF4: 
            pio_sm_put(pio0, IOW_PIO_SM, IO_WAIT);
            if (data & 0x10000) // byte cycle - write value to register
                ay3_dat = data & 0xFF; 
            else                // word cycle - set psg register
                ay3_reg = data & 0xFF;
            return;
            break;
        // 0177366-0177377 (reserved)
        case 0xFEF6:
        case 0xFEF8:
        case 0xFEFA:
        case 0xFEFC:
        case 0xFEFE:
            pio_sm_put(pio0, IOW_PIO_SM, IO_WAIT);
            return;
            break;
        default:
            pio_sm_put(pio0, IOW_PIO_SM, IO_END);    
    }
}

void __time_critical_func(handle_ior)(void) {
    uint16_t temp_addr = pio_sm_get(pio0, IOR_PIO_SM);
    switch (addr) {
        case 0xFEF0: 
            pio_sm_put(pio0, IOR_PIO_SM, IO_WAIT);
            pio_sm_put(pio0, IOR_PIO_SM, ~audioReadPsg0());
            return;
            break;
        case 0xFEF2: 
            pio_sm_put(pio0, IOR_PIO_SM, IO_WAIT);
            pio_sm_put(pio0, IOR_PIO_SM, ~audioReadPsg1());
            return;
            break;
        case 0xFEF4: 
            pio_sm_put(pio0, IOR_PIO_SM, IO_WAIT);
            pio_sm_put(pio0, IOR_PIO_SM, ~ay3_dat);
            return;
            break;
        case 0xFEF6:
        case 0xFEF8:
        case 0xFEFA:
        case 0xFEFC:
        case 0xFEFE:
            pio_sm_put(pio0, IOR_PIO_SM, IO_WAIT);
            pio_sm_put(pio0, IOR_PIO_SM, ~0x0000);
            return;
            break;
        default:
            pio_sm_put(pio0, IOR_PIO_SM, IO_END);    
    }
}

void __time_critical_func(addrmux_handler)()
{
        while (addr_mux_has_data()) {
            addr = pio_sm_get_blocking(pio1, ADDR_MUX_PIO_SM); 
        }
}

void play_ay()
{
    audioInit(AY38910_CLOCK, 44100);
    for (;;) {
        busy_wait_us_32(22);
        audioUpdate();
    }
    
}

/*
 * the main loop
 */
void busMainLoop()
{
    for (;;) {
    }
}

int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(400000, true);
    gpio_init_mask(1UL << LED_PIN );
    gpio_set_dir_out_masked(1UL << LED_PIN );

    //for(int i=AD0_PIN; i<(AD0_PIN + 16); ++i) {
    //    gpio_disable_pulls(i);
    //}
    //gpio_disable_pulls(IOW_PIN);
    //gpio_disable_pulls(IOR_PIN);
    //gpio_pull_up(IOCHRDY_PIN);
    gpio_init(RPLY_PIN);
    gpio_set_dir(RPLY_PIN, GPIO_OUT);
    gpio_put(RPLY_PIN, 0);

    gpio_init(SYNC_PIN);
    gpio_set_dir(SYNC_PIN, GPIO_IN);
    gpio_init(BYTE_PIN);
    gpio_set_dir(BYTE_PIN, GPIO_IN);

    //puts("Enabling bus transceivers...");
    // waggle ADS to set BUSOE latch
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 1);

    stdio_init_all();


    //while (!tud_cdc_connected()) { sleep_ms(100);  }
    //printf("tud_cdc_connected()\n");

    //measure_freqs();

    multicore_launch_core1(&play_ay);
    //play_ay();


    puts("Starting MPI bus PIO...");
    // gpio_set_drive_strength(ADS_PIN, GPIO_DRIVE_STRENGTH_12MA);
    //gpio_set_slew_rate(DIR_PIN, GPIO_SLEW_RATE_FAST);

    uint addr_mux_offset = pio_add_program(pio1, &addr_mux_program);
    pio_sm_claim(pio1, ADDR_MUX_PIO_SM);
    printf("addr_mux sm: %u\n", ADDR_MUX_PIO_SM);

    uint iow_offset = pio_add_program(pio0, &iow_program);
    pio_sm_claim(pio0, IOW_PIO_SM);
    printf("iow sm: %u\n", IOW_PIO_SM);

    uint ior_offset = pio_add_program(pio0, &ior_program);
    pio_sm_claim(pio0, IOR_PIO_SM);
    printf("ior sm: %u\n", IOR_PIO_SM);

    uint rply_offset = pio_add_program(pio0, &rply_program);
    pio_sm_claim(pio0, RPLY_PIO_SM);
    printf("rply sm: %u\n", RPLY_PIO_SM);

    uint rply2_offset = pio_add_program(pio0, &rply2_program);
    pio_sm_claim(pio0, RPLY2_PIO_SM);
    printf("rply2 sm: %u\n", RPLY2_PIO_SM);

    addr_mux_program_init(pio1, ADDR_MUX_PIO_SM, addr_mux_offset);

    irq_set_exclusive_handler(PIO1_IRQ_0, addrmux_handler);
    irq_set_priority(PIO1_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(PIO1_IRQ_0, true);

    iow_program_init(pio0, IOW_PIO_SM, iow_offset);

    irq_set_exclusive_handler(PIO0_IRQ_1, handle_iow);
    irq_set_priority(PIO0_IRQ_1, PICO_DEFAULT_IRQ_PRIORITY);
    irq_set_enabled(PIO0_IRQ_1, true);

    ior_program_init(pio0, IOR_PIO_SM, ior_offset);

    irq_set_exclusive_handler(PIO0_IRQ_0, handle_ior);
    irq_set_priority(PIO0_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY);
    irq_set_enabled(PIO0_IRQ_0, true);

    rply_program_init(pio0, RPLY_PIO_SM, rply_offset);
    rply_program_init(pio0, RPLY2_PIO_SM, rply2_offset);

    busMainLoop();

    return 0;
#endif
}