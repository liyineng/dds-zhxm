#include "xil_io.h"

#include "mb_interface.h"

#include "xil_types.h"



/************************************************

 * Base Address

 ************************************************/

#define GPIO_BASE   0x40000000

#define TIMER_BASE  0x41C00000

#define INTC_BASE   0x41200000

#define SPI_BASE    0x44A00000

#define UART_BASE   0x40600000



/************************************************

 * Register Offset

 ************************************************/

#define XTC_TCSR_OFFSET         0x00

#define XTC_TLR_OFFSET          0x04

#define SPICR                   0x60

#define SPISR                   0x64

#define SPIDTR                  0x68

#define SPISSR                  0x70



#define UART_RX_FIFO            0x00

#define UART_TX_FIFO            0x04

#define UART_STATUS             0x08

#define UART_CONTROL            0x0C



#define XIN_IER_OFFSET          0x08

#define XIN_IAR_OFFSET          0x0C

#define XIN_MER_OFFSET          0x1C

#define XIN_IVAR_OFFSET         0x100



// 中断掩码

#define XPAR_GPIO_INTR_ID       0

#define XPAR_TIMER_INTR_ID      1

#define XPAR_UART_INTR_ID       2



#define GPIO_MASK   (1 << XPAR_GPIO_INTR_ID)   // 0x01

#define TIMER_MASK  (1 << XPAR_TIMER_INTR_ID)  // 0x02

#define UART_MASK   (1 << XPAR_UART_INTR_ID)   // 0x04



#define CHA_CMD     0xC000




#define TIMER_CLK_HZ         100000000

#define SAMPLES_PER_CYCLE    128              // 改为128�?

#define DEFAULT_FREQ_HZ      300


#define WAVE_SINE       0

#define WAVE_SQUARE     1

#define WAVE_TRIANGLE   2

#define WAVE_SAWTOOTH   3

#define WAVE_ARBITRARY  4




// Fixed 6-byte frame: FF [FUNC+CH] [D0] [D1] [D2] [D3]
// CMD byte: bits[7:4]=function  bit[3]=channel

#define FRAME_SYNC      0xFF

#define FCMD_WAVE       0x0

#define FCMD_FREQ       0x1

#define FCMD_AMP        0x2

#define FCMD_SYNC       0x3

#define FCMD_FREQ_IDX   0x4




/************************************************

 * 波形查找表（128点，每个值除�?，范�?-127�?

 ************************************************/

const u8 sine_table[128] = {

    64,67,70,73,76,79,82,85,88,91,93,96,99,101,104,106,

    108,111,113,115,116,118,120,121,122,123,124,125,126,126,127,127,

    127,127,127,126,126,125,124,123,122,121,120,118,116,115,113,111,

    108,106,104,101,99,96,93,91,88,85,82,79,76,73,70,67,

    64,60,57,54,51,48,45,42,39,36,34,31,28,26,23,21,

    19,16,14,12,11,9,7,6,5,4,3,2,1,1,0,0,

    0,0,0,1,1,2,3,4,5,6,7,9,11,12,14,16,

    19,21,23,26,28,31,34,36,39,42,45,48,51,54,57,60

};



const u8 square_table[128] = {

    127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,

    127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,

    127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,

    127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,

    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0

};



const u8 triangle_table[128] = {

    0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,

    32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,

    64,65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,

    95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,

    127,125,123,121,119,117,115,113,111,109,107,105,103,101,99,97,

    95,93,91,89,87,85,83,81,79,77,75,73,71,69,67,65,

    64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,

    32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2

};



const u8 sawtooth_table[128] = {

    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,

    16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,

    32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,

    48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,

    64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,

    80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,

    96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,

    112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127

};



u8 arbitrary_table[128];



// 波形指针数组（快速查表）

const u8* wave_tables[5] = { sine_table, square_table, triangle_table, sawtooth_table, arbitrary_table };


const u32 freq_table[14] = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000 };



/************************************************

 * 全局状态变�?

 ************************************************/

volatile u8 table_index_a = 0;


volatile u8 wave_type_a = WAVE_SINE;


volatile u8 amplitude_a = 127;              // 最大幅值改�?27


volatile u32 freq_hz_a = DEFAULT_FREQ_HZ;

volatile u32 new_freq;

volatile u8 freq_update;





volatile u8 uart_frame[5];

volatile u8 uart_idx;

volatile u8 uart_got_sync;



volatile u8 gpio_key_value = 0;

u32 sw = 1;



/************************************************

 * 函数声明

 ************************************************/

void T0Handler(void) __attribute__((fast_interrupt));

void GPIO_Handler(void) __attribute__((fast_interrupt));

void UART_Handler(void) __attribute__((fast_interrupt));



void spi_init(void);

void dac_write_fast(u16 cmd, u8 value);

void uart_send_byte(u8 data);

void process_fixed_frame(u8 cmd_ch, u8* data);

void gpio_init(void);

void timer_init(void);

void uart_init(void);

void intc_init(void);

void arb_table_init(void);


u32 calc_load_value(u32 freq_hz);



/************************************************

 * SPI / DAC 驱动

 ************************************************/

void spi_init(void)

{

    Xil_Out32(SPI_BASE + SPISSR, 0xFFFFFFFE);

    Xil_Out32(SPI_BASE + SPICR, 0x66);

}



void dac_write_fast(u16 cmd, u8 value)

{

    u16 tx = cmd | ((u16)value << 4);

    while(Xil_In32(SPI_BASE + SPISR) & (1<<3));

    Xil_Out32(SPI_BASE + SPIDTR, tx);

}



/************************************************

 * 定时器频率计算（向上计数模式�?

 ************************************************/

u32 calc_load_value(u32 freq_hz)

{

    u32 timer_cnt;

    if(freq_hz == 0) freq_hz = 1;

    if(freq_hz > 20000) freq_hz = 20000;

    timer_cnt = TIMER_CLK_HZ / (freq_hz * SAMPLES_PER_CYCLE);

    if(timer_cnt < 2) timer_cnt = 2;

    return 0xFFFFFFFF - timer_cnt + 1;

}






/************************************************

 * UART 驱动与命令解�?

  ************************************************/



void uart_send_byte(u8 data)

{

    while(!(Xil_In32(UART_BASE + UART_STATUS) & (1<<2)));

    Xil_Out32(UART_BASE + UART_TX_FIFO, data);

}



void process_fixed_frame(u8 cmd_ch, u8* data)

{

    u8 func = (cmd_ch >> 4) & 0x0F;

    u32 freq;

    switch(func)

    {

        case FCMD_WAVE:

            wave_type_a = data[0]; table_index_a = 0;

            break;

        case FCMD_FREQ:

            freq = ((u32)data[0]) | ((u32)data[1] << 8)

                 | ((u32)data[2] << 16) | ((u32)data[3] << 24);

            freq_hz_a = freq; new_freq = freq; freq_update = 1;

            break;

        case FCMD_AMP:

            amplitude_a = data[0];

            break;

        case FCMD_FREQ_IDX:

            if(data[0] > 13) break;

            freq = freq_table[data[0]];

            freq_hz_a = freq; new_freq = freq; freq_update = 1;

            break;

        default:

            if(func == FCMD_SYNC) {

                if(data[0] != 0)

                    sw |=  0x80000000;

                else

                    sw &= ~0x80000000;

            }

            break;

    }

}





// UART interrupt - collect 5 bytes after FF sync

void UART_Handler(void)

{

    while(Xil_In32(UART_BASE + UART_STATUS) & (1<<0))

    {

        u32 status = Xil_In32(UART_BASE + UART_STATUS);

        u8 rx_byte = (u8)Xil_In32(UART_BASE + UART_RX_FIFO);

        if(status & ((1<<5) | (1<<6))) {

            uart_got_sync = 0;

            uart_idx = 0;

            continue;

        }

        if(!uart_got_sync) {

            if(rx_byte == FRAME_SYNC) {

                uart_got_sync = 1;

                uart_idx = 0;

            }

        } else {

            uart_frame[uart_idx] = rx_byte;

            uart_idx++;

            if(uart_idx >= 5) {

                process_fixed_frame(uart_frame[0], &uart_frame[1]);

                uart_got_sync = 0;

                uart_idx = 0;

            }

        }

    }

    Xil_Out32(INTC_BASE + XIN_IAR_OFFSET, UART_MASK);

}



/************************************************

 * GPIO 快速中断（保留调频功能�?

 ************************************************/

void GPIO_Handler(void)

{

//    u32 timer_cnt;

//    u32 load_value;

//    u32 tcsr;

//

    gpio_key_value = Xil_In32(GPIO_BASE + 0x00) & 0xFF;

    if(sw == 0) sw = 1;

//

//    timer_cnt = (23437 + ((390625 - 23437) * sw) / 255) / 100;

//    load_value = 0xFFFFFFFF - timer_cnt + 1;

//

//    tcsr = Xil_In32(TIMER_BASE + XTC_TCSR_OFFSET);

//    Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET, tcsr & (~(1<<7)));

//    Xil_Out32(TIMER_BASE + XTC_TLR_OFFSET, load_value);

//    Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET, tcsr | (1<<5));

//    Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET,

//              (tcsr & (1<<3)) | ((1<<7) | (1<<6) | (1<<4)) & (~(1<<5)));



    Xil_Out32(GPIO_BASE + 0x120, 1);

    Xil_Out32(INTC_BASE + XIN_IAR_OFFSET, GPIO_MASK);

}



/************************************************

 * 定时器快速中断（双通道波形输出�?

 ************************************************/

void T0Handler(void)

{

    u32 tcsr = Xil_In32(TIMER_BASE + XTC_TCSR_OFFSET);

    if(tcsr & (1<<8))

    {

        if(freq_update) {

            freq_update = 0;

            Xil_Out32(TIMER_BASE + XTC_TLR_OFFSET, calc_load_value(new_freq));

        }

        u8 raw_val = wave_tables[wave_type_a][table_index_a];

        u8 out_val = (raw_val * amplitude_a) >> 7;

        dac_write_fast(CHA_CMD, out_val);



        table_index_a++;

        if(table_index_a >= SAMPLES_PER_CYCLE) table_index_a = 0;



        Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET, tcsr | (1<<8));

    }

    Xil_Out32(INTC_BASE + XIN_IAR_OFFSET, TIMER_MASK);

}



/************************************************

 * 初始化函�?

 ************************************************/

void arb_table_init(void)

{

    for(int i = 0; i < 128; i++) arbitrary_table[i] = sine_table[i];

}



void gpio_init(void)

{

    Xil_Out32(GPIO_BASE + 0x04, 0xFFFFFFFF);

    Xil_Out32(GPIO_BASE + 0x11C, 0x80000000);

    Xil_Out32(GPIO_BASE + 0x128, 1);

    Xil_Out32(GPIO_BASE + 0x120, 1);

}



void uart_init(void)

{

    Xil_Out32(UART_BASE + UART_CONTROL, 0x03);

    Xil_Out32(UART_BASE + UART_CONTROL, 0x10);

}



void timer_init(void)

{

    u32 tcsr = Xil_In32(TIMER_BASE + XTC_TCSR_OFFSET);

    u32 load_value = calc_load_value(DEFAULT_FREQ_HZ);




    Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET, tcsr & (~(1<<7)));

    Xil_Out32(TIMER_BASE + XTC_TLR_OFFSET, load_value);

    Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET, tcsr | (1<<5));

    Xil_Out32(TIMER_BASE + XTC_TCSR_OFFSET, ((1<<7) | (1<<6) | (1<<4) | (1<<3)) & (~(1<<5)));

}



void intc_init(void)

{

    Xil_Out32(INTC_BASE + XIN_IAR_OFFSET, GPIO_MASK | TIMER_MASK | UART_MASK);



    Xil_Out32(INTC_BASE + XIN_IER_OFFSET, GPIO_MASK | TIMER_MASK | UART_MASK);

    Xil_Out32(INTC_BASE + XIN_MER_OFFSET, 0x03);

    Xil_Out32(INTC_BASE + XIN_IVAR_OFFSET + XPAR_GPIO_INTR_ID * 4, (int)GPIO_Handler);

    Xil_Out32(INTC_BASE + XIN_IVAR_OFFSET + XPAR_TIMER_INTR_ID * 4, (int)T0Handler);

    Xil_Out32(INTC_BASE + XIN_IVAR_OFFSET + XPAR_UART_INTR_ID * 4, (int)UART_Handler);

    microblaze_enable_interrupts();

}



/************************************************

 * 主函�?

 ************************************************/

int main(void)

{

    arb_table_init();

    spi_init();

    gpio_init();

    uart_init();

    timer_init();

    intc_init();



    while(1){

//    for(int i=0;i<10000;i++);

//

    };

    return 0;

}
