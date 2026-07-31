#include "sys.h"
#include "usart.h"

#ifdef __CC_ARM
#pragma import(__use_no_semihosting)
#endif
struct __FILE
{
    int handle;
};
FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    /* Keep PA9/USART1 as the board's CH340 diagnostic output. */
    while((USART1->SR & 0x40U) == 0U)
    {
    }
    USART1->DR = (u8)ch;
    return ch;
}

u8 USART_RX_BUF[USART_REC_LEN];
u16 USART_RX_STA;
/* Incremented for every physical byte received from K230 on PA3. */
volatile u16 K230_RAW_RX_COUNT;

void uart_init(u32 bound)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    /* K230 TX1 is physically wired to PA3 = USART2_RX.  PA10 stays with
     * the board CH340 circuit and must not be re-used as a K230 receiver. */
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3U;
    nvic.NVIC_IRQChannelSubPriority = 3U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    uart.USART_BaudRate = bound;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);

    uart.USART_Mode = USART_Mode_Rx;
    USART_Init(USART2, &uart);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void)
{
    u8 value;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        value = (u8)USART_ReceiveData(USART2);
        K230_RAW_RX_COUNT++;

        /* Preserve the CR/LF frame buffer used by the ball packet parser. */
        if((USART_RX_STA & 0x8000U) == 0U)
        {
            if((USART_RX_STA & 0x4000U) != 0U)
            {
                if(value == 0x0AU)
                    USART_RX_STA |= 0x8000U;
                else
                    USART_RX_STA = 0U;
            }
            else if(value == 0x0DU)
            {
                USART_RX_STA |= 0x4000U;
            }
            else
            {
                USART_RX_BUF[USART_RX_STA & 0x3FFFU] = value;
                USART_RX_STA++;
                if((USART_RX_STA & 0x3FFFU) >= (USART_REC_LEN - 1U))
                    USART_RX_STA = 0U;
            }
        }
    }
}
