#include "stm32f407xx.h"
#include "rcc_config.h"
#include "delay.h"

void GPIOB_PORT_INIT(void);
void CAN1_INIT(void);
void CAN1_TRANSMITT(uint8_t OUT_DATA);
uint8_t CAN1_RECIEVE(void);
void TIM4_MSDLY(uint32_t due);

static uint8_t cnt = 0;
static uint8_t can_one_data_rx = 0;

void GPIOB_PORT_INIT(void)
{
    RCC->AHB1ENR |= 1<<1;
    GPIOB->MODER |= (2<<16) | (2<<18); // PB8 - CAN1_RX PB9 - CAN1_TX
    GPIOB->AFR[1] |= (0x9UL<<0) | (0x9UL<<4); // PB8 - CAN1_RX PB9 - CAN1_TX
}

void CAN1_INIT(void)
{
    RCC->APB1ENR |= 1<<25;
    CAN1->MCR |= 1<<0; // To enable debug while CAN is working
    CAN1->MCR &= ~(1<<16); // To enable debug while CAN is working
    CAN1->BTR |= 0x1UL<<30; // loop back mode (comment means normal mode)CAN1->BTR &= ~(0x3UL<<24);
    CAN1->BTR |= (0x3UL<<24); // sjw
    CAN1->BTR &= ~(0x7UL<<20);
    CAN1->BTR |= (0x3UL<<20 ); // btr
    CAN1->BTR &= ~(0xFUL <<16);
    CAN1->BTR |= (0x2UL<<16); // prescaler
    CAN1->BTR |= (15<<0); // seg1
    CAN1->MCR &= ~(0x1UL<<0); // To enable debug while CAN is working
    while (CAN1->MSR & 0x1UL<<0);
    CAN1->MCR &= ~(0x1UL<<1); // To enable debug while CAN is working
    while (CAN1->MSR & 0x1UL<<1);
    CAN1->sTxMailBox[0].TIR = 0;
    CAN1->sTxMailBox[0].TIR = (0x245<<21);
    CAN1->sTxMailBox[0].TDHR = 0;
    CAN1->sTxMailBox[0].TDTR =2;
    CAN1->FMR |= (0x1UL<<0);
    CAN1->FMR |= 14<<8;
    CAN1->FS1R |= (0x1UL<<13);
    CAN1->sFilterRegister[13].FR1 = 0x245<<21;
    CAN1->FM1R |= 0x1UL<<13;
    CAN1->FA1R |= 0x1UL<<13;
    CAN1->FMR &= ~(0x1UL<<0);
}

void CAN1_TRANSMITT(uint8_t OUT_DATA)
{
    CAN1->sTxMailBox[0].TDLR = OUT_DATA;
    CAN1->sTxMailBox[0].TIR |= 1;
}

uint8_t CAN1_RECIEVE(void)
{
    uint8_t IN_DATA=0;
   while(!(CAN1->RF0R & 3));
    IN_DATA = (CAN1->sFIFOMailBox[0].RDLR) & 0xFF;
    can_one_data_rx = IN_DATA;
    CAN1->RF0R |= 1<<5;
    return IN_DATA;
}

int main(void)
{
    GPIOB_PORT_INIT();
    CAN1_INIT();
    while(1)
    {
        CAN1_TRANSMITT(100);
        TIM4_MSDLY(1000);
        can_one_data_rx = CAN1_RECIEVE();
    }
}