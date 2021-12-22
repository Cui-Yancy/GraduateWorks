#include "gpio.h"
#include "Cpu.h"

void GPIO_Init()
{
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);

    PINS_DRV_SetPinIntSel(PORTC, GPIO_BUTTON_R, PORT_INT_RISING_EDGE);

    INT_SYS_InstallHandler(PORTC_IRQn,&GPIO_ButtonIRQHandler,NULL);
    INT_SYS_SetPriority(PORTC_IRQn,GPIO_BUTTON_INTPRI);
    INT_SYS_EnableIRQ(PORTC_IRQn);
}

void GPIO_ButtonIRQHandler(){
    uint32_t ButtonPressed = PINS_DRV_GetPortIntFlag(PORTC) & (1<<GPIO_BUTTON_R);
    if(ButtonPressed != 0){
        switch(ButtonPressed)
        {
        case(1<<GPIO_BUTTON_R):
            PINS_DRV_TogglePins(PTD,1<<GPIO_RGB_BLUE);
            break;
        default:
            break;
        }
    }
    PINS_DRV_ClearPortIntFlagCmd(PORTC);
}
