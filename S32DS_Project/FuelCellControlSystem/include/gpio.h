#ifndef GPIO_H_
#define GPIO_H_

#define GPIO_BUTTON_R		12
#define GPIO_RGB_BLUE		0
#define GPIO_RGB_RED        15
#define GPIO_RGB_GREEN      16
#define GPIO_BUTTON_INTPRI	6

void GPIO_Init();
void GPIO_ButtonIRQHandler();

#endif /* GPIO_H_ */
