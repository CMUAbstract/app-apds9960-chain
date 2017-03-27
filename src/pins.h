#ifndef PIN_ASSIGN_H
#define PIN_ASSIGN_H

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT

#define BIT_INNER(idx) BIT ## idx
#define BIT(idx) BIT_INNER(idx)

#define GPIO_INNER(port, reg) P ## port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)

#if defined(BOARD_WISP)
#define     PORT_LED_1           4
#define     PIN_LED_1            0
#define     PORT_LED_2           J
#define     PIN_LED_2            6

#define     PORT_AUX            3
#define 		PIN_AUX_0						3
#define     PIN_AUX_1           4
#define     PIN_AUX_2           5


#define     PORT_AUX3           1
#define     PIN_AUX_3           4

#elif defined(BOARD_MSP_TS430)

#define     PORT_LED_1           1
#define     PIN_LED_1            0
#define     PORT_LED_2           4
#define     PIN_LED_2            6
#define     PORT_LED_3           1
#define     PIN_LED_3            0

#define     PORT_AUX_1            3
#define 		PIN_AUX_0						6
#define     PIN_AUX_1           4
#define     PIN_AUX_2           5


#define     PORT_AUX3           1
#define     PIN_AUX_3_3           4

#define 		PORT_RESET					3
#define 		PIN_RESET						5
#define 		PORT_SET 						3
#define 		PIN_SET						6

#define     PORT_AUX            1
#define     PIN_AUX_3           3
#define     PIN_AUX_4           4
#define			PIN_AUX_5					  5	


#elif defined(BOARD_SPRITE_APP_SOCKET_RHA)

#define     PORT_LED_1           1
#define     PIN_LED_1            0
#define     PORT_LED_2           1
#define     PIN_LED_2            2
#define     PORT_LED_3           1
#define     PIN_LED_3            0


#endif // BOARD_*

#endif
