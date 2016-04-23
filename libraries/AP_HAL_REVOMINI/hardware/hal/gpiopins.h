/* Abstract bitwise GPIO services */

// $Id: PINs.h 3110 2011-10-06 13:25:49Z svn $

#ifndef _PINS_H
#define _PINS_H

#include <_ansi.h>

// These services provide a simple and efficient mechanism for software to
// control individual GPIO pins in an asynchronous fashion, where each GPIO pin
// is independent of any other.  This API is NOT suitable for situations where
// GPIO pins are grouped in buses

// GPIO pins are numbered sequentially from the least significant bit of the
// lowest GPIO port to the highest port and bit.  For example, if an MCU has
// two 8-bit GPIO ports, the GPIO pins would be numbered GPIO0 to GPIO15.

// Each GPIO pin has two macro defined for it: GPIOnIN and GPIOnOUT.  These
// map to Cortex-M4 bitband registers for the particular GPIO pin.  Sadly,
// some Cortex-M4 devices (such as the STM32F4) do not have a single
// GPIO register that can be both written to and read from, thus preventing
// us from just defining a macro "GPIOn" for each GPIO pin.

typedef enum
{
  PIN_INPUT,
  PIN_OUTPUT,
} PIN_direction_t;

int PIN_configure(unsigned int pin, PIN_direction_t direction);

// PA0 is PIN0
#define PIN0	0
#define PIN0IN	(*((unsigned long int *) 0x42400200))
#define PIN0OUT	(*((unsigned long int *) 0x42400280))

// PA1 is PIN1
#define PIN1	1
#define PIN1IN	(*((unsigned long int *) 0x42400204))
#define PIN1OUT	(*((unsigned long int *) 0x42400284))

// PA2 is PIN2
#define PIN2	2
#define PIN2IN	(*((unsigned long int *) 0x42400208))
#define PIN2OUT	(*((unsigned long int *) 0x42400288))

// PA3 is PIN3
#define PIN3	3
#define PIN3IN	(*((unsigned long int *) 0x4240020C))
#define PIN3OUT	(*((unsigned long int *) 0x4240028C))

// PA4 is PIN4
#define PIN4	4
#define PIN4IN	(*((unsigned long int *) 0x42400210))
#define PIN4OUT	(*((unsigned long int *) 0x42400290))

// PA5 is PIN5
#define PIN5	5
#define PIN5IN	(*((unsigned long int *) 0x42400214))
#define PIN5OUT	(*((unsigned long int *) 0x42400294))

// PA6 is PIN6
#define PIN6	6
#define PIN6IN	(*((unsigned long int *) 0x42400218))
#define PIN6OUT	(*((unsigned long int *) 0x42400298))

// PA7 is PIN7
#define PIN7	7
#define PIN7IN	(*((unsigned long int *) 0x4240021C))
#define PIN7OUT	(*((unsigned long int *) 0x4240029C))

// PA8 is PIN8
#define PIN8	8
#define PIN8IN	(*((unsigned long int *) 0x42400220))
#define PIN8OUT	(*((unsigned long int *) 0x424002A0))

// PA9 is PIN9
#define PIN9	9
#define PIN9IN	(*((unsigned long int *) 0x42400224))
#define PIN9OUT	(*((unsigned long int *) 0x424002A4))

// PA10 is PIN10
#define PIN10	10
#define PIN10IN	(*((unsigned long int *) 0x42400228))
#define PIN10OUT	(*((unsigned long int *) 0x424002A8))

// PA11 is PIN11
#define PIN11	11
#define PIN11IN	(*((unsigned long int *) 0x4240022C))
#define PIN11OUT	(*((unsigned long int *) 0x424002AC))

// PA12 is PIN12
#define PIN12	12
#define PIN12IN	(*((unsigned long int *) 0x42400230))
#define PIN12OUT	(*((unsigned long int *) 0x424002B0))

// PA13 is PIN13
#define PIN13	13
#define PIN13IN	(*((unsigned long int *) 0x42400234))
#define PIN13OUT	(*((unsigned long int *) 0x424002B4))

// PA14 is PIN14
#define PIN14	14
#define PIN14IN	(*((unsigned long int *) 0x42400238))
#define PIN14OUT	(*((unsigned long int *) 0x424002B8))

// PA15 is PIN15
#define PIN15	15
#define PIN15IN	(*((unsigned long int *) 0x4240023C))
#define PIN15OUT	(*((unsigned long int *) 0x424002BC))

// PB0 is PIN16
#define PIN16	16
#define PIN16IN	(*((unsigned long int *) 0x42408200))
#define PIN16OUT	(*((unsigned long int *) 0x42408280))

// PB1 is PIN17
#define PIN17	17
#define PIN17IN	(*((unsigned long int *) 0x42408204))
#define PIN17OUT	(*((unsigned long int *) 0x42408284))

// PB2 is PIN18
#define PIN18	18
#define PIN18IN	(*((unsigned long int *) 0x42408208))
#define PIN18OUT	(*((unsigned long int *) 0x42408288))

// PB3 is PIN19
#define PIN19	19
#define PIN19IN	(*((unsigned long int *) 0x4240820C))
#define PIN19OUT	(*((unsigned long int *) 0x4240828C))

// PB4 is PIN20
#define PIN20	20
#define PIN20IN	(*((unsigned long int *) 0x42408210))
#define PIN20OUT	(*((unsigned long int *) 0x42408290))

// PB5 is PIN21
#define PIN21	21
#define PIN21IN	(*((unsigned long int *) 0x42408214))
#define PIN21OUT	(*((unsigned long int *) 0x42408294))

// PB6 is PIN22
#define PIN22	22
#define PIN22IN	(*((unsigned long int *) 0x42408218))
#define PIN22OUT	(*((unsigned long int *) 0x42408298))

// PB7 is PIN23
#define PIN23	23
#define PIN23IN	(*((unsigned long int *) 0x4240821C))
#define PIN23OUT	(*((unsigned long int *) 0x4240829C))

// PB8 is PIN24
#define PIN24	24
#define PIN24IN	(*((unsigned long int *) 0x42408220))
#define PIN24OUT	(*((unsigned long int *) 0x424082A0))

// PB9 is PIN25
#define PIN25	25
#define PIN25IN	(*((unsigned long int *) 0x42408224))
#define PIN25OUT	(*((unsigned long int *) 0x424082A4))

// PB10 is PIN26
#define PIN26	26
#define PIN26IN	(*((unsigned long int *) 0x42408228))
#define PIN26OUT	(*((unsigned long int *) 0x424082A8))

// PB11 is PIN27
#define PIN27	27
#define PIN27IN	(*((unsigned long int *) 0x4240822C))
#define PIN27OUT	(*((unsigned long int *) 0x424082AC))

// PB12 is PIN28
#define PIN28	28
#define PIN28IN	(*((unsigned long int *) 0x42408230))
#define PIN28OUT	(*((unsigned long int *) 0x424082B0))

// PB13 is PIN29
#define PIN29	29
#define PIN29IN	(*((unsigned long int *) 0x42408234))
#define PIN29OUT	(*((unsigned long int *) 0x424082B4))

// PB14 is PIN30
#define PIN30	30
#define PIN30IN	(*((unsigned long int *) 0x42408238))
#define PIN30OUT	(*((unsigned long int *) 0x424082B8))

// PB15 is PIN31
#define PIN31	31
#define PIN31IN	(*((unsigned long int *) 0x4240823C))
#define PIN31OUT	(*((unsigned long int *) 0x424082BC))

// PC0 is PIN32
#define PIN32	32
#define PIN32IN	(*((unsigned long int *) 0x42410200))
#define PIN32OUT	(*((unsigned long int *) 0x42410280))

// PC1 is PIN33
#define PIN33	33
#define PIN33IN	(*((unsigned long int *) 0x42410204))
#define PIN33OUT	(*((unsigned long int *) 0x42410284))

// PC2 is PIN34
#define PIN34	34
#define PIN34IN	(*((unsigned long int *) 0x42410208))
#define PIN34OUT	(*((unsigned long int *) 0x42410288))

// PC3 is PIN35
#define PIN35	35
#define PIN35IN	(*((unsigned long int *) 0x4241020C))
#define PIN35OUT	(*((unsigned long int *) 0x4241028C))

// PC4 is PIN36
#define PIN36	36
#define PIN36IN	(*((unsigned long int *) 0x42410210))
#define PIN36OUT	(*((unsigned long int *) 0x42410290))

// PC5 is PIN37
#define PIN37	37
#define PIN37IN	(*((unsigned long int *) 0x42410214))
#define PIN37OUT	(*((unsigned long int *) 0x42410294))

// PC6 is PIN38
#define PIN38	38
#define PIN38IN	(*((unsigned long int *) 0x42410218))
#define PIN38OUT	(*((unsigned long int *) 0x42410298))

// PC7 is PIN39
#define PIN39	39
#define PIN39IN	(*((unsigned long int *) 0x4241021C))
#define PIN39OUT	(*((unsigned long int *) 0x4241029C))

// PC8 is PIN40
#define PIN40	40
#define PIN40IN	(*((unsigned long int *) 0x42410220))
#define PIN40OUT	(*((unsigned long int *) 0x424102A0))

// PC9 is PIN41
#define PIN41	41
#define PIN41IN	(*((unsigned long int *) 0x42410224))
#define PIN41OUT	(*((unsigned long int *) 0x424102A4))

// PC10 is PIN42
#define PIN42	42
#define PIN42IN	(*((unsigned long int *) 0x42410228))
#define PIN42OUT	(*((unsigned long int *) 0x424102A8))

// PC11 is PIN43
#define PIN43	43
#define PIN43IN	(*((unsigned long int *) 0x4241022C))
#define PIN43OUT	(*((unsigned long int *) 0x424102AC))

// PC12 is PIN44
#define PIN44	44
#define PIN44IN	(*((unsigned long int *) 0x42410230))
#define PIN44OUT	(*((unsigned long int *) 0x424102B0))

// PC13 is PIN45
#define PIN45	45
#define PIN45IN	(*((unsigned long int *) 0x42410234))
#define PIN45OUT	(*((unsigned long int *) 0x424102B4))

// PC14 is PIN46
#define PIN46	46
#define PIN46IN	(*((unsigned long int *) 0x42410238))
#define PIN46OUT	(*((unsigned long int *) 0x424102B8))

// PC15 is PIN47
#define PIN47	47
#define PIN47IN	(*((unsigned long int *) 0x4241023C))
#define PIN47OUT	(*((unsigned long int *) 0x424102BC))

// PD0 is PIN48
#define PIN48	48
#define PIN48IN	(*((unsigned long int *) 0x42418200))
#define PIN48OUT	(*((unsigned long int *) 0x42418280))

// PD1 is PIN49
#define PIN49	49
#define PIN49IN	(*((unsigned long int *) 0x42418204))
#define PIN49OUT	(*((unsigned long int *) 0x42418284))

// PD2 is PIN50
#define PIN50	50
#define PIN50IN	(*((unsigned long int *) 0x42418208))
#define PIN50OUT	(*((unsigned long int *) 0x42418288))

// PD3 is PIN51
#define PIN51	51
#define PIN51IN	(*((unsigned long int *) 0x4241820C))
#define PIN51OUT	(*((unsigned long int *) 0x4241828C))

// PD4 is PIN52
#define PIN52	52
#define PIN52IN	(*((unsigned long int *) 0x42418210))
#define PIN52OUT	(*((unsigned long int *) 0x42418290))

// PD5 is PIN53
#define PIN53	53
#define PIN53IN	(*((unsigned long int *) 0x42418214))
#define PIN53OUT	(*((unsigned long int *) 0x42418294))

// PD6 is PIN54
#define PIN54	54
#define PIN54IN	(*((unsigned long int *) 0x42418218))
#define PIN54OUT	(*((unsigned long int *) 0x42418298))

// PD7 is PIN55
#define PIN55	55
#define PIN55IN	(*((unsigned long int *) 0x4241821C))
#define PIN55OUT	(*((unsigned long int *) 0x4241829C))

// PD8 is PIN56
#define PIN56	56
#define PIN56IN	(*((unsigned long int *) 0x42418220))
#define PIN56OUT	(*((unsigned long int *) 0x424182A0))

// PD9 is PIN57
#define PIN57	57
#define PIN57IN	(*((unsigned long int *) 0x42418224))
#define PIN57OUT	(*((unsigned long int *) 0x424182A4))

// PD10 is PIN58
#define PIN58	58
#define PIN58IN	(*((unsigned long int *) 0x42418228))
#define PIN58OUT	(*((unsigned long int *) 0x424182A8))

// PD11 is PIN59
#define PIN59	59
#define PIN59IN	(*((unsigned long int *) 0x4241822C))
#define PIN59OUT	(*((unsigned long int *) 0x424182AC))

// PD12 is PIN60
#define PIN60	60
#define PIN60IN	(*((unsigned long int *) 0x42418230))
#define PIN60OUT	(*((unsigned long int *) 0x424182B0))

// PD13 is PIN61
#define PIN61	61
#define PIN61IN	(*((unsigned long int *) 0x42418234))
#define PIN61OUT	(*((unsigned long int *) 0x424182B4))

// PD14 is PIN62
#define PIN62	62
#define PIN62IN	(*((unsigned long int *) 0x42418238))
#define PIN62OUT	(*((unsigned long int *) 0x424182B8))

// PD15 is PIN63
#define PIN63	63
#define PIN63IN	(*((unsigned long int *) 0x4241823C))
#define PIN63OUT	(*((unsigned long int *) 0x424182BC))

// PE0 is PIN64
#define PIN64	64
#define PIN64IN	(*((unsigned long int *) 0x42420200))
#define PIN64OUT	(*((unsigned long int *) 0x42420280))

// PE1 is PIN65
#define PIN65	65
#define PIN65IN	(*((unsigned long int *) 0x42420204))
#define PIN65OUT	(*((unsigned long int *) 0x42420284))

// PE2 is PIN66
#define PIN66	66
#define PIN66IN	(*((unsigned long int *) 0x42420208))
#define PIN66OUT	(*((unsigned long int *) 0x42420288))

// PE3 is PIN67
#define PIN67	67
#define PIN67IN	(*((unsigned long int *) 0x4242020C))
#define PIN67OUT	(*((unsigned long int *) 0x4242028C))

// PE4 is PIN68
#define PIN68	68
#define PIN68IN	(*((unsigned long int *) 0x42420210))
#define PIN68OUT	(*((unsigned long int *) 0x42420290))

// PE5 is PIN69
#define PIN69	69
#define PIN69IN	(*((unsigned long int *) 0x42420214))
#define PIN69OUT	(*((unsigned long int *) 0x42420294))

// PE6 is PIN70
#define PIN70	70
#define PIN70IN	(*((unsigned long int *) 0x42420218))
#define PIN70OUT	(*((unsigned long int *) 0x42420298))

// PE7 is PIN71
#define PIN71	71
#define PIN71IN	(*((unsigned long int *) 0x4242021C))
#define PIN71OUT	(*((unsigned long int *) 0x4242029C))

// PE8 is PIN72
#define PIN72	72
#define PIN72IN	(*((unsigned long int *) 0x42420220))
#define PIN72OUT	(*((unsigned long int *) 0x424202A0))

// PE9 is PIN73
#define PIN73	73
#define PIN73IN	(*((unsigned long int *) 0x42420224))
#define PIN73OUT	(*((unsigned long int *) 0x424202A4))

// PE10 is PIN74
#define PIN74	74
#define PIN74IN	(*((unsigned long int *) 0x42420228))
#define PIN74OUT	(*((unsigned long int *) 0x424202A8))

// PE11 is PIN75
#define PIN75	75
#define PIN75IN	(*((unsigned long int *) 0x4242022C))
#define PIN75OUT	(*((unsigned long int *) 0x424202AC))

// PE12 is PIN76
#define PIN76	76
#define PIN76IN	(*((unsigned long int *) 0x42420230))
#define PIN76OUT	(*((unsigned long int *) 0x424202B0))

// PE13 is PIN77
#define PIN77	77
#define PIN77IN	(*((unsigned long int *) 0x42420234))
#define PIN77OUT	(*((unsigned long int *) 0x424202B4))

// PE14 is PIN78
#define PIN78	78
#define PIN78IN	(*((unsigned long int *) 0x42420238))
#define PIN78OUT	(*((unsigned long int *) 0x424202B8))

// PE15 is PIN79
#define PIN79	79
#define PIN79IN	(*((unsigned long int *) 0x4242023C))
#define PIN79OUT	(*((unsigned long int *) 0x424202BC))

// PF0 is PIN80
#define PIN80	80
#define PIN80IN	(*((unsigned long int *) 0x42428200))
#define PIN80OUT	(*((unsigned long int *) 0x42428280))

// PF1 is PIN81
#define PIN81	81
#define PIN81IN	(*((unsigned long int *) 0x42428204))
#define PIN81OUT	(*((unsigned long int *) 0x42428284))

// PF2 is PIN82
#define PIN82	82
#define PIN82IN	(*((unsigned long int *) 0x42428208))
#define PIN82OUT	(*((unsigned long int *) 0x42428288))

// PF3 is PIN83
#define PIN83	83
#define PIN83IN	(*((unsigned long int *) 0x4242820C))
#define PIN83OUT	(*((unsigned long int *) 0x4242828C))

// PF4 is PIN84
#define PIN84	84
#define PIN84IN	(*((unsigned long int *) 0x42428210))
#define PIN84OUT	(*((unsigned long int *) 0x42428290))

// PF5 is PIN85
#define PIN85	85
#define PIN85IN	(*((unsigned long int *) 0x42428214))
#define PIN85OUT	(*((unsigned long int *) 0x42428294))

// PF6 is PIN86
#define PIN86	86
#define PIN86IN	(*((unsigned long int *) 0x42428218))
#define PIN86OUT	(*((unsigned long int *) 0x42428298))

// PF7 is PIN87
#define PIN87	87
#define PIN87IN	(*((unsigned long int *) 0x4242821C))
#define PIN87OUT	(*((unsigned long int *) 0x4242829C))

// PF8 is PIN88
#define PIN88	88
#define PIN88IN	(*((unsigned long int *) 0x42428220))
#define PIN88OUT	(*((unsigned long int *) 0x424282A0))

// PF9 is PIN89
#define PIN89	89
#define PIN89IN	(*((unsigned long int *) 0x42428224))
#define PIN89OUT	(*((unsigned long int *) 0x424282A4))

// PF10 is PIN90
#define PIN90	90
#define PIN90IN	(*((unsigned long int *) 0x42428228))
#define PIN90OUT	(*((unsigned long int *) 0x424282A8))

// PF11 is PIN91
#define PIN91	91
#define PIN91IN	(*((unsigned long int *) 0x4242822C))
#define PIN91OUT	(*((unsigned long int *) 0x424282AC))

// PF12 is PIN92
#define PIN92	92
#define PIN92IN	(*((unsigned long int *) 0x42428230))
#define PIN92OUT	(*((unsigned long int *) 0x424282B0))

// PF13 is PIN93
#define PIN93	93
#define PIN93IN	(*((unsigned long int *) 0x42428234))
#define PIN93OUT	(*((unsigned long int *) 0x424282B4))

// PF14 is PIN94
#define PIN94	94
#define PIN94IN	(*((unsigned long int *) 0x42428238))
#define PIN94OUT	(*((unsigned long int *) 0x424282B8))

// PF15 is PIN95
#define PIN95	95
#define PIN95IN	(*((unsigned long int *) 0x4242823C))
#define PIN95OUT	(*((unsigned long int *) 0x424282BC))

// PG0 is PIN96
#define PIN96	96
#define PIN96IN	(*((unsigned long int *) 0x42430200))
#define PIN96OUT	(*((unsigned long int *) 0x42430280))

// PG1 is PIN97
#define PIN97	97
#define PIN97IN	(*((unsigned long int *) 0x42430204))
#define PIN97OUT	(*((unsigned long int *) 0x42430284))

// PG2 is PIN98
#define PIN98	98
#define PIN98IN	(*((unsigned long int *) 0x42430208))
#define PIN98OUT	(*((unsigned long int *) 0x42430288))

// PG3 is PIN99
#define PIN99	99
#define PIN99IN	(*((unsigned long int *) 0x4243020C))
#define PIN99OUT	(*((unsigned long int *) 0x4243028C))

// PG4 is PIN100
#define PIN100	100
#define PIN100IN	(*((unsigned long int *) 0x42430210))
#define PIN100OUT	(*((unsigned long int *) 0x42430290))

// PG5 is PIN101
#define PIN101	101
#define PIN101IN	(*((unsigned long int *) 0x42430214))
#define PIN101OUT	(*((unsigned long int *) 0x42430294))

// PG6 is PIN102
#define PIN102	102
#define PIN102IN	(*((unsigned long int *) 0x42430218))
#define PIN102OUT	(*((unsigned long int *) 0x42430298))

// PG7 is PIN103
#define PIN103	103
#define PIN103IN	(*((unsigned long int *) 0x4243021C))
#define PIN103OUT	(*((unsigned long int *) 0x4243029C))

// PG8 is PIN104
#define PIN104	104
#define PIN104IN	(*((unsigned long int *) 0x42430220))
#define PIN104OUT	(*((unsigned long int *) 0x424302A0))

// PG9 is PIN105
#define PIN105	105
#define PIN105IN	(*((unsigned long int *) 0x42430224))
#define PIN105OUT	(*((unsigned long int *) 0x424302A4))

// PG10 is PIN106
#define PIN106	106
#define PIN106IN	(*((unsigned long int *) 0x42430228))
#define PIN106OUT	(*((unsigned long int *) 0x424302A8))

// PG11 is PIN107
#define PIN107	107
#define PIN107IN	(*((unsigned long int *) 0x4243022C))
#define PIN107OUT	(*((unsigned long int *) 0x424302AC))

// PG12 is PIN108
#define PIN108	108
#define PIN108IN	(*((unsigned long int *) 0x42430230))
#define PIN108OUT	(*((unsigned long int *) 0x424302B0))

// PG13 is PIN109
#define PIN109	109
#define PIN109IN	(*((unsigned long int *) 0x42430234))
#define PIN109OUT	(*((unsigned long int *) 0x424302B4))

// PG14 is PIN110
#define PIN110	110
#define PIN110IN	(*((unsigned long int *) 0x42430238))
#define PIN110OUT	(*((unsigned long int *) 0x424302B8))

// PG15 is PIN111
#define PIN111	111
#define PIN111IN	(*((unsigned long int *) 0x4243023C))
#define PIN111OUT	(*((unsigned long int *) 0x424302BC))

// PH0 is PIN112
#define PIN112	112
#define PIN112IN	(*((unsigned long int *) 0x42438200))
#define PIN112OUT	(*((unsigned long int *) 0x42438280))

// PH1 is PIN113
#define PIN113	113
#define PIN113IN	(*((unsigned long int *) 0x42438204))
#define PIN113OUT	(*((unsigned long int *) 0x42438284))

// PH2 is PIN114
#define PIN114	114
#define PIN114IN	(*((unsigned long int *) 0x42438208))
#define PIN114OUT	(*((unsigned long int *) 0x42438288))

// PH3 is PIN115
#define PIN115	115
#define PIN115IN	(*((unsigned long int *) 0x4243820C))
#define PIN115OUT	(*((unsigned long int *) 0x4243828C))

// PH4 is PIN116
#define PIN116	116
#define PIN116IN	(*((unsigned long int *) 0x42438210))
#define PIN116OUT	(*((unsigned long int *) 0x42438290))

// PH5 is PIN117
#define PIN117	117
#define PIN117IN	(*((unsigned long int *) 0x42438214))
#define PIN117OUT	(*((unsigned long int *) 0x42438294))

// PH6 is PIN118
#define PIN118	118
#define PIN118IN	(*((unsigned long int *) 0x42438218))
#define PIN118OUT	(*((unsigned long int *) 0x42438298))

// PH7 is PIN119
#define PIN119	119
#define PIN119IN	(*((unsigned long int *) 0x4243821C))
#define PIN119OUT	(*((unsigned long int *) 0x4243829C))

// PH8 is PIN120
#define PIN120	120
#define PIN120IN	(*((unsigned long int *) 0x42438220))
#define PIN120OUT	(*((unsigned long int *) 0x424382A0))

// PH9 is PIN121
#define PIN121	121
#define PIN121IN	(*((unsigned long int *) 0x42438224))
#define PIN121OUT	(*((unsigned long int *) 0x424382A4))

// PH10 is PIN122
#define PIN122	122
#define PIN122IN	(*((unsigned long int *) 0x42438228))
#define PIN122OUT	(*((unsigned long int *) 0x424382A8))

// PH11 is PIN123
#define PIN123	123
#define PIN123IN	(*((unsigned long int *) 0x4243822C))
#define PIN123OUT	(*((unsigned long int *) 0x424382AC))

// PH12 is PIN124
#define PIN124	124
#define PIN124IN	(*((unsigned long int *) 0x42438230))
#define PIN124OUT	(*((unsigned long int *) 0x424382B0))

// PH13 is PIN125
#define PIN125	125
#define PIN125IN	(*((unsigned long int *) 0x42438234))
#define PIN125OUT	(*((unsigned long int *) 0x424382B4))

// PH14 is PIN126
#define PIN126	126
#define PIN126IN	(*((unsigned long int *) 0x42438238))
#define PIN126OUT	(*((unsigned long int *) 0x424382B8))

// PH15 is PIN127
#define PIN127	127
#define PIN127IN	(*((unsigned long int *) 0x4243823C))
#define PIN127OUT	(*((unsigned long int *) 0x424382BC))

// PI0 is PIN128
#define PIN128	128
#define PIN128IN	(*((unsigned long int *) 0x42440200))
#define PIN128OUT	(*((unsigned long int *) 0x42440280))

// PI1 is PIN129
#define PIN129	129
#define PIN129IN	(*((unsigned long int *) 0x42440204))
#define PIN129OUT	(*((unsigned long int *) 0x42440284))

// PI2 is PIN130
#define PIN130	130
#define PIN130IN	(*((unsigned long int *) 0x42440208))
#define PIN130OUT	(*((unsigned long int *) 0x42440288))

// PI3 is PIN131
#define PIN131	131
#define PIN131IN	(*((unsigned long int *) 0x4244020C))
#define PIN131OUT	(*((unsigned long int *) 0x4244028C))

// PI4 is PIN132
#define PIN132	132
#define PIN132IN	(*((unsigned long int *) 0x42440210))
#define PIN132OUT	(*((unsigned long int *) 0x42440290))

// PI5 is PIN133
#define PIN133	133
#define PIN133IN	(*((unsigned long int *) 0x42440214))
#define PIN133OUT	(*((unsigned long int *) 0x42440294))

// PI6 is PIN134
#define PIN134	134
#define PIN134IN	(*((unsigned long int *) 0x42440218))
#define PIN134OUT	(*((unsigned long int *) 0x42440298))

// PI7 is PIN135
#define PIN135	135
#define PIN135IN	(*((unsigned long int *) 0x4244021C))
#define PIN135OUT	(*((unsigned long int *) 0x4244029C))

// PI8 is PIN136
#define PIN136	136
#define PIN136IN	(*((unsigned long int *) 0x42440220))
#define PIN136OUT	(*((unsigned long int *) 0x424402A0))

// PI9 is PIN137
#define PIN137	137
#define PIN137IN	(*((unsigned long int *) 0x42440224))
#define PIN137OUT	(*((unsigned long int *) 0x424402A4))

// PI10 is PIN138
#define PIN138	138
#define PIN138IN	(*((unsigned long int *) 0x42440228))
#define PIN138OUT	(*((unsigned long int *) 0x424402A8))

// PI11 is PIN139
#define PIN139	139
#define PIN139IN	(*((unsigned long int *) 0x4244022C))
#define PIN139OUT	(*((unsigned long int *) 0x424402AC))

// PI12 is PIN140
#define PIN140	140
#define PIN140IN	(*((unsigned long int *) 0x42440230))
#define PIN140OUT	(*((unsigned long int *) 0x424402B0))

// PI13 is PIN141
#define PIN141	141
#define PIN141IN	(*((unsigned long int *) 0x42440234))
#define PIN141OUT	(*((unsigned long int *) 0x424402B4))

// PI14 is PIN142
#define PIN142	142
#define PIN142IN	(*((unsigned long int *) 0x42440238))
#define PIN142OUT	(*((unsigned long int *) 0x424402B8))

// PI15 is PIN143
#define PIN143	143
#define PIN143IN	(*((unsigned long int *) 0x4244023C))
#define PIN143OUT	(*((unsigned long int *) 0x424402BC))


#endif
