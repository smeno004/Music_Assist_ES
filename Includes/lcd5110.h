#ifndef __LCD_5110_H__
#define __LCD_5110_H__

//Adapted from http://www.avrfreaks.net/forum/atmega-32-nokia-lcd-5110-problem
//        and the Adafruit GFX Library for Arduino

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>
//#include <avr/pgmspace.h>

#include <lcd5110_bitmaps.h>

//-----Config Stuff-----
#define DIN PINC3		//Data IN
#define CLK PINC2		//Clock
#define CE  PINC5		//Chip Enable
#define RST PINC6		//Reset
#define DC  PINC4		//Data/Command

#define LCD_PORT PORTC
#define LCD_PORT_DD DDRC
//----End Config Stuff----

//-----Variables-----
#define Black 1
#define White 0
#define Filled_bool 1
#define NotFilled_bool 0
//-----End Variables-----

//-----Function Declarations-----
void LCD_5110_SendCmd(unsigned long CommandOut);
void LCD_5110_SendData(unsigned long DataOut);
void LCD_5110_init(void);
void LCD_5110_update(void);
void LCD_5110_clear(void);
void LCD_5110_xy_goto(unsigned char x, unsigned char y);
void LCD_5110_SetPixel(unsigned char pixel_X, unsigned char pixel_Y, char color);
//void LCD_5110_DrawRect(short x1, short y1, short x2, short y2, char color, char fill);
//void LCD_5110_DrawLine(short x1, short y1, short x2, short y2, char color);
void LCD_5110_DrawCirc(short centX, short centY, short rad, char color, char fill);
//-----End Function Declarations-----

//-----Function Definitions-----

//-----Send Command to LCD 5110-----
void LCD_5110_SendCmd(unsigned long sentCmd) {
	//Set Data/Character High
	LCD_PORT |= (1<<DC); 

	//Set Chip/Enable High, then set it to Low
	LCD_PORT |= (1<<CE);
	LCD_PORT &= (~1<<CE);

	//Set Data/Character Low
	LCD_PORT &= (~1<<DC);

	//Pulsing Data IN and Clock signals
	unsigned char i = 0;
	for(i = 0; i < 8; i++) {
		if ((sentCmd >> (7 - i)) & 0x01) {
			LCD_PORT |= (1<<DIN);
		}
		else {
			LCD_PORT &= (~1<<DIN);
		}

		LCD_PORT |= (1<<CLK);
		LCD_PORT &= (~1<<CLK);
	}
	
	LCD_PORT |= (1<<CE); //Set Chip/Enable High
}
//-----END----Send Command to LCD 5110-----

//-----Send Data to LCD 5110-----
void LCD_5110_SendData(unsigned long sentData) {
	//Set Data/Character High
	LCD_PORT |= (1<<DC);

	//Set Chip/Enable High, then set it to Low
	LCD_PORT |= (1<<CE);
	LCD_PORT &= (~(1<<CE));

	//Pulsing Data IN and Clock signals
	unsigned char i = 0;
	for(i = 0; i < 8; i++) {
		if ((sentData >> (7 - i)) & 0x01) {
			LCD_PORT |= (1<<DIN);
		}
		else {
			LCD_PORT &= (~(1<<DIN));
		}

		LCD_PORT |= (1<<CLK);
		LCD_PORT &= (~(1<<CLK));
	}
	
	LCD_PORT |= (1<<CE); //Set Chip/Enable High
}
//-----END----Send Data to LCD 5110-----

//-----Initialize LCD 5110-----
void LCD_5110_init(void) {
	//Set pins to output
	LCD_PORT_DD |= (1<<DIN) | (1<<CLK) | (1<<CE) | (1<<DC) | (1<<RST);

	//Set Reset to high
	LCD_PORT |= (1<<RST);
	//Set Clock to high
	LCD_PORT |= (1<<CLK);
	
	//Pulse Reset after some delay
	_delay_ms(15);
	LCD_PORT |= (~1<<RST);
	_delay_ms(64); 
	LCD_PORT |= (1<<RST);

	//Set Chip Enable pin to Low
	LCD_PORT |= (~1<<CE);

	//set LCD control in additional command set (H=1)
	LCD_5110_SendCmd(0x21);		
	//set LCD VOP control (contrast)
	LCD_5110_SendCmd(0xBE);		
	//set LCD Temp coefficient
	LCD_5110_SendCmd(0x06);		
	//set LCD bias mode
	LCD_5110_SendCmd(0x13);		
	//set LCD control in basic mode set (H=0)
	LCD_5110_SendCmd(0x20);		
	//set display configuration control in normal mode
	LCD_5110_SendCmd(0x0C);		
}
//-----END----Initialize LCD 5110-----

//-----Update LCD 5110-----
void LCD_5110_update(void) {
	short i = 0;
	for (i = 0; i < 504; i++) {
		LCD_5110_SendData(LCDBuffer[i]);
	}
}
//-----END----Update LCD 5110-----

//-----Clear LCD 5110-----
void LCD_5110_clear(void) {
	short i = 0;
	short j = 0;

	for(i = 0; i < 8; i++) {
		for (j = 0; j < 90; j++) {
			LCD_5110_SendData(0x00);
		}
	}

	LCD_5110_xy_goto(0,0);

	for (i = 0; i < 504; i++) {
		LCDBuffer[i] = 0x00;
		LCD_5110_SendData(LCDBuffer[i]);
	}
}
//-----END----Clear LCD 5110-----

//-----Go to (x,y) Pixel-----
void LCD_5110_xy_goto(unsigned char x, unsigned char y) {
	LCD_5110_SendCmd(0x80 | x); //Set column position
	LCD_5110_SendCmd(0x40 | y); //Set Row position
}
//-----END----Go to (x,y) Pixel-----

//-----Set Pixel-----
void LCD_5110_SetPixel(unsigned char pixel_X, unsigned char pixel_Y, char color) {
	//Determine affected bit
	short yBit = (pixel_Y % 8);
	//Determine affected Byte
	short yByte = ((pixel_Y / 8) * 84) + pixel_X;

	if (color == Black) {
		LCDBuffer[yByte] |= (1 << yBit);
	}
	else {
		LCDBuffer[yByte] &= ~(1 << yBit);
	}
}
//-----END----Set Pixel-----

//-----Draws Circles-----
void LCD_5110_DrawCirc(short x1, short y1, short radius, char bw, char fill) {
	short i = 0;
	for (i = 0; i < 504; i++) {
		LCDBuffer[i] = 0x00;
		LCD_5110_SendData(LCDBuffer[i]);
	}

	if (fill == NotFilled_bool) {
		int x = 0;
		int y = radius;
		int p = 3 - (2 * radius);

		LCD_5110_SetPixel(x1+x,y1-y,bw);

		for (x=0;x<=y;x++){
			if (p<0) {
				y=y;
				p = (p+(4*x)+6);
			}
			else {
				y = y-1;
				p = p + ((4*(x-y)+10));
			}

			LCD_5110_SetPixel(x1 + x,y1+y,bw);
			LCD_5110_SetPixel(x1 - x,y1+y,bw);
			LCD_5110_SetPixel(x1 + x,y1-y,bw);
			LCD_5110_SetPixel(x1 - x,y1-y,bw);
			LCD_5110_SetPixel(x1 + y,y1+x,bw);
			LCD_5110_SetPixel(x1 - y,y1+x,bw);
			LCD_5110_SetPixel(x1 + y,y1-x,bw);
			LCD_5110_SetPixel(x1 - y,y1-x,bw);
		}
	}
	else {
		unsigned char r = 1;
		for (r=1;r<=radius;r++) {
			int x = 0;
			int y = r;
			int p = 3 - (2 * r);

			LCD_5110_SetPixel(x1+x,y1-y,bw);

			for (x=0;x<=y;x++){
				if (p<0) {
					y=y;
					p = (p+(4*x)+6);
				}
				else {
					y = y-1;
					p = p + ((4*(x-y)+10));
				}

				LCD_5110_SetPixel(x1 + x,y1+y,bw);
				LCD_5110_SetPixel(x1 - x,y1+y,bw);
				LCD_5110_SetPixel(x1 + x,y1-y,bw);
				LCD_5110_SetPixel(x1 - x,y1-y,bw);
				LCD_5110_SetPixel(x1 + y,y1+x,bw);
				LCD_5110_SetPixel(x1 - y,y1+x,bw);
				LCD_5110_SetPixel(x1 + y,y1-x,bw);
				LCD_5110_SetPixel(x1 - y,y1-x,bw);
			}
		}
	}
}
//-----END----Draws Circles-----

#endif //__LCD_5110_H__
