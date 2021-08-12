/*
 * Lchika1.c
 *
 * Created: 2020/05/11 14:28:55
 * Author : kuras
 */ 
#define F_CPU   1000000UL

#include <avr/io.h>
//#include "avr/iom328p.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "iic.h"
#include "eeprom.h"

#define BAUD_PRESCALE_9600 12 // U2X=1 1MHz 9600B

int write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	iic_start();
	int ack = iic_send(dev_addr<<1);
	if( ack != 0 )
	{
		return 10;
	}

	ack = iic_send(reg_addr);
	if( ack != 0 )
	{
		return 11;
	}

	ack = iic_send(data);
	if( ack != 0 )
	{
		return 12;
	}

	iic_stop();
	return 0;
}

void lcd_cmd( uint8_t cmd )
{
	write_reg( 0x3e, 0, cmd );
}

void lcd_data( uint8_t d )
{
	write_reg( 0x3e, 0x40, d );
}

void lcd_puts( const char *mes )
{
	while(*mes)
	lcd_data(*mes++);
}

#define contrast 40

void lcd_init()
{
	// reset
	_delay_ms(500);
	PORTC &= ~2; // reset = low
	_delay_ms(1);
	PORTC |= 2;  // reset = hi
	_delay_ms(10);

	// LCD initialize
	_delay_ms(40);
	lcd_cmd(0x38); // function set
	lcd_cmd(0x39); // function set
	lcd_cmd(0x14); // interval osc
	lcd_cmd(0x70 | (contrast & 15)); // contrast low
	lcd_cmd(0x5c | (contrast >> 4 & 3)); // contrast high / icon / power
	lcd_cmd(0x6c); // follower control
	_delay_ms(300);

	lcd_cmd(0x38); // function set (1line). if set to 2lines->0x38, 1line->0x3c
	lcd_cmd(0x0c); // display on
	lcd_cmd(0x01); // clear display
	_delay_ms(2);
}

void lcd_move(uint8_t pos)
{
	lcd_cmd(0x80 | pos);
}

void setup()
{
	lcd_init();
	lcd_puts("Hello, Arduino!");
	lcd_move(0x40);
	lcd_puts("i2c LCD module");
}

volatile uint8_t bRecieve;
char *waiting_message;

void setup_USART(unsigned int prescale){
	// Set baud rate
	UBRR0L = prescale;// Load lower 8-bits into the low byte of the UBRR register
	UBRR0H = (prescale >> 8);

	// 倍速ビットON
	UCSR0A = 2;
	// Enable receiver and transmitter and receive complete interrupt
	UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
	UCSR0C = 0b00000110;
}

void USART_SendByte(uint8_t u8Data){

	// Wait until last byte has been transmitted
	while((UCSR0A &(1<<UDRE0)) == 0);

	// Transmit data
	UDR0 = u8Data;
	
	// clear tx complete flag
	UCSR0A |= (1<<TXC0);
}

void USART_SendStr(const char * str)
{
	uint8_t i = 0;
	while( str[i] != '\0' )
	{
		USART_SendByte(str[i++]);
	}
	
	// wait for tx complete
	while( (UCSR0A & (1<<TXC0)) == 0 );
}

#define RCV_SIZE 100
volatile char RCV_BUF[RCV_SIZE];
uint8_t RCV_PTR = 0;

void check_command()
{
	if( strstr((char*)RCV_BUF, waiting_message) != NULL )
	{
		bRecieve = 1;
	}
}

ISR (USART_RX_vect)
{
	char value = UDR0;             //read UART register into value
	RCV_BUF[RCV_PTR] = value;
	if( RCV_PTR < RCV_SIZE-1 )
	{
		RCV_PTR++;
	}
}

uint8_t check_rcv(uint8_t cal_num)
{
	//_delay_ms(2000);
	memset((char*)RCV_BUF,0,RCV_SIZE);
	RCV_PTR = 0;
	sei();

	_delay_ms(500);
	PORTB |= 1; // WAKE_SW = 1 operation mode
	_delay_ms(500);


	cli();
	OSCCAL = cal_num;
	char mes[100];
	uint8_t success;
	if( strstr((char*)RCV_BUF, "CMD") != NULL )
	{
		success = 1;
		PORTD |= 1 << 4;
		_delay_ms(500);
	}
	else
	{
		success = 0;
		PORTD |= 1 << 4;
		_delay_ms(50);
	}
	lcd_move(0);
	lcd_puts(mes);
		
	//_delay_ms(1000);
	PORTB &= ~1; // WAKE_SW = 1 operation mode
	PORTD &= ~(1 << 4);
	return success;
}

int main(void)
{
	CLKPR = 0x80;// クロック分周比変更許可ビットON
	CLKPR = 0x03;// 分周比1/8 : クロックは1MHz
	
	uint8_t cal_num = OSCCAL;

	// B0 : output : WAKE_SW
	DDRB |= 1;
	PORTB &= ~1; // WAKE_SW = 0 sleep mode
	// B1 : output : CMD_MLDP
	DDRB |= 2;
	PORTB &= ~2; // CMD_MLDP = 0 cmd mode
	// B2 : input connect
	DDRB &= ~4;

	PORTD |= 1; // RXD端子プルアップ

	// D4 : output : LED
	DDRD = 1 << 4;
	PORTD |= 1 << 4;

	// C1 : output : lcd module reset
	DDRC |= 2;
	PORTC |= 2;
	
	// C2 : output : lcd module back light
	DDRC |= 4;
	PORTC |= 4;
	
	setup_iic();

	setup();

	PORTD &= ~(1 << 4);
	//uint8_t rom_val = read_eeprom(0);
	setup_USART(BAUD_PRESCALE_9600);
	//OSCCAL = rom_val;
	OSCCAL = 175;
	_delay_ms(1000);
	int s0 = check_rcv(cal_num);

	int sta1 = -1;
	int end1 = 0;	
	for(int n=0; n<128; n++)
	{
		OSCCAL = n;
		int s = check_rcv(cal_num);
		if(s)
		{
			if(sta1==-1) sta1 = n;
			else end1 = n;
		}
	}
	int sta2 = -1;
	int end2 = 0;
	for(int n=128; n<256; n++)
	{
		OSCCAL = n;
		int s = check_rcv(cal_num);
		if(s)
		{
			if(sta2==-1) sta2 = n;
			else end2 = n;
		}
	}
	char mes[100];
	sprintf(mes, "\x0d\x0a%d %s\x0d\x0a",cal_num, s0?"OK":"NG");
	USART_SendStr(mes);
	int cal_val;
	if(end1 - sta1 <= end2 - sta2)
	{
		cal_val = (sta2 + end2)/2;
	}
	else
	{
		cal_val = (sta1 + end1)/2;
	}
	cli();
	sprintf(mes, "\x0d\x0a%d-%d %d-%d Cal=%d(%x)\x0d\x0a",sta1,end1,sta2,end2, cal_val, cal_val);
	USART_SendStr(mes);
	write_eeprom(0,(unsigned char)cal_val);
	unsigned char rval = read_eeprom(0);
	if( rval == cal_val )
	{
		USART_SendStr("EEPROM Write success!!\x0d\x0a");
	}
	else
	{
		USART_SendStr("EEPROM Write Fail!!\x0d\x0a");
	}
}

