/*
 * eeprom.c
 *
 * Created: 2021/05/14 11:26:01
 *  Author: kuras
 */ 
#include <avr/io.h>
#include "eeprom.h"

unsigned char read_eeprom(uint16_t adrs)
{
	while((EECR & (1<<EEPE)));
	EEARH = adrs >> 8;
	EEARL = adrs & 0xff;
	EECR = (1<<EERE);
	return EEDR;
}

void write_eeprom(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR & (1<<EEPE)); /* �ȑO��EEPROM��۸���ݸފ����܂őҋ@ */
	EECR = (0<<EEPM1)|(0<<EEPM0); /* �Ή���۸���ݸގ�ʐݒ� */
	EEAR = uiAddress; /* EEPROM���ڽ�ݒ� */
	EEDR = ucData; /* EEPROM�������ݒl��ݒ� */
	EECR |= (1<<EEMPE); /* EEPROM����۸��ы��� */
	EECR |= (1<<EEPE); /* EEPROM��۸���ݸފJ�n */
}
