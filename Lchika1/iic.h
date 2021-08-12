/*
 * iic.h
 *
 * Created: 2020/05/14 16:59:08
 *  Author: kuras
 */ 


#ifndef IIC_H_
#define IIC_H_

// I2C�ɂ�PortC��4,5�ԃs�����g�p
#define I2C_DDDR	DDRC
#define I2C_DPIN	PINC
#define I2C_DPORT	PORTC
#define I2C_SCL		PINC5
#define I2C_SDA		PINC4

// AVR�����̃v���A�b�v���g�p����ꍇ�͈ȉ��BLOW���͏o�́ELOW�ݒ�BHI���͓��́EHI�ݒ�(Pull-up)�B
#define SCL_LOW		I2C_DDDR |= (1<<I2C_SCL); I2C_DPORT &= ~(1<<I2C_SCL);
#define SCL_HIGH	I2C_DDDR &= ~(1<<I2C_SCL); I2C_DPORT |= (1<<I2C_SCL);	/* SCL = High-Z */
#define	SCL_VAL		((I2C_DPIN & (1<<I2C_SCL)) ? 1 : 0)	/* SCL input level */
#define SDA_LOW		I2C_DDDR |= (1<<I2C_SDA); I2C_DPORT &= ~(1<<I2C_SDA);
#define SDA_HIGH	I2C_DDDR &= ~(1<<I2C_SDA); I2C_DPORT |= (1<<I2C_SDA);			/* SDA = High-Z */
#define	SDA_VAL		((I2C_DPIN & (1<<I2C_SDA)) ? 1 : 0)	/* SDA input level */

// AVR�����̃v���A�b�v���g�p���Ȃ��ꍇ�͈ȉ��B����LOW�ݒ�BLOW���͏o�́BHI���͓��́B
//#define SCL_LOW		I2C_DDDR |= (1<<I2C_SCL)
//#define SCL_HIGH	I2C_DDDR &= ~(1<<I2C_SCL)	/* SCL = High-Z */
//#define	SCL_VAL		((I2C_DPIN & (1<<I2C_SCL)) ? 1 : 0)	/* SCL input level */
//#define SDA_LOW		I2C_DDDR |= (1<<I2C_SDA)
//#define SDA_HIGH	I2C_DDDR &= ~(1<<I2C_SDA)			/* SDA = High-Z */
//#define	SDA_VAL		((I2C_DPIN & (1<<I2C_SDA)) ? 1 : 0)	/* SDA input level */

void setup_iic(void);

/* Generate start condition on the IIC bus */
void iic_start (void);

/* Generate stop condition on the IIC bus */
void iic_stop (void);

/* Send a byte to the IIC bus */
unsigned char iic_send (uint8_t dat);

unsigned char iic_recv (uint8_t nack);

#endif /* IIC_H_ */
