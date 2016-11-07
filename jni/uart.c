/******************************************************************************
 * uart.c
 * 
 * Copyright (c) 2010 by Li.Hua <lihua_338@163.com>.
 * 
 * DESCRIPTION: - 
 * 
 * Modification history
 * ----------------------------------------------------------------------------
 * Date         Version  Author       History
 * ----------------------------------------------------------------------------
 * 2010-07-05   1.0.0.0  Li.Hua       written
 * 2012-01-04   1.0.0.1  Li.Hua       增加串口设置时候用CLOCAL
 ******************************************************************************/

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>


#include "uart.h"

#if defined(MEMWATCH)
#include "memwatch.h"
#endif


struct speed_map {
	unsigned short speed;
	unsigned short value;
};

static const struct speed_map speeds[] = {
	{B0, 0},
	{B50, 50},
	{B75, 75},
	{B110, 110},
	{B134, 134},
	{B150, 150},
	{B200, 200},
	{B300, 300},
	{B600, 600},
	{B1200, 1200},
	{B1800, 1800},
	{B2400, 2400},
	{B4800, 4800},
	{B9600, 9600},
#ifdef	B19200
	{B19200, 19200},
#elif defined(EXTA)
	{EXTA, 19200},
#endif
#ifdef	B38400
	{B38400, 38400/256 + 0x8000U},
#elif defined(EXTB)
	{EXTB, 38400/256 + 0x8000U},
#endif
#ifdef B57600
	{B57600, 57600/256 + 0x8000U},
#endif
#ifdef B115200
	{B115200, 115200/256 + 0x8000U},
#endif
#ifdef B230400
	{B230400, 230400/256 + 0x8000U},
#endif
#ifdef B460800
	{B460800, 460800/256 + 0x8000U},
#endif
#ifdef B921600
	{B921600, 921600/256 + 0x8000U},
#endif
};

#define NUM_SPEEDS 		(sizeof(speeds)/sizeof(speeds[0]))

int cfsetspeed (struct termios *termios_p, speed_t speed) 
{
    size_t cnt; 
    
    for (cnt = 0; cnt < sizeof (speeds) / sizeof (speeds[0]); ++cnt) 
    if (speed == speeds[cnt].speed) 
    {
	    cfsetispeed (termios_p, speed); 
	    cfsetospeed (termios_p, speed); 
	    return 0; 
    }
    else if (speed == speeds[cnt].value) 
    { 
	    cfsetispeed (termios_p, speeds[cnt].speed); 
	    cfsetospeed (termios_p, speeds[cnt].speed); 
	    return 0; 
    } 
    
    errno = EINVAL; 
    return -1; 
} 

/******************************************************************************
 * Function:		tty_baud_to_value
 *
 * DESCRIPTION:	将Bxxxx定义转换成数值xxxx
 *
 * Input:		波特率定义，例如B115200, B9600, ...
 *
 * Output:		无
 *
 * Returns:		0 : 未定义的波特率
 *             	其它 : 波特率数值
 *
 * Modification history
 * ----------------------------------------------------------------------------
 *	Li.Hua	2008-09-05 12:07:20 ,  Written
 ******************************************************************************/
unsigned int tty_baud_to_value(speed_t speed)
{
	int i = 0;

	do {
		if (speed == speeds[i].speed) {
			if (speeds[i].value & 0x8000U) {
				return ((unsigned long) (speeds[i].value) & 0x7fffU) * 256;
			}
			return speeds[i].value;
		}
	} while (++i < NUM_SPEEDS);

	return 0;
}

/******************************************************************************
 * Function:		tty_value_to_baud
 *
 * DESCRIPTION:	将波特率数值转换成波特率定义
 *
 * Input:		value : 波特率数值，例如 115200, 9600, ...
 *
 * Output:		无
 *
 * Returns:		0xFFFFFFFF : 不支持改波特率
 *             	其它 : 波特率定义
 *
 * Modification history
 * ----------------------------------------------------------------------------
 *	Li.Hua	2008-09-05 12:09:58 ,  Written
 ******************************************************************************/
speed_t tty_value_to_baud(unsigned int value)
{
	int i = 0;

	do {
		if (value == tty_baud_to_value(speeds[i].speed)) {
			return speeds[i].speed;
		}
	} while (++i < NUM_SPEEDS);

	return (speed_t) ~0;
}

/******************************************************************************
 * Function:		set_tty_speed
 *
 * DESCRIPTION:	设置串口通信速率
 *
 * Input:		fd : 打开串口的文件句柄
 *				speed : 串口速度，见name_arr中列举的数值
 *
 * Output:		无
 *
 * Returns:		0    : 设置成功
 *				其它 : 设置失败
 *
 * Modification history
 * ----------------------------------------------------------------------------
 *	Li.Hua	2008-09-04 16:12:30 ,  Written
 ******************************************************************************/
int set_tty_speed(int fd, int speed)
{
	int	status;
	speed_t value;
	struct termios   Opt;

	tcgetattr(fd, &Opt);
	value = tty_value_to_baud(speed);
	if (value != ~0) {
		tcflush(fd, TCIOFLUSH);
		cfsetispeed(&Opt, value);
		cfsetospeed(&Opt, value);
		status = tcsetattr(fd, TCSANOW, &Opt);
		if (status != 0) {
			return -1;
		}
		tcflush(fd,TCIOFLUSH);
		return 0;
	}

	return -1;
}

/******************************************************************************
 * Function:	set_tty_property
 *
 * DESCRIPTION:	设置串口属性
 *
 * Input:		fd : 打开串口句柄
 *             	databits : 数据位(5,6,7,8)
 *				parity   : 校验 (N,E,O,S)
 *				stopbits : 停止位 (1,2)
 *
 * Output:		无
 *
 * Returns:		0    : 成功
 *				其它 : 错误返回码
 *
 * Modification history
 * ----------------------------------------------------------------------------
 *	Li.Hua	2008-09-04 16:15:25 ,  Written
 ******************************************************************************/
int tty_property_config(int fd, int baudrate, int databits, int parity, int stopbits, int flow)
{
	struct termios options;
	speed_t value;

	if  ( tcgetattr(fd, &options)  !=  0)
		return -EFAULT;

	value = tty_value_to_baud(baudrate);
	if (value != (speed_t)~0)
		cfsetspeed(&options, value);
	else
		return -EINVAL;

	options.c_cflag &= ~CSIZE;
	switch (databits) {
		case 5:
			options.c_cflag |= CS5;
			break;
		case 6:
			options.c_cflag |= CS6;
			break;
		case 7:
			options.c_cflag |= CS7;
			break;
		case 8:
			options.c_cflag |= CS8;
			break;
		default:
			fprintf(stderr, "Unsupported data size\n");
			return -EINVAL;
	}

	switch (parity) {
		case 'n':
		case 'N':
			options.c_cflag &= ~PARENB;   /* Clear parity enable */
			options.c_iflag &= ~INPCK;     /* Enable parity checking */
			break;
		case 'o':
		case 'O':
			options.c_cflag |= (PARODD | PARENB); 
			options.c_iflag |= INPCK;             /* Disnable parity checking */
			break;
		case 'e':
		case 'E':
			options.c_cflag |= PARENB;     /* Enable parity */
			options.c_cflag &= ~PARODD;  
			options.c_iflag |= INPCK;       /* Disnable parity checking */
			break;
		case 'S':
		case 's':  /*as no parity*/
			options.c_cflag &= ~PARENB;
			options.c_cflag &= ~CSTOPB;
			options.c_iflag |= INPCK;
			break;
		default:
			fprintf(stderr, "Unsupported parity\n");
			return -EINVAL;
	}

	switch (stopbits) {
		case 1:
			options.c_cflag &= ~CSTOPB;
			break;
		case 2:
			options.c_cflag |= CSTOPB;
			break;
		default:
			fprintf(stderr, "Unsupported stop bits\n");
			return -EINVAL;
	}

	switch (flow) {
		case 'h':
		case 'H':
			options.c_cflag |= CRTSCTS;
			options.c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		case 's':
		case 'S':
			options.c_cflag &= ~CRTSCTS;
			options.c_iflag |= IXON | IXOFF | IXANY;
			break;
		case 'n':
		case 'N':
			options.c_cflag &= ~CRTSCTS;
			options.c_iflag &= ~(IXON | IXOFF | IXANY);
			break;
		default:
			fprintf(stderr, "Unsupported flow\n");
			return -EINVAL;
	}

	options.c_lflag &= ~(ICANON);
	options.c_lflag&= ~(ISIG | ECHO | ECHOK | ECHOCTL | ECHONL | ECHOPRT);
	options.c_oflag &= ~(ONLCR | OCRNL | ONOCR | ONLRET | NOFLSH | TOSTOP);
	options.c_iflag &= ~(BRKINT | ICRNL | INLCR | IGNCR);
	options.c_iflag |= IGNBRK;
	options.c_cflag |= CREAD | CLOCAL;

	options.c_cc[VTIME] = 0; 
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */

	if (tcsetattr(fd,TCSANOW, &options) != 0) {
		perror("SetupSerial 3");
		return -EFAULT;
	}

	return 0;
}

