/******************************************************************************
 * uart.c
 *
 * Copyright (c) 2011 by Li.Hua <lihua_338@163.com>.
 *
 * DESCRIPTION: -
 *
 * Modification history
 * ----------------------------------------------------------------------------
 * Date         Version  Author       History
 * ----------------------------------------------------------------------------
 * 2011-01-20   1.0.0    Li.Hua       written
 ******************************************************************************/

#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <pthread.h>
#include <errno.h>

#include "posapi.h"
#include "jni_util.h"
#include "serial.h"

typedef struct {
    const char      *devname;
    int             portnum;
    int             fd;
    int             refcnt;
    pthread_mutex_t lock;
} Uart_t;


static Uart_t   global_uarts[] = {
    {
        .devname    = "/dev/ttyS3",
        .portnum    = PORT_COM1,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/ttyS2",
        .portnum    = PORT_EXT,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/ttyS1",
        .portnum    = PORT_MODEM,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = NULL,
        .portnum    = PORT_WNET,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/ttyS4",
        .portnum    = PORT_PINPAD,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = NULL,
        .portnum    = PORT_GPS,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/ttyGS0",
        .portnum    = PORT_XX,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/ttyUSB0",
        .portnum    = PORT_6210LINK8210,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/ttyACM0",
        .portnum    = PORT_6210LINK8210ACM,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
    {
        .devname    = "/dev/hid",
        .portnum    = PORT_HID,
        .fd         = -1,
        .refcnt     = 0,
        .lock       = PTHREAD_MUTEX_INITIALIZER,
    },
};

static Uart_t *uart_find(int PortNum)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(global_uarts); i++) {
        if (global_uarts[i].portnum == PortNum)
            return &global_uarts[i];
    }

    return NULL;
}

int check_usb_type(int index)
{
    int ret = -1, fd = -1;
    char file_name[256] = {0};
    char buf[512] = {0};

    LOGD("call in check_usb_type");

    sprintf(file_name, "/sys/bus/usb-serial/devices/ttyUSB%d/uevent", index);
    LOGD("check_usb_type: filename:%s", file_name);

    if (access(file_name, F_OK)) {
        LOGD("No such file");
        return -1;
    }

    fd = open(file_name, O_RDONLY);
    if (fd < 0) {
        LOGD("open file err");
        return -1;
    }

    ret = safety_read(fd, buf, 256);
    if (ret <= 0) {
        LOGD("safety_read err");
        ret = -1;
        goto EXIT;
    }
    LOGD("Read file Content:%s", buf);

    if (strstr(buf, "DRIVER=") == NULL) {
        LOGD("look for \"DRIVER=\" err");
        ret = -1;
        goto EXIT;
    }

    if (strstr(buf, "ftdi_sio") == NULL) {
        LOGD("look for \"DRIVER=\" err");
        ret = -1;
        goto EXIT;
    }

    ret = 0;
EXIT:
    close(fd);
    return ret;
}

static int usb_try_open(void)
{
    int i, ret = -1;
    char dvName[64];

    for (i = 0; i < 127; i++) {
        sprintf(dvName, "/dev/ttyUSB%d", i);
        if (check_usb_type(i) == 0) {
            ret = open(dvName, O_RDWR | O_NONBLOCK);
            LOGD("Open device:%s, ret=%d, errno=%d", dvName, ret, errno);
            break;
        }
    }
    return ret;
}

/* 打开指定的串口 */
int portOpen(int PortNum, const char *PortPara)
{
    Uart_t  *uart = uart_find(PortNum);
    int     retval = PORT_RET_BUSY;

    if (!uart)
        return PORT_RET_PORTERR;

    pthread_mutex_lock(&uart->lock);

    if (uart->refcnt++ == 0) {
        if (PortNum == PORT_6210LINK8210) {
            uart->fd = usb_try_open();
        } else if (PortNum == PORT_XX) {
            uart->fd  = open("/dev/ttyGS0", O_RDWR | O_NONBLOCK);
        } else {
            uart->fd = open(uart->devname, O_RDWR | O_NONBLOCK);
        }

        LOGD("PortNum=%d, uart->fd = %d, errno=%d",PortNum,uart->fd,errno);
        
        if (uart->fd >= 0) {
            int         baudrate, databits, parity, stopbits;
            const char  *str;
            char        *endptr;

            str = PortPara;
            baudrate = strtol(str, &endptr, 10);

            str = endptr + 1;
            databits = strtol(str, &endptr, 10);

            str = endptr + 1;
            parity = *str;

            str += 2;
            stopbits = strtol(str, &endptr, 10);

            if (PortNum == PORT_COM1 || PortNum == PORT_PINPAD || PortNum == PORT_EXT) {
                if (tty_property_config(uart->fd, baudrate, databits, parity, stopbits, 'n')) {
                    close(uart->fd);
                    uart->fd = -1;
                    uart->refcnt--;
                    retval = PORT_RET_PARAERR;
                } else {
                    retval = PORT_RET_OK;
                }
            } else {
                tty_property_config(uart->fd, baudrate, databits, parity, stopbits, 'n');
                retval = PORT_RET_OK;
            }
        }
    }

    pthread_mutex_unlock(&uart->lock);

    return retval;
}

/* 关闭指定的通讯口 */
int portClose(int PortNum)
{
    Uart_t  *uart = uart_find(PortNum);

    if (!uart)
        return PORT_RET_PORTERR;

    if (uart->refcnt == 0)
        return PORT_RET_NOTOPEN;

    pthread_mutex_lock(&uart->lock);
    if (--uart->refcnt == 0) {
        close(uart->fd);
        uart->fd = -1;
    }
    pthread_mutex_unlock(&uart->lock);

    return 0;
}

/* 使用指定的逻辑串口号发送若干字节的数据 */
int portSends(int PortNum, const void *SendBuf, size_t SendLen)
{
    Uart_t  *uart = uart_find(PortNum);

    if (!uart)
        return PORT_RET_PORTERR;

    if (uart->refcnt == 0)
        return PORT_RET_NOTOPEN;

    safety_full_write(uart->fd, SendBuf, SendLen);
    return 0;
}

/* 使用指定的逻辑串口号发送若干字节的数据 */
int portPrintf(int PortNum, const char *FmtBuf, ...)
{
    Uart_t  *uart = uart_find(PortNum);
    char    buffer[8 * 1024];
    va_list ap;
    int     nbytes;

    if (!uart)
        return PORT_RET_PORTERR;

    if (uart->refcnt == 0)
        return PORT_RET_NOTOPEN;


    va_start(ap, FmtBuf);
    nbytes = vsnprintf(buffer, sizeof(buffer), FmtBuf, ap);
    va_end(ap);

    return safety_full_write(uart->fd, buffer, nbytes);
}

/* 从指定的逻辑串口号, 接收一个字节的数据 */
int portRecv(int PortNum, uint8_t *RecvBuf, uint32_t TimeOutMs)
{
    Uart_t          *uart = uart_find(PortNum);
    struct timeval  t;
    fd_set          rfds;
    int             ret;

    if (!uart)
        return PORT_RET_PORTERR;

    if (uart->refcnt == 0)
        return PORT_RET_NOTOPEN;

    t.tv_sec  = TimeOutMs / 1000;
    t.tv_usec = (TimeOutMs % 1000) * 1000;

    FD_ZERO(&rfds);
    FD_SET(uart->fd, &rfds);

    ret = select(uart->fd + 1, &rfds, NULL, NULL, &t);
    if (ret < 0)
        perror("select err");

    if (ret > 0 && FD_ISSET(uart->fd, &rfds))
        return read(uart->fd, RecvBuf, 1) == 1 ? 0 : 0xff;

    return 0xff;
}

/* 复位通讯口，该函数将清除串口接收缓冲区中的所有数据 */
int portReset(int PortNum)
{
    Uart_t  *uart = uart_find(PortNum);

    if (!uart)
        return PORT_RET_PORTERR;

    if (uart->refcnt == 0)
        return PORT_RET_NOTOPEN;

    tcflush(uart->fd, TCIOFLUSH);
    return 0;
}

/* 检查指定通讯口的接收缓冲区是否已无数据待接收 */
int portCheckRecvBuf(int PortNum)
{
    Uart_t          *uart = uart_find(PortNum);
    struct timeval  t;
    fd_set          rfds;
    int             ret;

    if (!uart)
        return PORT_RET_PORTERR;

    if (uart->refcnt == 0)
        return PORT_RET_NOTOPEN;

    t.tv_sec  = 0;
    t.tv_usec = 10 * 100;

    FD_ZERO(&rfds);
    FD_SET(uart->fd, &rfds);

    ret = select(uart->fd + 1, &rfds, NULL, NULL, &t);
    if (ret < 0) {
        perror("Magcard swipe select");
        return PORT_RET_OK;
    }

    if (ret && FD_ISSET(uart->fd, &rfds))
        return PORT_RET_NOTEMPTY;

    return PORT_RET_OK;
}

int GetPortFd(int PortNum)//获取串口的句柄
{
    Uart_t  *uart = uart_find(PortNum);
    return uart->fd;
}


