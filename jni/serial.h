
#ifndef __SERIAL__H__
#define __SERIAL__H__

#include "uart.h"

#define     PORT_RET_OK           0x00        //  成功
#define     PORT_RET_NOTEMPTY     0x01        //  发送缓冲还有数据
#define     PORT_RET_PORTERR      0x02        //  逻辑串口号非法
#define     PORT_RET_NOTOPEN      0x03        //  串口未打开
#define     PORT_RET_BUSY         0x05        //  没有物理串口资源分配
#define     PORT_RET_MODEM_INUSE  0xf0		    //  MODEM通道正被系统占用（仅针对PortNum=PORT_MODEM）
#define     PORT_RET_PARAERR      0xfe        //  无效的通讯参数
#define     PORT_RET_TIMEOUT      0xff        //  超时

enum {
    PORT_COM1,
    PORT_EXT,
    PORT_MODEM,
    PORT_WNET,
    PORT_PINPAD,
    PORT_GPS,
    PORT_XX,
    PORT_6210LINK8210,
    PORT_6210LINK8210ACM,
    PORT_HID
};

int portOpen(int PortNum, const char *PortPara);
int portClose(int PortNum);
int portSends(int PortNum, const void *SendBuf, size_t SendLen);
int portRecv(int PortNum, uint8_t *RecvBuf, uint32_t TimeOutMs);
int portReset(int PortNum);
int portCheckRecvBuf(int PortNum);
int GetPortFd(int PortNum);

#endif
