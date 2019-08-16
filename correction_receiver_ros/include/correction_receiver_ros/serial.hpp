#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

enum SerialBaud
{
    SPABAUD_50 = B50,
    SPABAUD_110 = B110,
    SPABAUD_300 = B300,
    SPABAUD_600 = B600,
    SPABAUD_1200 = B1200,
    SPABAUD_2400 = B2400,
    SPABAUD_4800 = B4800,
    SPABAUD_9600 = B9600,
    SPABAUD_19200 = B19200,
    SPABAUD_38400 = B38400,
    SPABAUD_57600 = B57600,
    SPABAUD_115200 = B115200 
};

enum SerialDatabits
{
    SPADATABITS_5 = CS5,
    SPADATABITS_6 = CS6,
    SPADATABITS_7 = CS7,
    SPADATABITS_8 = CS8
};

enum SerialStopbits
{
    SPASTOPBITS_1 = 0,
    SPASTOPBITS_2 = CSTOPB
};

enum SerialParity
{
    SPAPARITY_NONE = 0,
    SPAPARITY_ODD = PARODD | PARENB,
    SPAPARITY_EVEN = PARENB
};

enum SerialProtocol
{
    SPAPROTOCOL_NONE = 0,
    SPAPROTOCOL_RTS_CTS = 9999,
    SPAPROTOCOL_XON_XOFF = IXOFF | IXON
};

struct serial
{
    termios Termios;
    int            Stream;
};

static SerialBaud SerialGetBaud(int baud)
{
    SerialBaud b = SPABAUD_115200;
    switch(baud)
    {
        case 50:     b = SPABAUD_50;     break;
        case 110:    b = SPABAUD_110;    break;
        case 300:    b = SPABAUD_300;    break;
        case 600:    b = SPABAUD_600;    break;
        case 1200:   b = SPABAUD_1200;   break;
        case 2400:   b = SPABAUD_2400;   break;
        case 4800:   b = SPABAUD_4800;   break;
        case 9600:   b = SPABAUD_9600;   break;
        case 19200:  b = SPABAUD_19200;  break;
        case 38400:  b = SPABAUD_38400;  break;
        case 57600:  b = SPABAUD_57600;  break;
        case 115200: b = SPABAUD_115200; break;
    }
    
    return b;
}

static SerialDatabits SerialGetDatabits(int databits)
{
    SerialDatabits d = SPADATABITS_8;
    switch(databits)
    {
        case 5: d = SPADATABITS_5; break;
        case 6: d = SPADATABITS_6; break;
        case 7: d = SPADATABITS_7; break;
        case 8: d = SPADATABITS_8; break;
    }

    return d;
}

static SerialStopbits SerialGetStopbits(int stopbits)
{
    SerialStopbits s = SPASTOPBITS_1;
    switch(stopbits)
    {
        case 1: s = SPASTOPBITS_1; break;
        case 2: s = SPASTOPBITS_2; break;
    }

    return s;
}

static SerialParity SerialGetParity(int parity)
{
    SerialParity p = SPAPARITY_NONE;
    if(parity%2 == 0 && parity != 0)
    {
        p = SPAPARITY_EVEN;
    }
    else if(parity%2 == 1)
    {
        p = SPAPARITY_ODD;
    }

    return p;
}

static SerialProtocol SerialGetProtocol(const char *buf, int *ressize)
{
    int r = 0;
    SerialProtocol Protocol = SPAPROTOCOL_NONE;
    
    /* try some possible forms for input, be as gentle as possible */
    if(!strncasecmp("xonxoff",buf,7)){r = 7; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(!strncasecmp("xon_xoff",buf,8)){r = 8; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(!strncasecmp("xon-xoff",buf,8)){r = 8; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(!strncasecmp("xon xoff",buf,8)){r = 8; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(!strncasecmp("xoff",buf,4)){r = 4; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(!strncasecmp("xon",buf,3)){r = 3; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(*buf == 'x' || *buf == 'X'){r = 1; Protocol=SPAPROTOCOL_XON_XOFF;}
    else if(!strncasecmp("rtscts",buf,6)){r = 6; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(!strncasecmp("rts_cts",buf,7)){r = 7; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(!strncasecmp("rts-cts",buf,7)){r = 7; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(!strncasecmp("rts cts",buf,7)){r = 7; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(!strncasecmp("rts",buf,3)){r = 3; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(!strncasecmp("cts",buf,3)){r = 3; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(*buf == 'r' || *buf == 'R' || *buf == 'c' || *buf == 'C')
    {r = 1; Protocol=SPAPROTOCOL_RTS_CTS;}
    else if(!strncasecmp("none",buf,4)){r = 4; Protocol=SPAPROTOCOL_NONE;}
    else if(!strncasecmp("no",buf,2)){r = 2; Protocol=SPAPROTOCOL_NONE;}
    else if(*buf == 'n' || *buf == 'N'){r = 1; Protocol=SPAPROTOCOL_NONE;}
    if(ressize) *ressize = r;
    
    return Protocol;
}

static void SerialFree(serial *sn)
{
    if(sn->Stream)
    {
        /* reset old settings */
        tcsetattr(sn->Stream, TCSANOW, &sn->Termios);
        close(sn->Stream);
        sn->Stream = 0;
    }
}

static int SerialInitialize(serial *sn, const char *Device,
                            SerialBaud Baud, SerialStopbits StopBits,
                            SerialProtocol Protocol, SerialParity Parity,
                            SerialDatabits DataBits, int dowrite)
{
    termios newtermios;

    if((sn->Stream = open(Device, O_RDWR | O_NOCTTY | O_NONBLOCK)) <= 0)
        return -1;
    
    tcgetattr(sn->Stream, &sn->Termios);
    memset(&newtermios, 0, sizeof(termios));
    newtermios.c_cflag = Baud | StopBits | Parity | DataBits | CLOCAL | CREAD;
    if(Protocol == SPAPROTOCOL_RTS_CTS)
        newtermios.c_cflag |= CRTSCTS;
    else
        newtermios.c_cflag |= Protocol;
    newtermios.c_cc[VMIN] = 1;
    tcflush(sn->Stream, TCIOFLUSH);
    tcsetattr(sn->Stream, TCSANOW, &newtermios);
    tcflush(sn->Stream, TCIOFLUSH);
    fcntl(sn->Stream, F_SETFL, O_NONBLOCK);
    
    return 0;
}

static int SerialInit(serial *sn, const char *Device,
                      int baud, int databits, 
                      int stopbits, int parity)
{
    return SerialInitialize(sn, Device, SerialGetBaud(baud),
                            SerialGetStopbits(stopbits), SPAPROTOCOL_NONE,
                            SerialGetParity(parity), SerialGetDatabits(databits), 1);
}

static int SerialRead(serial *sn, char *buffer, size_t size)
{
    int j = read(sn->Stream, buffer, size);
    if(j < 0)
    {
        if(errno == EAGAIN)
            return 0;
    else
        return j;
    }
    
    return j;
}

static int SerialWrite(serial *sn, const char *buffer, size_t size)
{
    int j = write(sn->Stream, buffer, size);
    if(j < 0)
    {
        if(errno == EAGAIN)
            return 0;
    else
        return j;
    }
    
    return j;
}

#endif