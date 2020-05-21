
#include "SerialPort.h"
using namespace std;

Serial::Serial():
    portNo(""),
    connected(false)
{
}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    close();
}

bool Serial::open(std::string portno, int baudrate)
{
    if (isConnected())
        close();

    fd = ::open (portno.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        //logd(TAG, "Error %d: Error opening %s: %s\n", errno,  portno.c_str(), strerror (errno));
        std::cout << "Error opening " << errno << std::endl;
        return false;
    }

    if (set_interface_attribs (fd, baudrate, 0) == -1) {

        //logd(TAG, "Error configuring serial attributes");
        std::cout << "Error configuring serial attributes" << std::endl;
        return false;
    } 

    portNo = portno;
    this->connected = true;
    return true;
}

bool Serial::close()
{
    if (this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        ::close(fd);
    }

    return true;
}

int Serial::readData(unsigned char *buffer, unsigned int nbChar)
{
    if (!isConnected())
        return 0;
   
    int bytes_avail;
    ioctl(fd, FIONREAD, &bytes_avail);

    if (bytes_avail > 1024) {
        //logd(TAG, "Buffer overflow!\n");
        std::cout << "Buffer overflow!\n";
        bytes_avail = 1024;
    } else if (bytes_avail > nbChar)
        bytes_avail = nbChar; 

    int n = ::read(fd, buffer, bytes_avail);//sizeof(rxBuffer));  // read up to 100 characters if ready to read
    
    return n;
}

void Serial::setMode(int mode)
{
    usbMode = mode;
}

int Serial::getMode(void)
{
    return usbMode;
}

bool Serial::writeData(unsigned char *buffer, unsigned int nbChar)
{
    if (!isConnected())
    {
        std::cout << "dongle not connected\n";
        return false;
    }
    int ret;
    ret = ::write(fd, buffer, nbChar);           // send 7 character greeting
    return true;
}

bool Serial::isConnected()
{
    //Simply return the connection status
    return this->connected;
}


int Serial::set_interface_attribs (int fd, int speed, int parity)
{
    struct termios2 tty;


    if (ioctl(fd, TCGETS2, &tty) < 0)
    {
        return -1;
    }
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = speed;
    tty.c_ospeed = speed;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY |INLCR | IGNCR | ICRNL); //enable xon
    tty.c_iflag |=IXOFF;

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading

    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (ioctl(fd, TCSETS2, &tty) < 0)
    {
        return -1;
    }
    
    return 0;
}