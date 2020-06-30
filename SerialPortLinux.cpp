
#include "SerialPort.h"
using namespace std;

Serial::Serial():
    portNo(""),
    connected(false)
{
	usbDeviceMap = createUsbDeviceMap();
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

    map<string, string>::iterator iter = usbDeviceMap.find(portno);
    if (iter != usbDeviceMap.end())
    {
        portno = iter->second;
    }

    fd = ::open (portno.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        //logd(TAG, "Error %d: Error opening %s: %s\n", errno,  portno.c_str(), strerror (errno));
        std::cout << "Error opening" << errno;
        return false;
    }

    switch (baudrate) {
    case BAUDRATE_4800:
        baudrate = B4800;
        break;

    case BAUDRATE_9600:
        baudrate = B9600;
        break;

    case BAUDRATE_19200:
        baudrate = B19200;
        break;

    case BAUDRATE_38400:
        baudrate = B38400;
        break;

    case BAUDRATE_57600:
        baudrate = B57600;
        break;

    case BAUDRATE_115200:
        baudrate = B115200;
        break;

    case BAUDRATE_230400:
        baudrate = B230400;
        break;

    case BAUDRATE_460800:
        baudrate = B460800;
        break;

    case BAUDRATE_921600:
        baudrate = B921600;
        break;

    default:
        baudrate = B115200;
        break;
    }

    if (set_interface_attribs (fd, baudrate, 0) == -1) {

        //logd(TAG, "Error configuring serial attributes");
        std::cout << "Error configuring serial attributes";
        return false;
    } 

    tcflush(fd,TCIOFLUSH);
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

        tcflush(fd,TCIOFLUSH);
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

    if (bytes_avail > INCOMING_DATA_MAX_LENGTH) {
        //logd(TAG, "Buffer overflow!\n");
        std::cout << "Buffer overflow!\n";
        bytes_avail = INCOMING_DATA_MAX_LENGTH;
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
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        //logd(TAG, "error %d from tcgetattr", errno);
        std::cout << "errpr tcgetattr";
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY |INLCR | IGNCR | ICRNL); // shut off xon/xoff ctrl


    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading

    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        //logd(TAG, "error %d from tcsetattr", errno);
        std::cout << "errpr tcsetattr";
        return -1;
    }
    return 0;
}

map<string, string> Serial::createUsbDeviceMap()
{
    map<string, string> map;
 
    struct udev *udev;
    struct udev_device *dev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *list, *node;
    const char *path;

    udev = udev_new();
    if (!udev) 
    {
        printf("can not create udev");
        return map;
    }

    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);

    list = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(node, list) 
    {
        path = udev_list_entry_get_name(node);
        dev = udev_device_new_from_syspath(udev, path);
        if (udev_device_get_property_value(dev, "ID_SERIAL_SHORT") &&
            udev_device_get_property_value(dev, "DEVNAME"))
        {
            string serialID(udev_device_get_property_value(dev, "ID_SERIAL_SHORT"));
            string devName(udev_device_get_property_value(dev, "DEVNAME"));
            map.insert(pair<string, string>(serialID, devName));
        }
        udev_device_unref(dev);
    }
}

