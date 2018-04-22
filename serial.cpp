#include "serial.h"

namespace serial
{

uart::uart(const char* aname, int abaud, int abitsize, int aparity, int astop, int atimeout_01s)
{
    name = std::string("/dev/") + aname;
    port_fd = open(name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (port_fd == -1)
        throw serial_open_error(name.c_str());

    fcntl(port_fd, F_SETFL, 0);//fcntl(port_fd, F_SETFL, FNDELAY);

    tcgetattr(port_fd, &options);    // read current options

    options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | PARODD); // Mask character size, parity and stop bits
    options.c_cflag |= abitsize;    // Select data bitsize
    if (aparity == PARODD)                      // Select parity
        options.c_cflag |= PARENB | PARODD;     // ODD parity
    else
        options.c_cflag |= aparity;             // other parity

    options.c_cflag |= (CLOCAL | CREAD);    // to ensure that program does not become the 'owner' of the port

    options.c_cflag &= ~CRTSCTS;    // disable hardware flow control

    cfsetispeed(&options, abaud);
    cfsetospeed(&options, abaud);

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input no echo

    options.c_iflag &= ~(INPCK | ISTRIP | IXON | IXOFF | IXANY); // disable parity check and software flow control

    if (options.c_cflag & PARENB)
        options.c_iflag |= (INPCK | ISTRIP); // enable parity check

    options.c_oflag &= ~OPOST;  // raw output
    options.c_cc[VTIME]    = atimeout_01s;   // timeout
    options.c_cc[VMIN]     = 0;

    tcflush(port_fd, TCIFLUSH);

    tcsetattr(port_fd,TCSANOW,&options);

    return;
}



uart::~uart()
{
    close(port_fd);
}

void uart::transmit(std::vector<uint8_t> adata)
{
    if (write(port_fd, adata.data(), adata.size()) < adata.size())
         throw serial_write_error(name.c_str(), adata.size(), port_fd);
    return;
}

std::vector<uint8_t> uart::receive()
{
    uint8_t buffer;
    std::vector<uint8_t> data;
    while(read(port_fd, &buffer, 1) > 0)
        data.push_back(buffer);
    return data;
}


}
