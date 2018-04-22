#ifndef SERIAL_H
#define SERIAL_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <stdint.h>


namespace serial
{

class uart
{
private:
    int port_fd;
    std::string name;
    struct termios options;
public:
    uart(const char* aname, int abaud, int abitsize, int aparity, int astop, int atimeout_01s);
    ~uart();
    std::vector<uint8_t> receive();
    void transmit(std::vector<uint8_t> adata);
};


class serial_open_error : public std::exception
{
    std::string name;
    std::string error_message;
public:
    serial_open_error(const char* aname)    {
                                            name = std::string(aname);
                                            error_message = "can not open port " + name +'\n';
                                            };
    const char * what() const noexcept {
                                        return (error_message).c_str();
                                        };
};

class serial_write_error : public std::exception
{
    const char *name;
    std::string error_message;
    int count;
    int port_fd;
public:
    serial_write_error(const char* aname, int acount, int aport_fd) {
                                                                    name = aname; count = acount;
                                                                    port_fd = aport_fd;
                                                                    error_message = "can not write " + std::to_string(count) +
                                                                                    " bytes to " + name + '\n';
                                                                    };
    const char * what() const noexcept {return error_message.c_str();};
};

}
#endif //SERIAL_H
