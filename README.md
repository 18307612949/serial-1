# Serial port

  Simple C++ library for work with serial ports in linux.

## Usage

### Create

```c++
try
{
    port = new serial::uart("ttyXY", B9600, CS8, 0, 0, 1000);
}
catch (serial::serial_open_error)
{
    // handle exception
};

```

### Transmit data

```c++
try
{
    port -> transmit(std::vector<uint8_t> data);
}
catch (serial::serial_write_error)
{
    // handle exception
}

```
### Receive

```c++
std::vector<uint8_t> data = port -> receive();
if (rb.size())
{
    // use data
}

```
