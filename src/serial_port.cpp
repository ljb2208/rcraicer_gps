#include "../include/rcraicer_gps/serial_port.h"

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

SerialPort::SerialPort(std::string port, int baudRate): port_fd(-1), port_setting_error(""), connected(false), alive(false), dataCallback(NULL)
{
    this->port = port;    

    if (connect(port, baudRate))
    {
      alive = true;
      runThread = std::shared_ptr<std::thread>(new std::thread(std::bind(&SerialPort::run, this)));
    }
}

SerialPort::~SerialPort()
{
  if (alive)
  {
    alive = false;
    runThread->join();

  }
}

void SerialPort::run()
{
  fd_set rfds;
  struct timeval tv;
  uint8_t data[512];
  int retval;
  volatile int received;    

  if (!isConnected())
  {
    return;
  }

  while (alive)
  {
    /* Watch stdin (fd 0) to see when it has input. */
    FD_ZERO(&rfds);
    FD_SET(port_fd, &rfds);

    /* Wait up to one seconds. */
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    retval = select(port_fd+1, &rfds, NULL, NULL, &tv);

    if (retval)
    {
        if ((received = read(port_fd, &data, 512)) >= 0)
        {
          dataMutex.lock();          

          for (int i=0; i < received; i++)
          {            
            if (dataCallback != NULL)
              dataCallback(data[i]);                                                                          
          }

          dataMutex.unlock();                             
        }
    }
  }
}

void SerialPort::lock()
{
  dataMutex.lock();
}

bool SerialPort::tryLock()
{
  return dataMutex.try_lock();
}

void SerialPort::unlock()
{
  dataMutex.unlock();
}

void SerialPort::registerDataCallback(DataCallback callback)
{
  dataCallback = callback;
}

void SerialPort::clearDataCallback()
{
  dataCallback = NULL;
}

bool SerialPort::connect(std::string port, int baudRate)
{
    connected = false;
    port_fd = open(port.c_str(), O_RDWR | O_NOCTTY /*| O_NDELAY*/);

    if (port_fd == -1)
    {
        port_setting_error = "could not open port";
        return false;
    }

    struct termios port_settings;    // structure to store the port settings in
    
    memset(&port_settings, 0, sizeof(port_settings));

    //Get the current options for the port...
    if(tcgetattr (port_fd, &port_settings) != 0)
    {
        port_setting_error = "could not get options for port";
        return false;
    }

    speed_t b;
    switch(baudRate)
    {
      case 4800:
        b = B4800;
        break;
      case 9600:
        b = B9600;
        break;
      case 19200:
        b = B19200;
        break;
      case 38400:
        b = B38400;
        break;
      case 57600:
        b = B57600;
        break;
      case 115200:
        b = B115200;
        break;
      case 230400:
        b = B230400;
        break;
      case 460800:
        b = B460800;
        break;
      default:        
        port_setting_error = "Unsupported baud";
        return false;
    }

    // set baud rates
    if(cfsetispeed(&port_settings, b) != 0 ||
       cfsetospeed(&port_settings, b) != 0)
    {     
      port_setting_error = "Could not set baud";
    }

    // set 8N1    
    port_settings.c_cflag &= ~PARENB; // no parity bit
    port_settings.c_cflag &= ~PARODD; // even parity
    
    port_settings.c_cflag &= ~CSTOPB; // only one stop bit
    port_settings.c_cflag |= CS8;     // set 8 data bits

    // disable hardware flow and software flow
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag &= ~(IXON | IXOFF | ~IXANY); //this part is important

    // enable reading and ignore control lines
    port_settings.c_cflag |= CREAD | CLOCAL;
    //set raw input mode (not canonical)
    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    port_settings.c_oflag &= ~OPOST; //  disable pre-processing of input data
                                         
    port_settings.c_cc[VMIN]  = 1; // wait for at least 1 character (read doesn't block)
    port_settings.c_cc[VTIME] = 0; // 0.5 seconds read timeout

    // apply the settings to the port
    tcflush(port_fd, TCIFLUSH);

    if(tcsetattr(port_fd, TCSANOW, &port_settings) == 0)
    {
      usleep(200000); //  wait for connection to be negotiated
                      //  found this length through experimentation      
      connected = true;
      return true;
    }
    
    port_setting_error = "Could not set serial port attributes";
    return false;   
}

int SerialPort::writePortInternal(const unsigned char* data, unsigned int length) const
{  
  if (!connected)
    return -2;

  int n;
  n=write(port_fd, data, length);

  if(n < 0)
  {  
    printf("Error %i from write: %s\n", errno, strerror(errno));
    int val = fcntl(port_fd, F_GETFL, 0);
    printf("file status = 0x%x\n", val);
    
    return -1;
  }    

  return n;
}

int SerialPort::writePort(const unsigned char* data, unsigned int length)
{
  if (isConnected())
  {
    std::unique_lock<std::mutex> lock(writeMutex);
    return writePortInternal(data, length);
  }

  return -1;
}

int SerialPort::writePortTry(const unsigned char* data, unsigned int length)
{
  if (isConnected())
  {
    std::unique_lock<std::mutex> lock(writeMutex, std::try_to_lock);

    if (lock)
      return writePortInternal(data, length);
  }

  return -1;
}