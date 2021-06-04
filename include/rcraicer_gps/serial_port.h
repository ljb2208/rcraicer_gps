#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H

#include <string>

#include <mutex>
#include <thread>
#include <functional>
#include <memory>
#include <vector>


class SerialPort
{
    public:
        SerialPort(std::string port, int baudRate);

        ~SerialPort();

        bool connect(std::string port, int baudRate);
        int writePort(const unsigned char* data, unsigned int length);
        int writePortTry(const unsigned char* data, unsigned int length);

        void lock();
        bool tryLock();
        void unlock();

        typedef std::function<void(const uint8_t)> DataCallback;
        void registerDataCallback(DataCallback callback);
        void clearDataCallback();
        void waitForData();        

        bool isConnected()
        {
            return connected;
        }

        std::string getErrorString()
        {
            return port_setting_error;
        }

    private:        

        void run();

        int writePortInternal(const unsigned char* data, unsigned int length) const;        
        std::string port;
        int port_fd; // file descriptor for serial port
        std::string port_setting_error;
        bool connected;
        std::vector<char> dataBuffer;

        std::shared_ptr<std::thread> runThread; ///< pointer to the read thread
        std::mutex dataMutex; ///< mutex for accessing incoming data
        std::mutex writeMutex; ///< mutex for writing serial data
        std::mutex waitMutex; ///< mutex for thread synchronization

        DataCallback dataCallback; ///< Callback triggered when new data arrives
        volatile bool alive;
};

#endif