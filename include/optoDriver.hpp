#ifndef OPTODRIVER_HPP
#define OPTODRIVER_HPP

#include <fcntl.h>		// File control definitions
#include <stdio.h> 		// Standard input/output definitions
#include <unistd.h> 	// UNIX standard function definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdlib.h>     // malloc, free, rand
#include <string.h>
#include <sys/ioctl.h>
#include <thread>
#include <iostream>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <chrono>



class optoDriver {
public:
    optoDriver(int speed, int filter, bool zero);
    optoDriver();
    ~optoDriver();

    // Sensor settings and data
    unsigned int streamingSlots[8];
    unsigned int stream_byte_len;
    float diff;
    std::chrono::milliseconds period;
    unsigned int speed;
    unsigned int filter;
    bool zero;
    int SerialNumber;
    char* port;
    std::vector<float> forces_vec; //publically accessible data readings

    // Configure and connect
    int openAndSetupComPort(const char* comport);
    int openFD();
    int closeComPort();
    int setConfig(unsigned int speed, unsigned int filter, bool zero);

    // Streaming options
    int startStreaming();
    int stopStreaming();
    int getStream();
    int getFD();

    // API commands
    int spinOnce();


private:
    // Multithreading
    std::mutex mu;              // thread mutex
    std::condition_variable cv; // Condition variable for passing thread locking order (prevent starvation)
    std::thread readThread;

    // Device specific options
    int BaudRate;
    int ByteSize;
    int StopBits;
    int Parity;
    int fd;                     // File handle for device
    struct termios old_options; // TERMIOS setting of device when first opened
    bool streamON;
    bool threadON;
    bool newData;
    bool hasFirst;

    //calibration conversion from count to N or Nm. These are unique for every transducer and may be changed
    //based on the datasheet received from Optoforce
    float XCalibration = 1 / 18.28;
    float YCalibration = 1 / 18.41;
    float ZCalibration = 1 / 2.12;
    float TxCalibration = 1 / 278.72;
    float TyCalibration = 1 / 259.55;
    float TzCalibration = 1 / 448.47;

    std::vector<float> calArray = {XCalibration, YCalibration, ZCalibration, TxCalibration, TyCalibration, TzCalibration};

    // Data
    std::chrono::time_point<std::chrono::system_clock> last_packet_time, last_retrieval_time;
    std::chrono::duration<float> dt;
    unsigned char * last_stream_data;

    // Stream
    int checkStream();
    int streamThread();

    // Low level reading and writing
    int readData(std::vector<float> & forces_array);
    int readFile(unsigned int rtn_data_len, unsigned char * output_data);
};


#endif // OPTODRIVER_HPP
