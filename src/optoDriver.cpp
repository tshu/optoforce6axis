#include "optoDriver.hpp"
#include <ros/ros.h>


//Sets up the Optoforce 6-axis sensor and starts data streaming thread
optoDriver::optoDriver(int speed, int filter, bool zero){
    fd = -2;             // File descriptor is invalid to start
    BaudRate = B1000000; // 1000000 baud rate
    ByteSize = CS8;      // 8 bits
    StopBits = CSTOPB; 	 // One stop bit
    Parity 	 = ~PARENB;  // No parity
    port = "optoforce";  // Default is ttyACM0, any other value is a static port rerouting to ttyACM0

    stream_byte_len = 22;
    streamON = false;
    newData  = false;
    hasFirst = false;
    SerialNumber = 0;
    last_stream_data = (unsigned char*)malloc(sizeof(unsigned char) * stream_byte_len);

    this->speed = speed;
    this->filter = filter;
    this->zero = zero;

    if (openAndSetupComPort(port)) {
        setConfig(speed, filter, zero);
        startStreaming();
    }
}

//Constructor chaining
optoDriver::optoDriver(){
    optoDriver(10, 4, true);
}

optoDriver::~optoDriver() {
    stopStreaming();
    closeComPort();
}

//Reads a single value from the data stream and populates public vector forces_vec with force and torque readings
int optoDriver::spinOnce() {
    if (!streamON) {
        if (getFD() > 0) {
            setConfig(speed, filter, zero);
            startStreaming();
        } else {
            if (openAndSetupComPort(port)) {
                setConfig(speed, filter, zero);
                startStreaming();
            } else {
                printf("Error with comport when spinOnce.");
                return 0;
            }
        }
        printf("Stream not on when spinOnce().");
        return 0;
    }

    //wait for header to match upon initial streaming startup, then populate forces_vec
    while (forces_vec.size() == 0) {
            getStream();
    }
    getStream();

    return 1;
}

// Initialize the USB port for reading. Necessary to perform before function setConfig()
// and function startStreaming()
int optoDriver::openAndSetupComPort(const char* comport) {
    char str[12];

    strcpy(str, "/dev/");
    strcat(str, comport);
    fd = open(str, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    if ( fd == -1 ) {
        printf("COMPORT %s not available!\n", str);
        fd = -2;
        return 0;
    }

    if (!openFD())
        return 0;

    printf("Established connection to %s: %d\n", comport, fd);

    fflush(stdout);
    return 1;
}

int optoDriver::getFD() {
    return fd;
}

int optoDriver::openFD() {
    struct termios options;  // Terminal options
    int rc;

    fcntl(fd, F_SETFL, 0);

    // Get last COM port attributes
    tcgetattr(fd, &old_options);
    options = old_options;

    // Flush the port's buffers (in and out) before we start using it
    tcflush(fd, TCIOFLUSH);

    // Set incoming baud rates
    cfsetispeed(&options, BaudRate);

    // Set outgoing baud rates
    cfsetospeed(&options, BaudRate);

    // Control Mode Parameters
    options.c_cflag &= CS8;
    options.c_cflag &= ~INPCK;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~(PARENB | PARODD);

    // Input Mode Parameters
    options.c_iflag = IGNBRK;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Local Flag Parameters
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Timeout and return length
    options.c_cc[1] = 0; // or use 1 if you feel like perma-waiting
    options.c_cc[VTIME]= 10; // Timeout in 0.1s

    //tcflush(fd, TCIOFLUSH); // Flush input and output data from device

    // Set the new attributes
    if((rc = tcsetattr(fd, TCSANOW, &options)) != 0){
        perror("Failed to set attr: ");
        return 0;
    }

    tcflush(fd, TCIOFLUSH); // clear buffer

    return 1;
}

int optoDriver::closeComPort() {
    if (fd > 0) {
        tcflush(fd, TCIOFLUSH); // clear buffer
        tcsetattr(fd, TCSANOW, &old_options); // reset the terminal options
        close(fd);
        fd = -2;
        return 1;
    }
    printf("Invalid file handle.");
    return 0;
}

// Creates a separate thread to constantly update last_stream_data, which may be extracted with
// public function getStream()
int optoDriver::startStreaming() {
    int ret = 1;
    last_packet_time = std::chrono::system_clock::now();
    last_retrieval_time = last_packet_time;
    if (ret == 1){
        // Start thread
        streamON = true;
        readThread = std::thread(&optoDriver::streamThread, this);
        printf("Started stream: \t %d\n", fd);
        fflush(stdout);
    }
    return 1;
}

// Public function to stop streaming thread
// Called by ~optoDriver() destructor
int optoDriver::stopStreaming() {
    if (streamON) {
        {
            {
                std::lock_guard<std::mutex> guard(mu);
                streamON = false;
            }
            if (readThread.joinable()) readThread.join();
        }
        printf("Closing stream: \t %d\n", fd);
        fflush(stdout);
        hasFirst = false;

        tcflush(fd, TCIOFLUSH); // clear buffer
        return 1;
    }
    
    printf("Could not close stream. Stream not on.");
    return 0;
}

//continually calls checkStream() while streaming thread is operating
int optoDriver::streamThread() {
    int ok = 1;

    while (ok) {
        ok = checkStream();
    }

    return 0;
}

//if readFile(args) is successful, raise newData flag and update time variables
int optoDriver::checkStream() {
    unsigned char * dat;
    int on;
    int ret = 0;

    dat = (unsigned char*)malloc(stream_byte_len * sizeof(unsigned char));

    {
        ret = readFile(stream_byte_len, (unsigned char*)dat);
    }

    std::lock_guard<std::mutex> guard(mu); {
        if (ret == 1) {
            // Get the time
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

            // Lock memory and update
            memcpy(last_stream_data, dat, sizeof(unsigned char)*stream_byte_len);
            period = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_packet_time);
            last_packet_time = now;
            newData = true;
            
        } else {
            newData = false;
        }
        free(dat);
        on = streamON;
    }

    return on;
}

// Private function to read data bytes from the USB port
int optoDriver::readFile(unsigned int rtn_data_len, unsigned char * output_data) {
    // Check to make sure the class has a valid device
    if ( fd < 0 ) {
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }
    if (!rtn_data_len) {
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }

    // Setup variables
    int header_size = 4;
    int num_bytes_read;
    int expectedHeader[] = {170, 7, 8, 16};
    int headerCheck = 0;


    // Initialize file descriptor sets
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    // Set timeout to 1.0 seconds
    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    if (!select(fd + 1, &read_fds, NULL, NULL, &timeout)) {
        perror("Could not select device!\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }

    // Wait for input to become ready or until the time out; the first parameter is
    // 1 more than the largest file descriptor in any of the sets
    num_bytes_read = read( fd, output_data, stream_byte_len ); // there was data to read

    // If nothing could be read, throw a message stating so
    if (!num_bytes_read) {
        printf("Error in reading header. Nothing to read. Attempting to restart stream.\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        stopStreaming();
        closeComPort();
        if (openAndSetupComPort(port)) {
            setConfig(speed, filter, zero);
            startStreaming();
            printf("Stream restarted.");
        }
        return 0;
    }

    // Gets the header and checks for completeness upon successful read
    for (int i = 0; i < header_size; i++) {
        if (output_data[i] == expectedHeader[i]) {
            headerCheck++;
        }
    }

    // Headers will not match on first data packet. Clears buffer and attempts to match on next streaming call
    if (headerCheck != header_size) {
        printf("Success not true on read\n");
        printf("Header received: ");
        for( int i = 0 ; i < 4 ; i ++ ){
            printf("%i, ", output_data[i]);
        }
        printf(". Expected: ");
        for( int i = 0 ; i < 4 ; i ++ ){
            printf("%i, ", expectedHeader[i]);
        }
        printf("\n");

        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }

    // Sets the hasFirst flag to indicate stream has correctly started
    if (!hasFirst) {
            hasFirst = true;
    }

    // Check for unexpected length of data
    if (num_bytes_read != stream_byte_len) {
        perror("Error in reading stream.\n");
        tcflush(fd, TCIOFLUSH); // clear buffer
        return 0;
    }
    return 1;

}

// Get current stream value for an external user
// Copies the stream shared memory data and exports it to forces_vec
int optoDriver::getStream() {
    // Guard takes on the mutex for the class and locks the memory for the functions herein.
    // This memory is released when it goes out of scope (i.e. the function returns or fails.
    {
        std::lock_guard<std::mutex> guard(mu);
        if (newData) {
            //memcpy(data,      last_stream_data,   sizeof(float)*stream_byte_len);  
            readData(forces_vec);
            dt = last_packet_time - last_retrieval_time;
            newData = false;
        } else {
            return 0;
        }
    }
    //lag in terms of clock cycles
    this->diff = dt.count();
    last_retrieval_time = std::chrono::system_clock::now();
    return 1;
}

//Reads the raw counts from last_stream_data and multiplies by the calArray to return data in N and Nm
int optoDriver::readData(std::vector<float> & forces_vector) {

    if (!hasFirst) {
        printf("No data to read. ");
        return 0;
    }

    forces_vector.resize(6);
    //stores element-wise multiplication of stream data and cal array into input forces_vector
    int16_t sixteen[6];

    for (int i = 8; i < 20; i += 2) {
        sixteen[i/2 - 4] = last_stream_data[i] << 8 | last_stream_data[i+1];
    }

    std::transform(&sixteen[0], &sixteen[6],
                calArray.begin(), forces_vector.begin(),  // assumes v1,v2 of same size > 1, 
                std::multiplies<float>() );

    return 1;
}
//Writes a configuration packet to the Optoforce transducer
//int speed: transmission frequency. The device itself internally polls at a constant 1kHz, but will
//          write to the port at the designated frequency; default is 100Hz
//int filter: low pass filter; default is 15Hz
//bool zero: commands the transducer to zero its readings; value of 0 will clear offsets and value of 1
//          will zero with current offsets

int optoDriver::setConfig(unsigned int speed, unsigned int filter, bool zero) {
    if ( fd < 0 ) {
        printf("Invalid file handle during setConfig.");
        return 0;
    }

    // Setup variables
    unsigned int write_size = 9; //total number of bytes in send
    unsigned char * write_array;
    int wv;

    uint16_t chkSum = 0;
    unsigned int i;

    // Allocate memory for command write
    write_array = (unsigned char*) malloc(write_size * sizeof(unsigned char));
    if( write_array == NULL ){
        printf("ERROR: Out of memory\n");
        return 0;
    }
    memset((void *)write_array, 0, sizeof(write_array)); // clear the buffer out.

    //set up the header
    write_array[0] = 170;
    write_array[1] = 0;
    write_array[2] = 50;
    write_array[3] = 3;

    switch(speed){
        case 0: //Stop transmission
        case 1: //1000Hz
        case 3: //333Hz
        case 10: //100Hz
        case 33: //30Hz
        case 100: // 10Hz
            write_array[4] = speed;
            this->speed = speed;
            printf("\nSpeed set to %i. ", speed);
            break;
        default:
            write_array[4] = 1;
            this->speed = 4;
            printf("Invalid speed. Speed set to 100Hz. ");
    }

    switch(filter){
        case 0: //No filtering
        case 1: //500Hz
        case 2: //150Hz
        case 3: //50Hz
        case 4: //15Hz
        case 5: //5Hz
        case 6: //1.5Hz
            write_array[5] = filter;
            this->filter = filter;
            printf("Filter set to %i. ", filter);
            break;
        default:
            write_array[5] = 4;
            this->filter = 4;
            printf("Invalid filter. Filter set to 15Hz. ");
    }

    if(zero){
        write_array[6] = 255;
        this->zero = true;
        printf("Optoforce zeroed.\n");
    } else {
        write_array[6] = 0;
        this->zero = false;
        printf("Optoforce not zeroed.\n");
    }

    //create checksum: sum of indices 0 to 7, stored as uint16_t

    for (i = 0; i < 7; i++){
        chkSum += write_array[i];
    }
    write_array[7] = (unsigned char) (chkSum >> 8);
    write_array[8] = (unsigned char) (chkSum);

    // Write the command to the device
    wv = write(fd, write_array, write_size);  //Send data
    if (wv == -1) {
        perror("Write Error: \n");
    } else if ( wv == 0 ) {
        perror("Nothing was written: \n");
    }

    printf("Number of bytes written during config: %i \n", wv);

    free(write_array);
    return 1;
}




