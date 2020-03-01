/**
 * @file logger.h
 * @author revilo196    
 * @brief advanced logging and recording of data to a serial port using msgpack
 * @version 0.1
 * @date 2020-02-23
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef LOGGER_H
#define LOGGER_H 

#include <mbed.h>
#include "mpack.h"

//the size of both swapping buffers
#define LOG_BIN_BUFFER_SIZE 512

/**
 * @brief advanced logging function for logging and recording data to a serail port using msgpack fromat.
 * 
 * the CompactBufferLogger loggs named data packages to the serial port.
 * it uses two buffers, one for async sending data, one for writing/buffering logs, 
 * when the write buffer is full the buffers a swaped.
 * 
 * this implementation is block free as long as the sending of the send buffer is finished before the next write buffer is full
 * 
 * a log message is started with the begin() method
 * and ended with the submit() methode
 * 
 */
class CompactBufferLogger
{
private:
    Serial * out; 
    char b1[LOG_BIN_BUFFER_SIZE]; //first buffer
    char b2[LOG_BIN_BUFFER_SIZE]; //second buffer
    char * send_buffer;           //poiters for buffer swaping
    char * write_buffer;
    char * write_ptr;           //pointer to the current write buffer positon
    uint16_t write_space;       //space left in the write buffer

    bool flushing;
    void flush();
    void flush_fin(int events);
    
    //growable writer;
    char* big_buffer;
    char* data;
    size_t size;
    mpack_writer_t writer;

public:
    int wait_for_flush_counter = 0;  //couter how often the logger had to wait for the serial port to finish
    CompactBufferLogger(Serial * px);
    ~CompactBufferLogger();
    void begin(const char* type, uint8_t count); 
    void log(const char* key, uint64_t  value);
    void log(const char* key, int64_t  value);
    void log(const char* key, uint32_t  value);
    void log(const char* key, int32_t  value);
    void log(const char* key, uint16_t  value);
    void log(const char* key, int16_t  value);
    void log(const char* key, uint8_t  value);
    void log(const char* key, int8_t  value);
    void log(const char* key, float value);
    void log(const char* key, bool value);
    void log(const char* key, float value_x, float value_y );
    void log(const char* key, float value_x, float value_y, float value_z);
    void log(const char* key, float * vec, int size);
    void log(const char* key, float * mat, int h, int w);
    void log(const char* key, int * mat, int h, int w);
    void log(const char* key, int * vec, int size);
    void submit();

};


#endif // LOGGER_H