#pragma once
#ifndef LOGGER_H
#define LOGGER_H 

#include <mbed.h>
#include "mpack.h"

#define LOG_BIN_BUFFER_SIZE 512

class CompactBufferLogger
{
private:
    Serial * out; 
    char b1[LOG_BIN_BUFFER_SIZE];
    char b2[LOG_BIN_BUFFER_SIZE];
    char * send_buffer;
    char * write_buffer;
    char * write_ptr;
    uint16_t write_space;

    bool flushing;
    void flush();
    void flush_fin(int events);
    
    //growable writer;
    char* big_buffer;
    char* data;
    size_t size;
    mpack_writer_t writer;

public:
    int wait_for_flush_counter = 0;
    CompactBufferLogger(Serial * px);
    ~CompactBufferLogger();
    void begin(const char* type, uint8_t count); 
    void log(const char* key, uint64_t  value);
    void log(const char* key, int64_t  value);
    void log(const char* key, uint32_t  value);
    void log(const char* key, int32_t  value);
    void log(const char* key, uint16_t  value);
    void log(const char* key, int16_t  value);
    void log(const char* key, float value);
    void log(const char* key, bool value);
    void log(const char* key, float value_x, float value_y );
    void log(const char* key, float value_x, float value_y, float value_z);
    void submit();

};


#endif // LOGGER_H