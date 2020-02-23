/**
 * @file logger.cpp
 * @author revilo196
 * @brief advanced logging and recording of data to a serial port using msgpack
 * @version 0.1
 * @date 2020-02-23
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "logger.h"

/**
 * @brief Construct a new Compact Buffer Logger:: Compact Buffer Logger object 
 * 
 * @param px serial port used for writing logs 
 */
CompactBufferLogger::CompactBufferLogger(Serial * px)
{
    flushing = false;
    out = px;
    send_buffer = b1;
    write_buffer = b2;
    write_ptr = write_buffer;
    write_space = LOG_BIN_BUFFER_SIZE;
}

CompactBufferLogger::~CompactBufferLogger()
{
}

/**
 * @brief begin a new log message
 * 
 * this initializes the mpack msg builder
 * 
 * @param type const char* (cstr) defining the message type
 * @param count count of values to log (count of log(...) calls between begin() and submit() )
 */
void CompactBufferLogger::begin(const char* type, uint8_t count){
    mpack_writer_init_growable(&writer,&data,&size);
    mpack_start_map(&writer, 1);
    mpack_write_cstr(&writer, type);
    mpack_start_map(&writer, count);
}
/**
 * @brief log a uint64_t key, value pair
 * 
 * @param key cstr name of the value
 * @param value uint64_t value
 */
void CompactBufferLogger::log(const char* key, uint64_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_uint (&writer, value);
}
/**
 * @brief log a int64_t key, value pair
 * 
 * @param key cstr name of the value
 * @param value int64_t value
 */
void CompactBufferLogger::log(const char* key, int64_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_int (&writer, value);
}
/**
 * @brief log a uint32_t key, value pair
 * 
 * @param key cstr name of the value
 * @param value uint32_t value
 */
void CompactBufferLogger::log(const char* key, uint32_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_u32 (&writer, value);
}
/**
 * @brief log a int32_t key, value pair
 * 
 * @param key cstr name of the value
 * @param value int32_t value
 */
void CompactBufferLogger::log(const char* key, int32_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_i32 (&writer, value);
}
/**
 * @brief log a uint16_t key, value pair
 * 
 * @param key cstr name of the value
 * @param value uint16_t value
 */
void CompactBufferLogger::log(const char* key, uint16_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_u16 (&writer, value);
}
/**
 * @brief log a int16_t key, value pair
 * 
 * @param key cstr name of the value
 * @param value int16_t value
 */
void CompactBufferLogger::log(const char* key, int16_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_i16 (&writer, value);
}
/**
 * @brief log a float key-value-pair
 * 
 * @param key cstr name of the value
 * @param value float value
 */
void CompactBufferLogger::log(const char* key, float value) {
    mpack_write_cstr(&writer, key);
    mpack_write_float (&writer, value);
}
/**
 * @brief log a bool key-value-pair
 * 
 * @param key cstr name of the value
 * @param value bool value
 */
void CompactBufferLogger::log(const char* key, bool value) {
    mpack_write_cstr(&writer, key);
    mpack_write_bool (&writer, value);
}
/**
 * @brief  log a 2D vector key-value-pair
 * 
 * @param key cstr name of the value
 * @param value_x x corrdinate 
 * @param value_y y corrdinate
 */
void CompactBufferLogger::log(const char* key, float value_x, float value_y) {
    mpack_write_cstr(&writer, key);
    mpack_start_array(&writer,2);
        mpack_write_float (&writer, value_x);
        mpack_write_float (&writer, value_y);
    mpack_finish_array (&writer);
}
/**
 * @brief log a 3D vector key-value-pair
 * 
 * @param key cstr name of the value
 * @param value_x x corrdinate 
 * @param value_y y corrdinate
 * @param value_z z corrdinate
 */
void CompactBufferLogger::log(const char* key, float value_x, float value_y, float value_z) {
    mpack_write_cstr(&writer, key);
    mpack_start_array(&writer,3);
        mpack_write_float (&writer, value_x);
        mpack_write_float (&writer, value_y);
        mpack_write_float (&writer, value_z);
    mpack_finish_array (&writer);
}
/**
 * @brief log a named array values
 * 
 * @param key cstr name of the array
 * @param vec pointer to the array
 * @param size size of the array
 */
void CompactBufferLogger::log(const char* key, float * vec, int size){
mpack_write_cstr(&writer, key);
mpack_start_array(&writer, size);
    for (uint16_t i = 0; i< size; i++) {
             mpack_write_float(&writer, vec[i]);
        }
mpack_finish_array (&writer);
}

/**
 * @brief log a 2-Dimensional float array (matrix) 
 * 
 * @param key cstr name of the matrix
 * @param mat pointer to the matrix
 * @param h height matrix (1. Dimension)
 * @param w  width of the matrix (2. Dimension)
 */
void CompactBufferLogger::log(const char* key, float * mat, int h, int w){
    mpack_write_cstr(&writer, key);
    mpack_start_array(&writer,h);
        for (uint16_t i = 0; i< h; i++) {
            mpack_start_array(&writer,w);
            for (uint16_t j = 0; j< w; j++) { 
                mpack_write_float(&writer, mat[i*h + j]);
            }
            mpack_finish_array (&writer);
        }
    mpack_finish_array (&writer);
}

/**
 * @brief log a 2-Dimensional int array (matrix) 
 * 
 * @param key cstr name of the matrix
 * @param mat pointer to the matrix
 * @param h height matrix (1. Dimension)
 * @param w  width of the matrix (2. Dimension)
 */
void CompactBufferLogger::log(const char* key, int * mat, int h, int w){
    mpack_write_cstr(&writer, key);
    mpack_start_array(&writer,h);
        for (uint16_t i = 0; i< h; i++) {
            mpack_start_array(&writer,w);
            for (uint16_t j = 0; j< w; j++) { 
                mpack_write_int(&writer, mat[i*h + j]);
            }
            mpack_finish_array (&writer);
        }
    mpack_finish_array (&writer);
}

/**
 * @brief log a named integer array values
 * 
 * @param key cstr name of the array
 * @param vec pointer to the array
 * @param size size of the array
 */
void CompactBufferLogger::log(const char* key, int * vec, int size){
mpack_write_cstr(&writer, key);
mpack_start_array(&writer, size);
    for (uint16_t i = 0; i< size; i++) {
             mpack_write_int(&writer, vec[i]);
        }
mpack_finish_array (&writer);
}

/**
 * @brief submits the log message.
 * completes the msgpack and writes the message to the write buffer
 * if the write buffer is full the buffers are swaped and data is send over the serial port
 * 
 */
void CompactBufferLogger::submit()
{
    mpack_finish_map(&writer);

    // finish writing
    if (mpack_writer_destroy(&writer) != mpack_ok) {
        out->printf("An error occurred encoding the data!\n");
    }

    if(size + 6 <= write_space) {
        //there is enough space in the buffer

        //write header into the buffer
        write_ptr[0] = 0xFF;
        write_ptr[1] = 0x4D;
        write_ptr[2] = 0x50;
        write_ptr[3] = 0xFF;
        (*((uint16_t*)(write_ptr+4))) = (uint16_t)size;

        //write data into the buffer
        memcpy(write_ptr+6, data, size);

        //increment buffer pointer 
        write_space -= size + 6;
        write_ptr += size + 6;

    } else if (size + 6 <= LOG_BIN_BUFFER_SIZE) {
        flush(); //swaps the buffers

        write_ptr[0] = 0xFF;
        write_ptr[1] = 0x4D;
        write_ptr[2] = 0x50;
        write_ptr[3] = 0xFF;
        (*((uint16_t*)(write_ptr+4))) = (uint16_t)size;

        //write data into the buffer
        memcpy(write_ptr+6, data, size);

        //increment buffer pointer 
        write_space -= size + 6;
        write_ptr += size + 6;

    } else {
        // the message is too big for one buffer!
        // direct output
        // using a big buffer
        big_buffer = (char*)malloc(size+6);
        big_buffer[0] = 0xFF;
        big_buffer[1] = 0x4D;
        big_buffer[2] = 0x50;
        big_buffer[3] = 0xFF;
        (*((uint16_t*)(big_buffer+4))) = (uint16_t)size;
        flushing = true;
        out->write((uint8_t*)big_buffer,(int)size+6,callback(this,&CompactBufferLogger::flush_fin),SERIAL_EVENT_TX_COMPLETE);
    }
    free(data);
    data = NULL;
    size = 0;
}

/**
 * @brief start sending the data and swap the buffers 
 * 
 */
void CompactBufferLogger::flush()
{   
    if (flushing) {
        wait_for_flush_counter++;
        while (flushing) {
            wait_us(1000);
        } // wait for the last flush to finish
        flushing = false;
    }
    flushing = true; // write data to serial
    out->write((uint8_t*)write_buffer,LOG_BIN_BUFFER_SIZE-write_space,callback(this,&CompactBufferLogger::flush_fin),SERIAL_EVENT_TX_COMPLETE);

    // swap buffers
    char *s = send_buffer; 
    send_buffer = write_buffer;
    write_buffer = s;
    write_ptr = s;
    //reset buffer space
    write_space = LOG_BIN_BUFFER_SIZE;

}

/**
 * @brief callback of the serial port write functon 
 * gets called when the write is complete
 * @param events 
 */
void CompactBufferLogger::flush_fin(int events)
{
    if ((events & SERIAL_EVENT_TX_COMPLETE) > 0)
    {
        flushing = false;
        if (big_buffer != NULL) {
            //free the big buffer
            free(big_buffer);
            big_buffer = NULL;
        }

    }
}