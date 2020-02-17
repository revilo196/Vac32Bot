#include "logger.h"

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
void CompactBufferLogger::begin(const char* type, uint8_t count)
{
    mpack_writer_init_growable(&writer,&data,&size);
    mpack_start_map(&writer, 1);
    mpack_write_cstr(&writer, type);
    mpack_start_map(&writer, count);
}
void CompactBufferLogger::log(const char* key, uint64_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_uint (&writer, value);
}
void CompactBufferLogger::log(const char* key, int64_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_int (&writer, value);
}
void CompactBufferLogger::log(const char* key, uint32_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_u32 (&writer, value);
}
void CompactBufferLogger::log(const char* key, int32_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_i32 (&writer, value);
}
void CompactBufferLogger::log(const char* key, uint16_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_u16 (&writer, value);
}
void CompactBufferLogger::log(const char* key, int16_t  value) {
    mpack_write_cstr(&writer, key);
    mpack_write_i16 (&writer, value);
}
void CompactBufferLogger::log(const char* key, float value) {
    mpack_write_cstr(&writer, key);
    mpack_write_float (&writer, value);
}
void CompactBufferLogger::log(const char* key, bool value) {
    mpack_write_cstr(&writer, key);
    mpack_write_bool (&writer, value);
}
void CompactBufferLogger::log(const char* key, float value_x, float value_y) {
    mpack_write_cstr(&writer, key);
    mpack_start_array(&writer,2);
        mpack_write_float (&writer, value_x);
        mpack_write_float (&writer, value_y);
    mpack_finish_array (&writer);
}
void CompactBufferLogger::log(const char* key, float value_x, float value_y, float value_z) {
    mpack_write_cstr(&writer, key);
    mpack_start_array(&writer,3);
        mpack_write_float (&writer, value_x);
        mpack_write_float (&writer, value_y);
        mpack_write_float (&writer, value_z);
    mpack_finish_array (&writer);
}
void CompactBufferLogger::submit()
{
    mpack_finish_map(&writer);
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