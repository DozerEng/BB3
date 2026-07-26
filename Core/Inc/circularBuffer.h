/*
 * circularBuffer.h
 *
 *  Created on: Mar 8, 2026
 *      Author: mpill
 */

#ifndef INC_CIRCULARBUFFER_H_
#define INC_CIRCULARBUFFER_H_

#include <stdint.h>

/*
 * Circular Buffer Struct
 */
typedef struct
{
    uint8_t *buffer;       // pointer to memory
    uint16_t size;         // size of buffer (power of 2)
    uint16_t mask;         // precomputed mask for wrapping
    volatile uint16_t readIndex;
    volatile uint16_t writeIndex;
} CircularBuffer_t;


/*
 * Creation / Initialization
 */
static inline CircularBuffer_t circularBuffer_new(uint8_t *buffer, uint16_t size)
{
    CircularBuffer_t buf;

    buf.buffer = buffer;
    buf.size = size;
    buf.mask = size - 1;       // requires buf.size to be a power of 2
    buf.readIndex = 0;
    buf.writeIndex = 0;

    return buf;
}

static inline void circularBuffer_clear(CircularBuffer_t *buf)
{
    buf->readIndex = 0;
    buf->writeIndex = 0;
}


/*
 * State Functions
 */
static inline uint8_t circularBuffer_isEmpty(CircularBuffer_t *buf)
{
    return buf->readIndex == buf->writeIndex;
}

static inline uint8_t circularBuffer_isFull(CircularBuffer_t *buf)
{
    return ((buf->writeIndex + 1) & buf->mask) == buf->readIndex;
}

static inline uint16_t circularBuffer_available(CircularBuffer_t *buf)
{
    return (buf->writeIndex - buf->readIndex) & buf->mask;
}

static inline uint16_t circularBuffer_space(CircularBuffer_t *buf)
{
    return (buf->readIndex - buf->writeIndex - 1) & buf->mask;
}


/*
 * Read Functions
 */
static inline uint8_t circularBuffer_readByte(CircularBuffer_t *buf, uint8_t *byte)
{
    uint16_t readIndex = buf->readIndex;

    if(readIndex == buf->writeIndex)
        return 0;

    *byte = buf->buffer[readIndex];
    buf->readIndex = (readIndex + 1) & buf->mask;

    return 1;
}

static inline uint8_t circularBuffer_peek(CircularBuffer_t *buf, uint8_t *byte)
{
    if(buf->readIndex == buf->writeIndex)
        return 0;

    *byte = buf->buffer[buf->readIndex];
    return 1;
}

static inline uint16_t circularBuffer_read(CircularBuffer_t *buf, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    uint16_t readIndex = buf->readIndex;

    while((readIndex != buf->writeIndex) && (count < len))
    {
        data[count++] = buf->buffer[readIndex];
        readIndex = (readIndex + 1) & buf->mask;
    }

    buf->readIndex = readIndex;

    return count;
}


/*
 * Write Functions
 */
static inline uint8_t circularBuffer_writeByte(CircularBuffer_t *buf, uint8_t byte)
{
    uint16_t next = (buf->writeIndex + 1) & buf->mask;

    if(next == buf->readIndex)
        return 0;

    buf->buffer[buf->writeIndex] = byte;
    buf->writeIndex = next;

    return 1;
}

static inline uint16_t circularBuffer_write(CircularBuffer_t *buf, uint8_t *data, uint16_t len)
{
    uint16_t count = 0;
    uint16_t writeIndex = buf->writeIndex;

    while(count < len)
    {
        uint16_t next = (writeIndex + 1) & buf->mask;

        if(next == buf->readIndex)
            break;

        buf->buffer[writeIndex] = data[count++];
        writeIndex = next;
    }

    buf->writeIndex = writeIndex;

    return count;
}

#endif /* INC_CIRCULARBUFFER_H_ */
