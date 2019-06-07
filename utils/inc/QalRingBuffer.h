/*
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdlib.h>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <iostream>
#include <string.h>

#ifndef QALRINGBUFFER_H_
#define QALRINGBUFFER_H_

#define DEFAULT_QAL_RING_BUFFER_SIZE 4096 * 10

typedef enum {
    READER_DISABLED = 0,
    READER_ENABLED = 1,
} qal_ring_buffer_reader_state;

class QalRingBuffer;

class QalRingBufferReader
{
protected:
    std::shared_ptr <QalRingBuffer> ringBuffer_;
    size_t unreadSize_;
    size_t readOffset_;
    qal_ring_buffer_reader_state state_;
public:
    size_t advanceReadOffset(size_t advanceSize);
    size_t read(void* readBuffer, size_t readSize);
    void updateState(qal_ring_buffer_reader_state state);
    void getIndices(uint32_t *startIndice, uint32_t *endIndice);
    size_t getUnreadSize();
    QalRingBufferReader(std::shared_ptr<QalRingBuffer>buffer) :
        readOffset_(0),
        unreadSize_(0),
        state_(READER_ENABLED),
        ringBuffer_((std::shared_ptr<QalRingBuffer>)buffer)
    {/* empty constructor */}

    ~QalRingBufferReader();
    friend class QalRingBuffer;
    friend class StreamSoundTrigger;
};

class QalRingBuffer
{
public:
    explicit QalRingBuffer(size_t bufferSize) :
        buffer_((char*)(new char[bufferSize])),
        writeOffset_(0),
        startIndex(0),
        endIndex(0),
        bufferEnd_(bufferSize)
    { /* empty constructor */}

    QalRingBufferReader* newReader();

    int32_t removeReader(std::shared_ptr<QalRingBufferReader> reader);
    size_t read(std::shared_ptr<QalRingBufferReader>reader, void* readBuffer,
                size_t readSize);
    size_t write(void* writeBuffer, size_t writeSize);
    size_t getFreeSize();
    void updateIndices(uint32_t startIndice, uint32_t endIndice);
protected:
    std::mutex mutex_;
    const size_t bufferEnd_;
    uint32_t startIndex;
    uint32_t endIndex;
    size_t writeOffset_;
    std::vector<QalRingBufferReader*> readOffsets_;
    char* buffer_;
    void updateUnReadSize(size_t writtenSize);
    friend class QalRingBufferReader;
};
#endif
