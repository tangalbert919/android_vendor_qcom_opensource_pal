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


#include "QalRingBuffer.h"
#include "QalCommon.h"
#define LOG_TAG "QalRingBuffer"

int32_t QalRingBuffer::removeReader(std::shared_ptr<QalRingBufferReader> reader)
{

    return 0;
}

size_t QalRingBuffer::read(std::shared_ptr<QalRingBufferReader>reader,
                           void* readBuffer, size_t readSize)
{
    return 0;
}

size_t QalRingBuffer::getFreeSize()
{

    size_t freeSize = bufferEnd_;
    std::vector<QalRingBufferReader*>::iterator it;

    for (it = readOffsets_.begin(); it != readOffsets_.end(); it++) {
        if ((*(it))->state_ == READER_ENABLED)
            freeSize = std::min(freeSize, bufferEnd_ - (*(it))->unreadSize_);
    }
    return freeSize;
}

void QalRingBuffer::updateUnReadSize(size_t writtenSize)
{
    int32_t i = 0;
    std::vector<QalRingBufferReader*>::iterator it;

    for (it = readOffsets_.begin(); it != readOffsets_.end(); it++, i++) {
        if ((*(it))->state_ == READER_ENABLED) {
            (*(it))->unreadSize_ += writtenSize;
            QAL_VERBOSE(LOG_TAG, "Reader (%d), unreadSize(%d)", i, (*(it))->unreadSize_);
        }
    }
}

void QalRingBuffer::updateIndices(uint32_t startIndice, uint32_t endIndice)
{
    startIndex = startIndice;
    endIndex = endIndice;
    QAL_VERBOSE(LOG_TAG, "start index = %u, end index = %u", startIndex, endIndex);
}

size_t QalRingBuffer::write(void* writeBuffer, size_t writeSize)
{
    /* update the unread size for each reader*/
    mutex_.lock();
    size_t freeSize = getFreeSize();
    size_t writtenSize = 0;
    int32_t i = 0;
    size_t sizeToCopy = 0;

    QAL_DBG(LOG_TAG, "Enter. freeSize(%d), writeOffset(%d)", freeSize, writeOffset_);

    if (writeSize <= freeSize)
        sizeToCopy = writeSize;
    else
        sizeToCopy = freeSize;

    if (sizeToCopy) {
        //buffer wrapped around
        if (writeOffset_ + sizeToCopy > bufferEnd_) {
            i = bufferEnd_ - writeOffset_;

            casa_osal_memcpy(buffer_ + writeOffset_, i, writeBuffer, i);
            writtenSize += i;
            sizeToCopy -= writtenSize;
            casa_osal_memcpy(buffer_, sizeToCopy, (char*)writeBuffer + writtenSize,
                             sizeToCopy);
            writtenSize += sizeToCopy;
            writeOffset_ = sizeToCopy;
        } else {
            casa_osal_memcpy(buffer_ + writeOffset_, sizeToCopy, writeBuffer,
                             sizeToCopy);
            writeOffset_ += sizeToCopy;
            writtenSize = sizeToCopy;
        }
    }
    updateUnReadSize(writtenSize);
    writeOffset_ = writeOffset_ % bufferEnd_;
    QAL_DBG(LOG_TAG, "Exit. writeOffset(%d)", writeOffset_);
    mutex_.unlock();
    return writtenSize;
}

void QalRingBuffer::reset()
{
    mutex_.lock();
    writeOffset_ = 0;
    mutex_.unlock();
    // reset indices also
}

size_t QalRingBufferReader::read(void* readBuffer, size_t bufferSize)
{
    int32_t readSize = 0;
    // Return 0 when no data can be read for current reader
    if (unreadSize_ == 0)
        return 0;

    ringBuffer_->mutex_.lock();

    // when writeOffset leads readOffset
    if (ringBuffer_->writeOffset_ > readOffset_) {
        unreadSize_ = ringBuffer_->writeOffset_ - readOffset_;

        if (bufferSize >= unreadSize_) {
            casa_osal_memcpy(readBuffer, unreadSize_, ringBuffer_->buffer_ +
                             readOffset_, unreadSize_);
            readOffset_ += unreadSize_;
            readSize = unreadSize_;
            unreadSize_ = 0;
        } else {
            casa_osal_memcpy(readBuffer, unreadSize_, ringBuffer_->buffer_ +
                             readOffset_, bufferSize);
            readOffset_ += bufferSize;
            readSize = bufferSize;
            unreadSize_ = ringBuffer_->writeOffset_ - readOffset_;
        }
    } else {
        //When readOffset leads WriteOffset
        int32_t freeClientSize = bufferSize;
        int32_t i = ringBuffer_->bufferEnd_ - readOffset_;

        if (bufferSize >= i) {
            casa_osal_memcpy(readBuffer, i, (char*)(ringBuffer_->buffer_ +
                             readOffset_), i);
            readSize = i;
            freeClientSize -= readSize;
            unreadSize_ = ringBuffer_->writeOffset_;
            readOffset_ = 0;
            //copy remaining unread buffer
            if (freeClientSize > unreadSize_) {
                casa_osal_memcpy((char *)readBuffer + readSize, unreadSize_,
                                 ringBuffer_->buffer_, unreadSize_);
                readSize += unreadSize_;
                readOffset_ = unreadSize_;
                unreadSize_ = 0;
            } else {
                //copy whatever we can
                casa_osal_memcpy((char *)readBuffer + readSize, freeClientSize,
                                 ringBuffer_->buffer_, freeClientSize);
                readSize += freeClientSize;
                readOffset_ = freeClientSize;
                unreadSize_ = ringBuffer_->writeOffset_ - readOffset_;
            }

        } else {
            casa_osal_memcpy(readBuffer, bufferSize, ringBuffer_->buffer_ +
                             readOffset_, bufferSize);
            readSize = bufferSize;
            readOffset_ += bufferSize;
            unreadSize_ = ringBuffer_->bufferEnd_ - readOffset_ +
                          ringBuffer_->writeOffset_;
        }
    }
    ringBuffer_->mutex_.unlock();
    return readSize;
}

size_t QalRingBufferReader::advanceReadOffset(size_t advanceSize)
{
    std::lock_guard<std::mutex> lock(ringBuffer_->mutex_);

    /* add code to advance the offset here*/

    return 0;
}

void QalRingBufferReader::updateState(qal_ring_buffer_reader_state state)
{
    state_ = state;
    // TODO: Handle read offsets for different scenario
}

void QalRingBufferReader::getIndices(uint32_t *startIndice, uint32_t *endIndice)
{
    *startIndice = ringBuffer_->startIndex;
    *endIndice = ringBuffer_->endIndex;
    QAL_VERBOSE(LOG_TAG, "start index = %u, end index = %u",
                ringBuffer_->startIndex, ringBuffer_->endIndex);
}

size_t QalRingBufferReader::getUnreadSize()
{
    return unreadSize_;
}

void QalRingBufferReader::reset()
{
    ringBuffer_->mutex_.lock();
    readOffset_ = 0;
    unreadSize_ = 0;
    state_ = READER_ENABLED;
    ringBuffer_->mutex_.unlock();
}

QalRingBufferReader* QalRingBuffer::newReader()
{
    QalRingBufferReader* readOffset =
                  new QalRingBufferReader(this);
    readOffsets_.push_back(readOffset);
    return readOffset;
}
