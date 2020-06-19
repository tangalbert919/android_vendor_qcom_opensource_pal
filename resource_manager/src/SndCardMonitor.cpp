/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "QAL: SndMonitor"
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/poll.h>
#include <list>
#include "ResourceManager.h"
#include "QalDefs.h"
#include "QalCommon.h"
#include "SndCardMonitor.h"

#define SNDCARD_PATH "/proc/asound/cards"
#define READY_TO_READ(p) ((p)->revents & (POLLIN|POLLPRI))
#define ERROR_IN_FD(p) ((p)->revents & (POLLERR|POLLHUP|POLLNVAL))

//TODO: Needs update if we support more than one snd card.
int SndCardMonitor::parseSndcards(int sndNum)
{
    int ret = 0;
    char path[128] = {0};
    int fd = -1;

    snprintf(path, sizeof(path), "/proc/asound/card%d/state", sndNum);
    QAL_VERBOSE(LOG_TAG, "Opening sound card state : %s", path);

    if ((fd = open(path, O_RDONLY)) < 0) {
        QAL_ERR(LOG_TAG, "Open %s failed error: %s", path, strerror(errno));
        return -EINVAL;
    }

    ret = SndCardMonitor::addNewSndCard(sndNum, fd);
    if (ret != 0) {
         QAL_ERR(LOG_TAG, "Adding new sound card failed, ret %d", ret);
         close(fd);
         return ret;
    }

    return ret;
}

int  SndCardMonitor::onSndcardStateUpdate(sndcard_t *s)
{
    int ret = 0;
    char *rd_buf;
    card_status_t status;

    rd_buf = readState(s->fd);

    QAL_VERBOSE(LOG_TAG, "card num %d, new state %s old state %d", s->card, rd_buf, s->status);

    if (strstr(rd_buf, "OFFLINE"))
        status = CARD_STATUS_OFFLINE;
    else if (strstr(rd_buf, "ONLINE"))
        status = CARD_STATUS_ONLINE;
    else {
        ret = -EINVAL;
        QAL_ERR(LOG_TAG, "unknown state, ret %d", ret);
        return ret;
    }

    s->status = status;
    QAL_ERR(LOG_TAG, "state %d", status);

    return status;
}

char* SndCardMonitor::readState(int fd)
{
    struct stat buf;
    char *state = NULL;

    if (fstat(fd, &buf) < 0)
        return NULL;

    off_t pos = lseek(fd, 0, SEEK_CUR);
    off_t avail = buf.st_size - pos;
    if (avail <= 0) {
        QAL_ERR(LOG_TAG, "avail %ld", avail);
        return NULL;
    }

    state = (char *)calloc(avail+1, sizeof(char));
    if (!state)
        return NULL;

    ssize_t bytes = read(fd, state, avail);
    if (bytes <= 0)
        return NULL;

    // trim trailing whitespace
    while (bytes && isspace(*(state+bytes-1))) {
        *(state + bytes - 1) = '\0';
        --bytes;
    }
    lseek(fd, 0, SEEK_SET);
    return state;
}

int SndCardMonitor::addNewSndCard(int card, int fd)
{
    char *state = NULL;
    bool online;
    sndcard_t *s = NULL;

    state = readState(fd);
    if (!state) {
        QAL_ERR(LOG_TAG, "Failed to read the state, card %d", card);
        return -EINVAL;
    }
    online = state && !strcmp(state, "ONLINE");
    QAL_DBG(LOG_TAG, "card %d initial state %s %d", card, state, online);

    s = (sndcard_t *)calloc(sizeof(sndcard_t), 1);
    if (!s) {
        QAL_ERR(LOG_TAG, "Calloc failed to sndcard");
        return -ENOMEM;
    }
    s->card = card;
    s->fd = fd;

    if (state)
        free(state);

    s->status = online ? CARD_STATUS_ONLINE : CARD_STATUS_OFFLINE;
    sndCards.push_back(s);
    return 0;
}

void SndCardMonitor::monitorThreadLoop()
{
    int i = 1;
    std::list<sndcard_t *>::iterator it;
    card_status_t status;
    std::shared_ptr<ResourceManager> rm = ResourceManager::getInstance();
    unsigned int num_poll_fds = sndCards.size() + 1/*pipe*/;
    struct pollfd *pfd = (struct pollfd *)calloc(sizeof(struct pollfd),
                                                  num_poll_fds);
    if (!pfd) {
        QAL_ERR(LOG_TAG, "Calloc failed for poll fds");
        return;
    }

    QAL_VERBOSE(LOG_TAG, "Start monitor threadLoop.");

    pfd[0].fd = intPipe[0];
    pfd[0].events = POLLPRI|POLLIN;
    for (it=sndCards.begin(); it!=sndCards.end(); it++) {
        pfd[i].fd = (*it)->fd;
        pfd[i].events = POLLPRI;
        i++;
    }

    while (1) {
        if (poll(pfd, num_poll_fds, -1) < 0) {
            int errno_ = errno;
            QAL_INFO(LOG_TAG, "poll() failed with err %s", strerror(errno_));
            switch (errno_) {
                case EINTR:
                case ENOMEM:
                    sleep(2);
                    continue;
                default:
                    /* above errors can be caused due to current system
                     * state .. any other error is not expected
                     */
                    QAL_ERR(LOG_TAG, "unxpected poll() system call failure");
                    break;
            }
        }
        QAL_VERBOSE(LOG_TAG, "out of poll");

        // check if requested to exit
        if (READY_TO_READ(&pfd[0])) {
            char buf[2]={0};
            read(pfd[0].fd, buf, 1);
            if (!strcmp(buf, "Q"))
                break;
        } else if (ERROR_IN_FD(&pfd[0])) {
            /* do not consider for poll again
             * POLLERR - can this happen?
             * POLLHUP - adev must not close pipe
             * POLLNVAL - fd is valid
             */
            QAL_ERR(LOG_TAG, "unxpected error in pipe poll fd 0x%x",
                             pfd[0].revents);
            pfd[0].fd *= -1;
        }

        i = 1;
        for (it=sndCards.begin(); it!=sndCards.end(); it++) {
            if (READY_TO_READ(&pfd[i])) {
                sndcard_t *snd = *it;
                status = static_cast<card_status_t>(onSndcardStateUpdate(snd));
                QAL_INFO(LOG_TAG, "rm %p status %d", rm.get(), status);
                rm->ssrHandler(status);
            } else if (ERROR_IN_FD(&pfd[i])) {
                /* do not consider for poll again
                 * POLLERR - can this happen as we are reading from a fs?
                 * POLLHUP - not valid for cardN/state
                 * POLLNVAL - fd is valid
                 */
                QAL_ERR(LOG_TAG, "unxpected error in card poll fd 0x%x",
                                 pfd[i].revents);
                pfd[i].fd *= -1;
            }
            ++i;
        }
    }
    return;
}

SndCardMonitor::SndCardMonitor(int sndNum)
{
    if (pipe(intPipe) < 0) {
        QAL_ERR(LOG_TAG, "failed to get pipe");
        return;
    }
    if (parseSndcards(sndNum) < 0) {
        QAL_ERR(LOG_TAG, "Unable to parse sound cards");
        goto parse_sndcards_error;
    }
    mThread = std::thread(&SndCardMonitor::monitorThreadLoop, this);
    QAL_INFO(LOG_TAG, "Snd card monitor init done.");
    return;

parse_sndcards_error:
    close(intPipe[0]);
    close(intPipe[1]);
    return;
}


SndCardMonitor::~SndCardMonitor()
{
   QAL_DBG(LOG_TAG, "destructor called");
   write(intPipe[1], "Q", 1);
   mThread.join();
   close(intPipe[0]);
   close(intPipe[1]);
   sndCards.clear();
}
