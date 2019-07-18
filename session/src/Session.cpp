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

#define LOG_TAG "Session"

#include "Session.h"
#include "Stream.h"
#include "ResourceManager.h"


#include "SessionGsl.h"
#include "SessionAlsaPcm.h"
#include "SessionAlsaCompress.h"

Session::Session()
{

}

Session::~Session()
{

}


Session* Session::makeSession(const std::shared_ptr<ResourceManager>& rm, const struct qal_stream_attributes *sAttr)
{
    if (!rm || !sAttr) {
        QAL_ERR(LOG_TAG,"Invalid parameters passed");
        return nullptr;
    }

    const qal_alsa_or_gsl ag = rm->getQALConfigALSAOrGSL();
    Session* s = (Session*) nullptr;

    switch (ag) {
        case ALSA:{
                switch (sAttr->type) {
                    //create compressed if the stream type is compressed
                    case QAL_STREAM_COMPRESSED:
                        s =  new SessionAlsaCompress(rm);
                        break;
                    default:
                        s = new SessionAlsaPcm(rm);
                        break;
                    }
                }
                break;
        case GSL:
                s = ((Session*) new SessionGsl(rm));
                break;
         default:
                s = ((Session*) nullptr);
                break;
    };

    return s;
}

