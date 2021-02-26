/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
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
#ifndef CONTEXTMANAGER_H
#define CONTEXTMANAGER_H

#include <vector>
#include <thread>

#include <PalApi.h>

enum USECASE {
    USECASE_ACD = 1,
    USECASE_RAW_DATA = 2,
    USECASE_UPD = 3,
};

class Usecase
{
protected:
    uint32_t usecase_id;
    uint32_t no_of_devices;
    pal_device *pal_devices;
    pal_stream_attributes *stream_attributes;
    pal_stream_handle_t *pal_stream;

public:

    Usecase(uint32_t usecase_id);
    ~Usecase();
    uint32_t GetUseCaseID();
    virtual int32_t SetUseCaseData(uint32_t size, void *data);
    int32_t Open();
    virtual int32_t Configure();
    int32_t Start();
    int32_t StopAndClose();
};

class UsecaseACD :public Usecase
{
private:
    struct pal_param_context_list *context_list;

public:
    UsecaseACD(uint32_t usecase_id);
    ~UsecaseACD();
    int32_t SetUseCaseData(uint32_t size, void *data);
    int32_t Configure();
};

class UsecaseUPD : public Usecase
{
public:
    UsecaseUPD(uint32_t usecase_id);
    ~UsecaseUPD();
};

class UsecaseRawData :public Usecase
{
public:
    UsecaseRawData(uint32_t usecase_id);
    ~UsecaseRawData();
};

class UsecaseFactory
{
public:
    static Usecase* UsecaseCreate(int32_t usecase_id);
};

class see_client
{
private:
    uint32_t see_id;
    std::map<uint32_t, Usecase*> usecases;

public:
    see_client(uint32_t id);
    ~see_client();
    uint32_t Get_SEE_ID();
    Usecase* Usecase_Get(uint32_t usecase_id);
    int32_t Usecase_Remove(uint32_t usecase_id);
    int32_t Usecase_Add(uint32_t usecase_id, Usecase* uc);
    void CloseAllUsecases();
};

class ContextManager
{
private:
    std::map<uint32_t, see_client *> see_clients;
    pal_stream_handle_t *proxy_stream;
    bool exit_cmd_thread_;
    std::thread cmd_thread_;

    see_client* SEE_Client_CreateIf_And_Get(uint32_t see_id);
    see_client * SEE_Client_Get_Existing(uint32_t see_id);
    int32_t OpenAndStartProxyStream();
    int32_t StopAndCloseProxyStream();
    int32_t CreateCommandProcessingThread();
    void DestroyCommandProcessingThread();
    void CloseAll();
    static void CommandThreadRunner(ContextManager& cm);

public:
    //functions
    ContextManager();
    ~ContextManager();
    int32_t Init();
    void DeInit();

    //test only, should be removed when integrating on linux.
    int32_t process_deregister_request(uint32_t see_id, uint32_t usecase_id);
    int32_t process_register_request(uint32_t see_id, uint32_t usecase, uint32_t size, void * payload);
    int32_t process_close_all();
    int32_t test();
};

#endif /*CONTEXTMANAGER_H*/