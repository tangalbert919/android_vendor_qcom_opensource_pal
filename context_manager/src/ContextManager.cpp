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

#include <iostream>
#include <conio.h>
#include <chrono>
#include <ContextManager.h>

#define LOG_TAG "PAL: ContextManager"


int32_t ContextManager::process_register_request(uint32_t see_id, uint32_t usecase_id, uint32_t size, void *payload)
{
    int32_t rc = 0;
    Usecase *uc = NULL;
    see_client *seeclient = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter see_id:%d, usecase_id:%d, payload_size:%d", see_id, usecase_id, size);

    seeclient = SEE_Client_CreateIf_And_Get(see_id);
    if (seeclient == NULL) {
        rc = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error:%d, Failed to get see_client:%d", rc, see_id);
        goto exit;
    }

    uc = seeclient->Usecase_Get(usecase_id);
    if (uc == NULL) {
        PAL_VERBOSE(LOG_TAG, "Creating new Usecase:%d for see_id:%d", usecase_id, see_id);

        uc = UsecaseFactory::UsecaseCreate(usecase_id);
        if (uc == NULL) {
            rc = -EINVAL;
            PAL_ERR(LOG_TAG, "Error:%d, Failed to create usease:%d for see_client:%d", rc, usecase_id, see_id);
            goto exit;
        }

        rc = uc->Open();
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d, Failed to Open() usease:%d for see_client:%d", rc, usecase_id, see_id);
            goto exit;
        }

        rc = uc->SetUseCaseData(size, payload);
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d, Failed to Setusecase() usease:%d for see_client:%d", rc, usecase_id, see_id);
            goto exit;
        }

        rc = uc->Configure();
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d, Failed to Configure() usease:%d for see_client:%d", rc, usecase_id, see_id);
            uc->StopAndClose();
            goto exit;
        }

        rc = uc->Start();
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d, Failed to Start() usease:%d for see_client:%d", rc, usecase_id, see_id);
            uc->StopAndClose();
            goto exit;
        }

        seeclient->Usecase_Add(usecase_id, uc);

    }
    else {
        PAL_VERBOSE(LOG_TAG, "Retrieving existing Usecase:%d for see_id:%d", usecase_id, see_id);
        rc = uc->SetUseCaseData(size, payload);
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d, Failed to Setusecase() usease:%d for see_client:%d", rc, usecase_id, see_id);
            goto exit;
        }
        rc = uc->Configure();
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d, Failed to Configure() usease:%d for see_client:%d", rc, usecase_id, see_id);
            uc->StopAndClose();
            goto exit;
        }
    }

exit:
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);
    return rc;
}

int32_t ContextManager::process_deregister_request(uint32_t see_id, uint32_t usecase_id)
{
    int32_t rc = 0;
    Usecase *uc = NULL;
    see_client *seeclient = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter see_id:%d, usecase_id:%d", see_id, usecase_id);

    seeclient = SEE_Client_Get_Existing(see_id);
    if (seeclient == NULL) {
        rc = -1; //EFAILED;
        PAL_ERR(LOG_TAG, "Error:%d, Failed to get see_client:%d", rc, see_id);
        goto exit;
    }

    uc = seeclient->Usecase_Get(usecase_id);
    if (uc == NULL) {
        rc = -EINVAL;
        PAL_VERBOSE(LOG_TAG, "Error:%d Retrieving existing Usecase:%d for see_id:%d", rc, usecase_id, see_id);
        goto exit;
    }

    rc = uc->StopAndClose();
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d, Failed to StopAndClose() usease:%d for see_client:%d", rc, usecase_id, see_id);
        rc = 0; //force rc to success as nothing can be done if teardown fails.
    }

    rc = seeclient->Usecase_Remove(usecase_id);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d, Failed to remove usease:%d for see_client:%d", rc, usecase_id, see_id);
        rc = 0; //force rc to success as nothing can be done if teardown fails.
    }

exit:
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);
    return rc;
}

int32_t ContextManager::process_close_all()
{
    PAL_VERBOSE(LOG_TAG, "Enter:");

    this->CloseAll();

    PAL_VERBOSE(LOG_TAG, "Exit: rc:%d", 0);

    return 0;
}

int32_t ContextManager::test() {

    int32_t rc = 0;
    uint32_t context_ids[6] = { 5, 0x8001307, 0x8001307 + 1 , 0x8001307 + 2 , 0x8001307 + 3 , 0x8001307 + 4 };
    uint32_t context_ids_2[3] = { 2, 0x8001307 + 5, 0x8001307 + 6 };
    uint32_t see_id_123 = 123;
    uint32_t see_id_456 = 456;
    uint32_t see_id_789 = 789;
    uint32_t usecase_id_acd = USECASE_ACD;
    uint32_t usecase_id_rawdata = USECASE_RAW_DATA;
    uint32_t usecase_id_upd = USECASE_UPD;

    PAL_VERBOSE(LOG_TAG, "Enter");
    rc = this->Init();
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to Init() ContextManger", rc);
        goto exit;
    }

    rc = this->process_register_request(see_id_123, usecase_id_acd, sizeof(context_ids), (void *)context_ids);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
        goto exit;
    }

    rc = this->process_register_request(see_id_123, usecase_id_acd, sizeof(context_ids), (void *)context_ids);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
    }

    rc = this->process_register_request(see_id_456, usecase_id_acd, sizeof(context_ids), (void *)context_ids_2);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
    }

    rc = this->process_register_request(see_id_789, usecase_id_rawdata, 0, NULL);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
    }

    rc = this->process_register_request(see_id_789, usecase_id_upd, 0, NULL);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
    }

    rc = this->process_deregister_request(see_id_123, usecase_id_acd);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_de-register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
    }

    rc = this->process_deregister_request(see_id_789, usecase_id_rawdata);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_de-register_request() for see_id:%d, usecase_id:%d", rc, see_id_123, usecase_id_acd);
    }

    rc = this->process_close_all();
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to process_close_all()", rc);
    }
    PAL_ERR(LOG_TAG, "*********************************************");
    this->DeInit();
exit:
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);
    return rc;
}

ContextManager::ContextManager()
{
    PAL_VERBOSE(LOG_TAG, "Enter");
    PAL_VERBOSE(LOG_TAG, "Exit");
}

ContextManager::~ContextManager()
{
    PAL_VERBOSE(LOG_TAG, "Enter");
    PAL_VERBOSE(LOG_TAG, "Exit");
}

int32_t ContextManager::Init()
{
    int32_t rc = 0;
    PAL_VERBOSE(LOG_TAG, "Enter");

    rc = OpenAndStartProxyStream();
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to OpenAndStartProxyStream", rc);
        goto exit;
    }

    rc = CreateCommandProcessingThread();
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to CreateCommandProcessingThread", rc);
        goto stop_and_close_proxy_stream;
    }

    goto exit;

stop_and_close_proxy_stream:
    StopAndCloseProxyStream();

exit:
    PAL_VERBOSE(LOG_TAG, "Exit");
    return rc;
}

void ContextManager::DeInit()
{
    PAL_VERBOSE(LOG_TAG, "Enter");

    CloseAll();
    StopAndCloseProxyStream();
    DestroyCommandProcessingThread();

    PAL_VERBOSE(LOG_TAG, "Exit");
}

void ContextManager::CloseAll()
{
    int32_t rc = 0;
    std::map<uint32_t, see_client*>::iterator it_see_client;
    see_client *see = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter");
    for (auto it_see_client = this->see_clients.begin(); it_see_client != this->see_clients.cend();) {
        see = it_see_client->second;
        PAL_VERBOSE(LOG_TAG, "Calling CloseAllUsecases for see_client:%d", see->Get_SEE_ID());
        see->CloseAllUsecases();
        see_clients.erase(it_see_client++);
    }

    PAL_VERBOSE(LOG_TAG, "Exit");
}


int32_t ContextManager::OpenAndStartProxyStream()
{
    int32_t rc = 0;
    struct pal_stream_attributes stream_attr;

    PAL_VERBOSE(LOG_TAG, "Enter");

    memset(&stream_attr, 0, sizeof(struct pal_stream_attributes));
    stream_attr.type = PAL_STREAM_CONTEXT_PROXY;

    rc = pal_stream_open(&stream_attr, 0, NULL, 0, NULL, NULL, 0, &this->proxy_stream);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to open() proxy stream", rc);
        goto exit;
    }

    rc = pal_stream_start(this->proxy_stream);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to start() proxy stream", rc);
        goto close_stream;
    }

    goto exit;

close_stream:
    pal_stream_close(this->proxy_stream);;
    this->proxy_stream = NULL;

exit:
    PAL_VERBOSE(LOG_TAG, "Exit rc:", rc);
    return rc;
}

int32_t ContextManager::StopAndCloseProxyStream()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter");

    if (this->proxy_stream) {
        rc = pal_stream_stop(this->proxy_stream);
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d Failed to stop() proxy stream", rc);
        }

        rc = pal_stream_close(this->proxy_stream);
        if (rc) {
            PAL_ERR(LOG_TAG, "Error:%d Failed to close() proxy stream", rc);
        }

        this->proxy_stream = NULL;
    }

    PAL_VERBOSE(LOG_TAG, "Exit rc:", rc);
    return rc;
}

void ContextManager::CommandThreadRunner(ContextManager& cm) {

    while (!cm.exit_cmd_thread_) {
    }
    PAL_VERBOSE(LOG_TAG, "Exiting  CommandThreadRunner:");
}

int32_t ContextManager::CreateCommandProcessingThread()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter");

    exit_cmd_thread_ = false;
    cmd_thread_ = std::thread(CommandThreadRunner, std::ref(*this));

    PAL_VERBOSE(LOG_TAG, "Exit rc:", rc);
    return rc;
}

void ContextManager::DestroyCommandProcessingThread()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter");

    exit_cmd_thread_ = true;

    if (cmd_thread_.joinable()) {
        PAL_DBG(LOG_TAG, "Join cmd_thread_ thread");
        cmd_thread_.join();
    }

    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);
}

see_client::see_client(uint32_t id)
{
    PAL_VERBOSE(LOG_TAG, "Enter seeid:%d", id);

    this->see_id = id;

    PAL_VERBOSE(LOG_TAG, "Exit ");
}

see_client::~see_client()
{
    PAL_VERBOSE(LOG_TAG, "Enter seeid:%d", this->see_id);

    // ACM should close all active usecases before calling destructor.
    usecases.clear();

    PAL_VERBOSE(LOG_TAG, "Exit ");
}

see_client* ContextManager::SEE_Client_CreateIf_And_Get(uint32_t see_id)
{
    std::map<uint32_t, see_client*>::iterator it;
    see_client* client = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter seeid:%d", see_id);

    it = see_clients.find(see_id);
    if (it != see_clients.end()) {
        client = it->second;
    } else {
        client = new see_client(see_id);
        see_clients.insert(std::make_pair(see_id, client));
    }

    PAL_VERBOSE(LOG_TAG, "Exit");

    return client;
}

see_client* ContextManager::SEE_Client_Get_Existing(uint32_t see_id)
{
    std::map<uint32_t, see_client*>::iterator it;
    see_client* client = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter seeid:%d", see_id);

    it = see_clients.find(see_id);
    if (it != see_clients.end()) {
        client = it->second;
    }
    else {
        PAL_ERR(LOG_TAG, "Error:%d see_id:%d doesnt exist", -EINVAL, see_id);
    }

    PAL_VERBOSE(LOG_TAG, "Exit");
    return client;
}

uint32_t see_client::Get_SEE_ID()
{
    return this->see_id;
}

Usecase* see_client::Usecase_Get(uint32_t usecase_id)
{
    std::map<uint32_t, Usecase*>::iterator it;
    Usecase* uc = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter usecase_id:%d", usecase_id);

    it = usecases.find(usecase_id);
    if (it != usecases.end()) {
        uc = it->second;
    }

    PAL_VERBOSE(LOG_TAG, "Exit");
    return uc;
}

int32_t see_client::Usecase_Remove(uint32_t usecase_id)
{
    int32_t rc = 0;
    std::map<uint32_t, Usecase*>::iterator it;
    Usecase* uc = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter usecase_id:%d", usecase_id);

    it = usecases.find(usecase_id);
    if (it != usecases.end()) {
        PAL_VERBOSE(LOG_TAG, "Found usecase_id:%d", usecase_id);
        uc = it->second;

        //remove from usecase map
        usecases.erase(it);

        //delete uc, expectation is that the UC has been stopped and closed.
        if (uc)
            delete uc;
    }
    else {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d Cannot find Found usecase_id:%d", rc, usecase_id);
    }

    PAL_VERBOSE(LOG_TAG, "Exit: rc:%d", rc);

    return rc;
}

int32_t see_client::Usecase_Add(uint32_t usecase_id, Usecase* uc)
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter: usecase_id:%d", usecase_id);

    usecases.insert(std::make_pair(usecase_id, uc));

    PAL_VERBOSE(LOG_TAG, "Exit: rc:%d", rc);

    return rc;
}

void see_client::CloseAllUsecases()
{
    std::map<uint32_t, Usecase*>::iterator it_uc;
    Usecase *uc = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter:");

    for (auto it_uc = this->usecases.begin(); it_uc != this->usecases.cend();) {
        uc = it_uc->second;
        PAL_VERBOSE(LOG_TAG, "Calling StopAndClose on usecase_id:%d", uc->GetUseCaseID());
        uc->StopAndClose();
        usecases.erase(it_uc++);
        delete uc;
    }

    PAL_VERBOSE(LOG_TAG, "Exit:");
}

Usecase* UsecaseFactory::UsecaseCreate(int32_t usecase_id)
{
    Usecase* ret_usecase = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter usecase_id:%d", usecase_id);

    switch (usecase_id) {
    case USECASE_ACD:
        ret_usecase = new UsecaseACD(usecase_id);
        break;
    case USECASE_RAW_DATA:
        ret_usecase = new UsecaseRawData(usecase_id);
        break;
    case USECASE_UPD:
        ret_usecase = new UsecaseUPD(usecase_id);
        break;
    default:
        ret_usecase = NULL;
        PAL_ERR(LOG_TAG, "Error:%d Invalid usecaseid:%d", -EINVAL, usecase_id);
        break;
    }

    PAL_VERBOSE(LOG_TAG, "Exit:");
    return ret_usecase;
}

Usecase::Usecase(uint32_t usecase_id)
{
    PAL_VERBOSE(LOG_TAG, "Enter");

    this->usecase_id = usecase_id;
    this->stream_attributes = (struct pal_stream_attributes *) calloc (1, sizeof(struct pal_stream_attributes));

    PAL_VERBOSE(LOG_TAG, "Exit");
}

Usecase::~Usecase()
{
    PAL_VERBOSE(LOG_TAG, "Enter");

    free(this->stream_attributes);
    this->stream_attributes = NULL;

    free(this->pal_devices);
    this->pal_devices = NULL;

    this->usecase_id = 0;
    this->no_of_devices = 0;

    PAL_VERBOSE(LOG_TAG, "Exit");
}

uint32_t Usecase::GetUseCaseID()
{
    return this->usecase_id;
}

int32_t Usecase::Open()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);

    rc = pal_stream_open(this->stream_attributes, this->no_of_devices, this->pal_devices, 0, NULL, NULL, 0, &(this->pal_stream));
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to Open() usecase:%d", rc, this->usecase_id);
    }

    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);

    return rc;
}

int32_t Usecase::Configure()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);

    return rc;
}

int32_t Usecase::Start()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);

    rc = pal_stream_start(this->pal_stream);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to Start() usecase:%d", rc, this->usecase_id);
    }

    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);

    return rc;
}

int32_t Usecase::StopAndClose()
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);

    rc = pal_stream_stop(this->pal_stream);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to stop() usecase:%d", rc, this->usecase_id);
    }

    rc = pal_stream_close(this->pal_stream);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d Failed to Close() usecase:%d", rc, this->usecase_id);
    }

    rc = 0;
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);

    return rc;
}

int32_t Usecase::SetUseCaseData(uint32_t size, void *data)
{
    int32_t rc = 0;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);

    return rc;
}

UsecaseACD::UsecaseACD(uint32_t usecase_id) : Usecase(usecase_id)
{
    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", usecase_id);

    this->context_list = NULL;
    this->stream_attributes->type = PAL_STREAM_ACD;
    this->no_of_devices = 1;
    this->pal_devices = (struct pal_device *) calloc (this->no_of_devices, sizeof(struct pal_device));

    //input device
    this->pal_devices[0].id = PAL_DEVICE_IN_HANDSET_VA_MIC;
    this->pal_devices[0].config.bit_width = 16;
    this->pal_devices[0].config.sample_rate = 16000;
    this->pal_devices[0].config.ch_info.channels = 1;

    PAL_VERBOSE(LOG_TAG, "Exit ");
}

UsecaseACD::~UsecaseACD()
{
    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);
    if (this->context_list) {
        free(this->context_list);
        this->context_list = NULL;
    }
    PAL_VERBOSE(LOG_TAG, "Exit ");
}

int32_t UsecaseACD::Configure()
{
    int32_t rc = 0;
    pal_param_payload *context_payload = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);

    context_payload = (pal_param_payload *)calloc(1, sizeof(pal_param_payload) + sizeof(this->context_list));
    context_payload->payload_size = sizeof(this->context_list);
    memcpy(context_payload->payload, this->context_list, sizeof(this->context_list));
    rc = pal_stream_set_param(this->pal_stream, PAL_PARAM_ID_CONTEXT_LIST, context_payload);
    if (rc) {
        PAL_ERR(LOG_TAG, "Error:%d setting parameters to stream usecase:%d", rc, this->usecase_id);
        goto exit;
    }

exit:
    if (context_payload)
        free(context_payload);

    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);
    return rc;
}

int32_t UsecaseACD::SetUseCaseData(uint32_t size, void *data)
{
    int rc = 0;
    int no_contexts = 0;
    uint32_t *ptr = NULL;

    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d, size:%d", this->usecase_id, size);

    if (!size || !data) {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d Invalid size:%d or data:%p for usecase:%d", rc, size, data, this->usecase_id);
        goto exit;
    }

    if (this->context_list) {
        free(this->context_list);
        this->context_list = NULL;
    }

    ptr = (uint32_t *)data;
    no_contexts = (uint32_t)ptr[0];
    PAL_VERBOSE(LOG_TAG, "Number of contexts:%d for usecase:%d, size:%d", no_contexts, this->usecase_id);

    if (size < (sizeof(uint32_t) + (no_contexts * sizeof(uint32_t)))) {
        rc = -EINVAL;
        PAL_ERR(LOG_TAG, "Error:%d Insufficient data for no_contexts:%d and usecase:%d", rc, no_contexts, this->usecase_id);
        goto exit;
    }
    this->context_list = (struct pal_param_context_list *)calloc(size, sizeof(uint8_t));
    if (!this->context_list) {
        rc = -ENOMEM;
        PAL_ERR(LOG_TAG, "Error:%d Failed to allocate memory for context_list", rc);
    }
    memcpy(this->context_list, data, size);

exit:
    PAL_VERBOSE(LOG_TAG, "Exit rc:%d", rc);
    return rc;
}

UsecaseRawData::UsecaseRawData(uint32_t usecase_id) : Usecase(usecase_id)
{
    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", usecase_id);

    this->stream_attributes->type = PAL_STREAM_CONTEXT_RAWDATA;
    this->no_of_devices = 1;
    this->pal_devices = (struct pal_device *) calloc (this->no_of_devices, sizeof(struct pal_device));

    //input device
    this->pal_devices[0].id = PAL_DEVICE_IN_HANDSET_VA_MIC;
    this->pal_devices[0].config.bit_width = 16;
    this->pal_devices[0].config.sample_rate = 16000;
    this->pal_devices[0].config.ch_info.channels = 1;

    PAL_VERBOSE(LOG_TAG, "Exit");
}

UsecaseRawData::~UsecaseRawData()
{
    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);
    //cleanup is done in baseclass
    PAL_VERBOSE(LOG_TAG, "Exit");
}

UsecaseUPD::UsecaseUPD(uint32_t usecase_id) : Usecase(usecase_id)
{
    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", usecase_id);

    this->stream_attributes->type = PAL_STREAM_ULTRASOUND;
    this->no_of_devices = 2;
    this->pal_devices = (struct pal_device *) calloc (this->no_of_devices, sizeof(struct pal_device));

    //output device
    this->pal_devices[0].id = PAL_DEVICE_OUT_ULTRASOUND;
    this->pal_devices[0].config.bit_width = 16;
    this->pal_devices[0].config.sample_rate = 96000;
    this->pal_devices[0].config.ch_info.channels = 1;

    //input device
    this->pal_devices[0].id = PAL_DEVICE_IN_ULTRASOUND_MIC;
    this->pal_devices[0].config.bit_width = 16;
    this->pal_devices[0].config.sample_rate = 96000;
    this->pal_devices[0].config.ch_info.channels = 1;

    PAL_VERBOSE(LOG_TAG, "Exit");
}

UsecaseUPD::~UsecaseUPD()
{
    PAL_VERBOSE(LOG_TAG, "Enter usecase:%d", this->usecase_id);
    //cleanup is done in baseclass
    PAL_VERBOSE(LOG_TAG, "Exit");
}
