/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * instrumentation.c
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * This software is licensed under the 3-Clause BSD License.
 *
 * Copyright (C) 2018-2023 - KIOXIA Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "instrumentation.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/unistd.h>
#include <unistd.h>

#include "../config.h"
#include "../log-manager.h"
#include "../sef-utils.h"
#include "sef-event.h"
#include "sef-slist.h"    // for container_of

const char* INSTRUMENTATION_DEFAULT_PATH = "/tmp/SEFFTLDomain";
#if defined(__APPLE__) || defined(__NetBSD__)
#define MSG_CONFIRM 0
#endif

#include <inttypes.h>
#include <pthread.h>
#include <strings.h>

#include "dlist.h"

struct INSHandle_
{
    // state used to register/invoke actions
    TmaDList actionList;
    pthread_mutex_t actionListLock;

    TmaDList stateList;
    TmaDList ioList;
    pthread_mutex_t counterLock;

    // state used by thread so user can externally invoke
    pthread_t thread;
    LogHandle logHandle;    // handle for logging
    int serverSock;
    int ttd;
};

struct ActionEntry
{
    TmaDListEntry link;
    char* verb;    // strdup() so req free
    INSAction action;
    void* arg;
    INSAction pendingAction;    // what to set action to
    void* pendingArg;           // what to set arg to
    int active;                 // Set if action() is in call
    int pending;                // set if setAction called while in action().
};

struct CounterEntry
{
    int instance;
    int counterNum;
    struct INSCounter* counters;
    void* arg;
    INSCounterUpdate updateFunc;
    TmaDListEntry link;
    INSCounterSet setFunc;
};

static void defaultAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    snprintf(out, size, "Unknown Command '%s'\n", verb);
}

static struct ActionEntry* defaultEntry()
{
    static struct ActionEntry action = {.action = defaultAction};
    return &action;
}

static struct ActionEntry* searchAction(const char* verb, TmaDList* actionList)
{
    struct ActionEntry* entry = NULL;
    while ((entry = utl_DListNextAs(actionList, entry, struct ActionEntry, link)))
    {
        if (strcasecmp(verb, entry->verb) < 0)
        {
            break;
        }
    }
    return entry;
}

static struct ActionEntry* findAction(const char* verb, TmaDList* actionList)
{
    struct ActionEntry* entry = NULL;
    while ((entry = utl_DListNextAs(actionList, entry, struct ActionEntry, link)))
    {
        if (strcasecmp(verb, entry->verb) == 0)
        {
            break;
        }
    }
    return entry ?: defaultEntry();
}

static void freeActionEntry(struct ActionEntry* entry)
{
    if (entry)
    {
        SUfree(entry->verb);
        SUfree(entry);
    }
}

static void invokeAction(INSHandle ictxt,
                         const char* key,
                         const char* verb,
                         char** savePtr,
                         char* outgoingBuf,
                         ssize_t outgoingBufSize)
{
    struct ActionEntry* entry;

    pthread_mutex_lock(&ictxt->actionListLock);
    {
        entry = findAction(key ?: "", &ictxt->actionList);
        entry->active = 1;
    }
    pthread_mutex_unlock(&ictxt->actionListLock);

    entry->action(verb, savePtr, entry->arg, outgoingBuf, outgoingBufSize);

    // Handle pending update that came in while this action was active.
    pthread_mutex_lock(&ictxt->actionListLock);
    {
        entry->active = 0;
        if (entry->pending)
        {
            entry->arg = entry->pendingArg;
            entry->action = entry->pendingAction;
            entry->pending = 0;
            if (!entry->action)
            {
                utl_DListRemove(&entry->link);
                freeActionEntry(entry);
            }
        }
    }
    pthread_mutex_unlock(&ictxt->actionListLock);
}

void INSInvokeAction(INSHandle hInst, const char* verbStr, char* out, ssize_t size)
{
    if (hInst)
    {
        char* inStr = strdup(verbStr);
        char* savePtr = NULL;
        char* verb;
        char* p;

        // remove trailing newline
        p = strchr(inStr, '\n');
        if (p)
        {
            p[0] = '\0';
        }
        verb = strtok_r(inStr, " \t", &savePtr);
        invokeAction(hInst, verb, verb, &savePtr, out, size);
        SUfree(inStr);
    }
}

// GCC 10 throws a false positive stack-buffer-overflow error when
// pthread_cancel quits the recvfrom with MSG_WAITALL
#if defined(__GNUC__) && __GNUC__ >= 10
static __attribute__((no_sanitize("address"))) void* InstrumentationThread(void* arg)
#else
static void* InstrumentationThread(void* arg)
#endif    // __GNUC__
{
    struct sockaddr_un clientSockAddr;
    socklen_t clientLen;
    int rc, bytesRecv = 0;
    char readBuf[256], writeBuf[2048];
    struct INSHandle_* ictxt = arg;

    clientLen = sizeof(clientSockAddr);

    while (!ictxt->ttd)
    {
        // receive data from the client
        bzero(readBuf, sizeof(readBuf));

        bytesRecv = recvfrom(ictxt->serverSock, readBuf, sizeof(readBuf), MSG_WAITALL,
                             (struct sockaddr*)&clientSockAddr, &clientLen);

        if (bytesRecv == -1)
        {
            LogError(ictxt->logHandle, "Recvfrom Error; (%d) - %s", errno, strerror(errno));
        }
        else
        {
            // pthread cancellation is disabled to ensure proper response
            pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

            readBuf[sizeof(readBuf) - 1] = '\0';
            writeBuf[0] = '\0';
            INSInvokeAction(ictxt, readBuf, writeBuf, sizeof(writeBuf));

            // add a newline
            char* p = strchr(writeBuf, '\0');
            if (p == writeBuf || p[-1] != '\n')
            {
                if (p - writeBuf - 1 < sizeof(writeBuf))
                {
                    strcpy(p++, "\n");
                }
                else if (sizeof(writeBuf) >= 2)
                {
                    p[-1] = '\n';
                }
            }

            // send data to the requester
            rc = sendto(ictxt->serverSock, (const char*)writeBuf, p - writeBuf, MSG_CONFIRM,
                        (const struct sockaddr*)&clientSockAddr, clientLen);
            if (rc == -1)
            {
                LogError(ictxt->logHandle, "Sendto Error; (%d) - %s", errno, strerror(errno));
            }

            pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        }
    }

    return 0;
}

void INSRegisterAction(INSHandle ictxt, const char* verb, INSAction action, void* arg)
{
    if (ictxt == NULL)
    {
        return;
    }

    pthread_mutex_lock(&ictxt->actionListLock);
    struct ActionEntry* entry = findAction(verb, &ictxt->actionList);
    if (entry->active)
    {
        entry->pending = 1;
        entry->pendingArg = arg;
        entry->pendingAction = action;
        entry = NULL;
    }
    pthread_mutex_unlock(&ictxt->actionListLock);
    if (entry == NULL)
    {
        return;
    }
    if (entry != defaultEntry())
    {
        utl_DListRemove(&entry->link);
    }
    else
    {
        entry = SUzalloc(sizeof(*entry));
        utl_DListInitEntry(&entry->link);
        entry->verb = strdup(verb);
    }
    entry->action = action;
    entry->arg = arg;
    if (action)
    {
        struct ActionEntry* node;

        pthread_mutex_lock(&ictxt->actionListLock);
        node = searchAction(entry->verb, &ictxt->actionList);
        if (node)
        {
            utl_DListInsertBefore(&node->link, &entry->link);
        }
        else
        {
            utl_DListPushTail(&ictxt->actionList, &entry->link);
        }
        pthread_mutex_unlock(&ictxt->actionListLock);
    }
    else
    {
        freeActionEntry(entry);
    }
}

static void getLogAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    char logName[10];
    LogHandle logHandle = arg;

    if (strcasecmp(verb, "help") == 0)
    {
        snprintf(out, size, "* getlog: returns log level\n");
        return;
    }

    LogGetLevelName(logHandle, logName, sizeof(logName));
    snprintf(out, size, "Log Level: %i (%s)\n", LogGetLevel(logHandle), logName);
}

static void setLogAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    char logName[10];
    int newLogLevel, oldLogLevel;
    LogHandle logHandle = arg;

    if (strcasecmp(verb, "help") == 0)
    {
        snprintf(out, size,
                 "* setlog INT: sets the log level | possible values are 0 to 5\n"
                 "* setlog ENUM: sets the log level | possible values are \"trace\", \"debug\", "
                 "\"info\", \"error\", "
                 "\"fatal\", \"nolog\"\n");
        return;
    }

    oldLogLevel = LogGetLevel(logHandle);

    if (sscanf(*savePtr, "%d", &newLogLevel) != 0)
    {
        LogSetLevel(logHandle, newLogLevel);
    }
    else
    {
        LogSetLevelByName(logHandle, *savePtr);
    }

    if (LogGetLevel(logHandle) != oldLogLevel)
    {
        LogGetLevelName(logHandle, logName, sizeof(logName));
        snprintf(out, size, "Log Level Set To: %d (%s)\n", LogGetLevel(logHandle), logName);
    }
    else
    {
        snprintf(out, size, "Log Level not updated\n");
    }
}

static void helpAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    char* key = strdup("");
    ssize_t remaining = size;
    struct INSHandle_* ictxt = arg;

    remaining -= snprintf(out, remaining, "Available Commands:\n");

    while (remaining > 0)
    {
        pthread_mutex_lock(&ictxt->actionListLock);
        struct ActionEntry* node = searchAction(key, &ictxt->actionList);
        if (node)
        {
            SUfree(key);
            key = strdup(node->verb);
        }
        pthread_mutex_unlock(&ictxt->actionListLock);
        if (node == NULL)
        {
            break;
        }
        // skip the help action!
        if (strcasecmp(node->verb, "help"))
        {
            char* savePtr = NULL;
            char input[256];    // todo: limit verb size on registration
            snprintf(input, sizeof(input), "help %s", key);
            char* verb = strtok_r(input, " ", &savePtr);
            invokeAction(ictxt, key, verb, &savePtr, &out[size - remaining], remaining);
            remaining -= strlen(&out[size - remaining]);
        }
        if (*key)
        {
            strchr(key, '\0')[-1]++;    // todo: won't work for '\xff' so reject if verb ends in 0xff
        }
    }
    SUfree(key);
    if (remaining > 0)
    {
        snprintf(&out[size - remaining], remaining, "* help: For help\n");
    }
}

static void counterAction(TmaDList* list, bool isJson, bool isInfo, char* out, ssize_t size)
{
    int i;
    int _1stValue = true;
    struct CounterEntry* entry = NULL;
    ssize_t loc = 0, remaining = size;
    const char* valStr = isJson ? "\"%s\": %" PRId64 : "%s = %" PRId64;

    if (isInfo)
    {
        int maxNameLength = 7;

        while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
        {
            for (i = 0; i < entry->counterNum; i++)
            {
                if (maxNameLength < strlen(entry->counters[i].name))
                {
                    maxNameLength = strlen(entry->counters[i].name) + 3;
                }
            }
        }

        remaining -= snprintf(out, remaining, "%-*s %s\n", maxNameLength, "Name", "Description");
        while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
        {
            for (i = 0; i < entry->counterNum; i++)
            {
                int outputSize = snprintf(NULL, 0, "%-*s %s\n", maxNameLength,
                                          entry->counters[i].name, entry->counters[i].description);

                if (remaining - outputSize > 0)
                {
                    remaining -= snprintf(&out[size - remaining], remaining, "%-*s %s\n", maxNameLength,
                                          entry->counters[i].name, entry->counters[i].description);
                }
            }
        }

        return;
    }

    if (isJson)
    {
        loc += snprintf(out, remaining, "{");
        remaining -= 2;    // for closing curly brackets
    }

    while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
    {
        // update value
        if (entry->updateFunc != NULL)
        {
            entry->updateFunc(entry->arg);
        }

        for (i = 0; i < entry->counterNum; i++)
        {
            int outputSize;

            outputSize = snprintf(NULL, 0, valStr, entry->counters[i].name, *entry->counters[i].value);
            outputSize += isJson && _1stValue ? 0 : 1;

            if (remaining - outputSize > 0)
            {
                if (isJson && !_1stValue)
                {
                    loc += snprintf(&out[loc], remaining, ",");
                }
                loc += snprintf(&out[loc], remaining, valStr, entry->counters[i].name,
                                *entry->counters[i].value);
                if (!isJson)
                {
                    loc += snprintf(&out[loc], remaining, "\n");
                }
                _1stValue = false;
                remaining -= outputSize;
            }
        }
    }

    if (isJson)
    {
        snprintf(&out[loc], remaining, "}");
    }
}

static int counterSet(TmaDList* list, const char* name, int64_t val)
{
    int i;
    struct CounterEntry* entry = NULL;

    while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
    {
        for (i = 0; i < entry->counterNum; i++)
        {
            if (strcmp(name, entry->counters[i].name) == 0)
            {
                if (!entry->setFunc || entry->setFunc(entry->arg, i, val))
                {
                    return ENOTSUP;
                }
                return 0;
            }
        }
    }
    return EINVAL;
}

static void dumpAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    TmaDList* list;
    struct INSHandle_* ictxt = arg;
    bool isJson = false, isReset = false, isInfo = false;
    char* how = strtok_r(NULL, " \t", savePtr);

    if (strcmp(verb, "help") == 0)
    {
        snprintf(out, size, "* dump [info][json][reset]: dump all instrumentation counters\n");
        return;
    }

    while (how)
    {
        if (strcasecmp(how, "json") == 0)
        {
            isJson = true;
        }
        else if (strcasecmp(how, "reset") == 0)
        {
            isReset = true;
        }
        else if (strcasecmp(how, "info") == 0 || strcasecmp(how, "help") == 0)
        {
            isInfo = true;
        }
        else
        {
            snprintf(out, size, "Unexpected argument '%s'\n", how);
            return;
        }
        how = strtok_r(NULL, " \t", savePtr);
    }

    list = &ictxt->ioList;

    pthread_mutex_lock(&ictxt->actionListLock);
    counterAction(list, isJson, isInfo, out, size);
    pthread_mutex_unlock(&ictxt->actionListLock);

    // reset values
    if (isReset)
    {
        int i;
        struct CounterEntry* entry = NULL;

        pthread_mutex_lock(&ictxt->actionListLock);
        while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
        {
            for (i = 0; i < entry->counterNum; i++)
            {
                atomic_store(entry->counters[i].value, 0);
            }
        }
        pthread_mutex_unlock(&ictxt->actionListLock);
    }
}

static void stateAction(const char* verb, char** savePtr, void* arg, char* out, ssize_t size)
{
    struct INSHandle_* ictxt = arg;
    bool isJson = false, isInfo = false;
    char* how = strtok_r(NULL, " \t", savePtr);

    if (strcasecmp(verb, "help") == 0)
    {
        snprintf(out, size,
                 "* state [json][info]: dump all io state data\n"
                 "* state set <counter> <value>: set the value\n");
        return;
    }

    while (how)
    {
        if (strcasecmp(how, "json") == 0)
        {
            isJson = true;
        }
        else if (strcasecmp(how, "info") == 0 || strcasecmp(how, "help") == 0)
        {
            isInfo = true;
        }
        else if (strcasecmp(how, "set") == 0)
        {
            char* name = strtok_r(NULL, " \t", savePtr);
            char* val = strtok_r(NULL, " \t", savePtr);
            int ret;

            if (!name || !val)
            {
                snprintf(out, size, "set usage: set <counter_name> <value>\n");
                return;
            }
            pthread_mutex_lock(&ictxt->actionListLock);
            ret = counterSet(&ictxt->stateList, name, strtoll(val, NULL, 0));
            pthread_mutex_unlock(&ictxt->actionListLock);
            if (ret == EINVAL)
            {
                snprintf(out, size, "Unknown counter '%s'\n", name);
            }
            else if (ret == ENOTSUP)
            {
                snprintf(out, size, "Counter '%s' is not setable\n", name);
            }
            else
            {
                snprintf(out, size, "Counter '%s' set\n", name);
            }
            return;
        }
        else
        {
            snprintf(out, size, "Unexpected argument '%s'\n", how);
            return;
        }
        how = strtok_r(NULL, " \t", savePtr);
    }

    pthread_mutex_lock(&ictxt->actionListLock);
    counterAction(&ictxt->stateList, isJson, isInfo, out, size);
    pthread_mutex_unlock(&ictxt->actionListLock);
}

static void freeCounterEntry(struct CounterEntry* entry)
{
    int i;

    for (i = 0; i < entry->counterNum; i++)
    {
        SUfree(entry->counters[i].name);
        SUfree(entry->counters[i].description);
    }

    SUfree(entry->counters);
    SUfree(entry);
}

static void cleanupCounters(struct INSHandle_* ictxt)
{
    int l;
    TmaDList* list;
    struct CounterEntry* entry = NULL;

    for (l = 0; l < 2; l++)
    {
        if (l == 0)
        {
            list = &ictxt->ioList;
        }
        else
        {
            list = &ictxt->stateList;
        }

        while ((entry = utl_DListPopHeadAs(list, struct CounterEntry, link)))
        {
            freeCounterEntry(entry);
        }
    }
}

int INSInit(const char* socketPath, INSHandle* hInst, LogHandle logHandle)
{
    struct sockaddr_un serverSockAddr;
    socklen_t serverLen;
    int rc;

    *hInst = NULL;

    if (socketPath == NULL)
    {
        return -EINVAL;
    }

    struct INSHandle_* ictxt = SUmalloc(sizeof(*ictxt));
    if (ictxt == NULL)
    {
        return -ENOMEM;
    }

    // init handle
    ictxt->ttd = 0;
    ictxt->serverSock = -1;
    ictxt->logHandle = logHandle;
    utl_DListInit(&ictxt->actionList);
    utl_DListInit(&ictxt->stateList);
    utl_DListInit(&ictxt->ioList);
    pthread_mutex_init(&ictxt->actionListLock, NULL);
    pthread_mutex_init(&ictxt->counterLock, NULL);

    // Create a UNIX domain stream socket
    ictxt->serverSock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (ictxt->serverSock == -1)
    {
        LogFatal(ictxt->logHandle, "Socket Error; (%d) - %s", errno, strerror(errno));
        return -ESOCKTNOSUPPORT;
    }

    // Create a UNIX domain stream socket
    ictxt->serverSock = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (ictxt->serverSock == -1)
    {
        LogFatal(ictxt->logHandle, "Socket Error; (%d) - %s", errno, strerror(errno));
        return -ESOCKTNOSUPPORT;
    }

    // delete file
    unlink(socketPath);

    // prepare UNIX Domain
    serverSockAddr.sun_family = AF_UNIX;
    strcpy(serverSockAddr.sun_path, socketPath);
    serverLen = sizeof(serverSockAddr);

    // bind UNIX Domain
    rc = bind(ictxt->serverSock, (struct sockaddr*)&serverSockAddr, serverLen);
    if (rc < 0)
    {
        LogFatal(ictxt->logHandle, "Bind Error; (%d) - %s", errno, strerror(errno));
        close(ictxt->serverSock);
        return -ESOCKTNOSUPPORT;
    }

    // init the instrumentation
    if (pthread_create(&ictxt->thread, NULL, InstrumentationThread, ictxt))
    {
        LogFatal(ictxt->logHandle, "Error creating instrumentation thread (%d) - %s", errno,
                 strerror(errno));
        pthread_mutex_destroy(&ictxt->actionListLock);
        pthread_mutex_destroy(&ictxt->counterLock);
        SUfree(ictxt);
        return -errno;
    }

    // register local actions
    INSRegisterAction(ictxt, "help", helpAction, ictxt);
    INSRegisterAction(ictxt, "dump", dumpAction, ictxt);
    INSRegisterAction(ictxt, "state", stateAction, ictxt);
    INSRegisterAction(ictxt, "getlog", getLogAction, ictxt->logHandle);
    INSRegisterAction(ictxt, "setlog", setLogAction, ictxt->logHandle);

    LogDebug(ictxt->logHandle, "instrumentation thread was init");
    *hInst = ictxt;
    return 0;
}

void INSCleanup(INSHandle* hInst)
{
    struct INSHandle_* ictxt = *hInst;

    if (ictxt == NULL)
    {
        return;
    }
    // cancel and join the instrumentation thread
    ictxt->ttd = 1;
    pthread_cancel(ictxt->thread);
    if (pthread_join(ictxt->thread, NULL))
    {
        LogError(ictxt->logHandle, "Unable to join instrumentation thread; (%d) - %s", errno,
                 strerror(errno));
    }

    // close socket
    if (ictxt->serverSock != -1)
    {
        close(ictxt->serverSock);
    }

    // remove all counters
    cleanupCounters(ictxt);

    // remove instrumentation's actions
    INSUnRegisterAction(ictxt, "getlog");
    INSUnRegisterAction(ictxt, "setlog");
    INSUnRegisterAction(ictxt, "help");
    INSUnRegisterAction(ictxt, "dump");
    INSUnRegisterAction(ictxt, "state");

    // pickup anything leaked
    struct ActionEntry* entry = NULL;
    while ((entry = utl_DListGetHeadAs(&ictxt->actionList, struct ActionEntry, link)))
    {
        INSUnRegisterAction(ictxt, entry->verb);
        entry = NULL;
    }

    SUfree(ictxt);
    *hInst = NULL;
}

int registerCounters(TmaDList* list,
                     pthread_mutex_t* ListLock,
                     struct INSCounter* counters,
                     int counterNum,
                     void* arg,
                     INSCounterUpdate updateFunc,
                     INSCounterSet setFunc)
{
    int i;
    struct CounterEntry* entry = NULL;
    static int cntInstance;
    int instance;

    // check counter already there
    pthread_mutex_lock(ListLock);
    while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
    {
        int i, j;

        for (i = 0; i < entry->counterNum; i++)
        {
            for (j = 0; j < counterNum; j++)
            {
                if (strcasecmp(entry->counters[i].name, counters[j].name) == 0)
                {
                    pthread_mutex_unlock(ListLock);
                    return -EEXIST;
                }
            }
        }
    }

    instance = ++cntInstance;
    if (instance == INT_MAX)
    {
        cntInstance = 0;
    }
    pthread_mutex_unlock(ListLock);

    // create new counters entry
    entry = SUzalloc(sizeof(struct CounterEntry));
    utl_DListInitEntry(&entry->link);
    entry->instance = instance;
    entry->counters = SUmalloc(sizeof(struct INSCounter) * counterNum);
    entry->counterNum = counterNum;
    entry->arg = arg;
    entry->updateFunc = updateFunc;
    entry->setFunc = setFunc;

    for (i = 0; i < counterNum; i++)
    {
        entry->counters[i].name = strdup(counters[i].name);
        entry->counters[i].description = strdup(counters[i].description);
        entry->counters[i].value = counters[i].value;
    }

    // add counters entry
    pthread_mutex_lock(ListLock);
    utl_DListPushHead(list, &entry->link);
    pthread_mutex_unlock(ListLock);

    return instance;
}

int unregisterCounters(TmaDList* list, pthread_mutex_t* ListLock, int instanceId)
{
    struct CounterEntry* entry = NULL;

    pthread_mutex_lock(ListLock);
    while ((entry = utl_DListNextAs(list, entry, struct CounterEntry, link)))
    {
        if (entry->instance == instanceId)
        {
            utl_DListRemove(&entry->link);
            freeCounterEntry(entry);
            break;
        }
    }
    pthread_mutex_unlock(ListLock);
    return entry ? 0 : -ENOENT;
}

int INSRegisterIoCounters(INSHandle ictxt,
                          struct INSCounter* counters,
                          int counterNum,
                          void* arg,
                          INSCounterUpdate updateFunc)
{
    TmaDList* list;

    if (ictxt == NULL)
    {
        return -EINVAL;
    }

    list = &ictxt->ioList;
    return registerCounters(list, &ictxt->counterLock, counters, counterNum, arg, updateFunc, NULL);
}

int INSRegisterStateCounters(INSHandle ictxt,
                             struct INSCounter* counters,
                             int counterNum,
                             void* arg,
                             INSCounterUpdate updateFunc,
                             INSCounterSet setFunc)
{
    TmaDList* list;

    if (ictxt == NULL)
    {
        return -EINVAL;
    }

    list = &ictxt->stateList;
    return registerCounters(list, &ictxt->counterLock, counters, counterNum, arg, updateFunc, setFunc);
}

int INSUnRegisterIoCounters(INSHandle ictxt, int instanceId)
{
    if (ictxt == NULL)
    {
        return -EINVAL;
    }

    return unregisterCounters(&ictxt->ioList, &ictxt->counterLock, instanceId);
}

int INSUnRegisterStateCounters(INSHandle ictxt, int instanceId)
{
    if (ictxt == NULL)
    {
        return -EINVAL;
    }

    return unregisterCounters(&ictxt->stateList, &ictxt->counterLock, instanceId);
}
