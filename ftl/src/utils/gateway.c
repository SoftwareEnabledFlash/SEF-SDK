/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * gateway.c
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
#include "gateway.h"
#include "sef-event.h"

#include <errno.h>
#include <stdatomic.h>
#include <stdint.h>

enum sefGatewayStatus { kSEFGatewayClosed = 0, kSEFGatewayOpen = 1, kSEFGatewayFlush = 2 };

struct sefGatewayWait
{
    atomic_uint_least64_t count;    // count copied from gateway (if next is null)
    struct sefGatewayWait* next;    // ptr to next waiter
    uint8_t state;                  // state to set before waiters are signaled
    struct sefEvent waitEvent;      // event signaled when count hits 0
};

void sefGatewayInit(struct sefGateway* gateway)
{
    gateway->gate.state = kSEFGatewayClosed;
    gateway->gate.data = 0;
}

int sefGatewayOpen(struct sefGateway* gateway)
{
    struct sefGateway tempGateway = {.gate.state = kSEFGatewayOpen};
    // only open gateway if it's closed
    if (atomic_load(&gateway->val))
    {
        return EAGAIN;
    }

    atomic_store(&gateway->val, tempGateway.val);
    return 0;
}

// Someone has called sefGatewaySetWait() converting gateway.data from a
// counter to a pointer to a sefGatewayWait struct.  This is done to drain
// the gateway.  This function allows the caller to also wait for the gateway
// to drain (called by threads trying to enter a gateway that's flushing).
// The Incoming oldVal points to the value of gateway when the caller noticed
// it was in a draining state.  Add to waiter list by linking a sefGatewayWait
// struct into the waiter's list.  gateway.data (missing the bottom two bits)
// is the head pointer and it's done with a simple push.
//
// If the gateway has changed values (*oldVal doesn't match), EAGAIN is
// returned and *oldVal updated to what was there instead.
//
// If we got into the waiters list, the code blocks until its event is set
// and then returns 0.
static int sefGatewayPushWait(struct sefGateway* gateway, uint64_t* oldVal)
{
    // create a callback
    struct sefGatewayWait wait;
    struct sefGateway tempGateway = {.val = *oldVal};

    // link ourselves in
    wait.next = (void*)((uintptr_t)tempGateway.gate.data << 2);
    tempGateway.gate.data = ((uintptr_t)&wait) >> 2;

    SefEventInit(&wait.waitEvent, 0);
    if (!atomic_compare_exchange_weak(&gateway->val, oldVal, tempGateway.val))
    {
        SefEventFree(&wait.waitEvent);
        return EAGAIN;
    }

    // wait for all the entries to leave
    SefEventWait(&wait.waitEvent);
    SefEventFree(&wait.waitEvent);
    return 0;
}

// Caller is setting a state that will cause the gateway to drain (flush or close)
// and needs to wait for the drain to finish (the first waiter so to speak for
// flush).  Incoming oldVal is what the gateway was when the caller decided to
// cause the gateway to drain.  s1 is the state the gateway is moving to to
// cause the gateway to drain (flush or closed).  s2 is the state the gateway
// will move to once the gateway is drained (flush goes back to open and closed
// stays closed).
//
// The gateway is converting data from a counter to a pointer to a sefGatewayWait
// struct that will hold the counter instead.  Threads leaving the gateway will
// walk the list of wait struct until the last one is hit and that one holds the
// count.  Whoever decrements the count to 0 pops the whole list off setting
// the gateway state to s2 (saved in the sefGatewayWait struct), walks the list
// signalling everyone's event.  Any other code that wishes to wait as well
// can call sefGatewayPushWait().
//
// If the gateway has changed values (*oldVal doesn't match), EAGAIN is
// returned and *oldVal updated to what was there instead.
//
// If we converted the gateway to be waiters list, the code blocks until its
// event is set and then returns 0.
static int sefGatewaySetWait(struct sefGateway* gateway, uint64_t* oldVal, uint8_t s1, uint8_t s2)
{
    struct sefGateway newVal;
    struct sefGatewayWait wait;
    struct sefGateway tempGateway = {.val = *oldVal};

    SefEventInit(&wait.waitEvent, 0);
    wait.count = tempGateway.gate.data;
    wait.state = s2;
    wait.next = NULL;
    newVal.gate.state = s1;
    newVal.gate.data = ((uintptr_t)&wait) >> 2;
    if (!atomic_compare_exchange_weak(&gateway->val, oldVal, newVal.val))
    {
        SefEventFree(&wait.waitEvent);
        return EAGAIN;
    }

    // wait for all the entries to leave
    SefEventWait(&wait.waitEvent);

    SefEventFree(&wait.waitEvent);
    return 0;
}

// Set state to drain entries, s1 to drain, s2 once drained
static int sefGatewayDrain(struct sefGateway* gateway, uint8_t s1, uint8_t s2)
{
    struct sefGateway tempGateway;
    uint64_t oldVal;

    // get the gateway value
    oldVal = atomic_load(&gateway->val);
    for (;;)
    {
        tempGateway.val = oldVal;

        // only can drain an open gateway
        if (tempGateway.gate.state != kSEFGatewayOpen)
        {
            return EINVAL;
        }

        // set final state of the gateway if no data
        if (!tempGateway.gate.data)
        {
            tempGateway.gate.state = s2;
            if (!atomic_compare_exchange_weak(&gateway->val, &oldVal, tempGateway.val))
            {
                continue;
            }

            return 0;
        }

        if (!sefGatewaySetWait(gateway, &oldVal, s1, s2))
        {
            continue;
        }

        return 0;
    }
}

int sefGatewayClose(struct sefGateway* gateway)
{
    return sefGatewayDrain(gateway, kSEFGatewayClosed, kSEFGatewayClosed);
}

int sefGatewayFlush(struct sefGateway* gateway)
{
    return sefGatewayDrain(gateway, kSEFGatewayFlush, kSEFGatewayOpen);
}

// Caller wants to enter the gateway, it must be in the open or flushing
// state.  All others status will return EACCES result.
//
// If the gateway is open, gateway.data is incremented to keep count of the
// number of threads that have entered.  If 0 is returned, the caller is
// required to call sefGatewayLeave() to undo the action of entering.
//
// If the gateway is flushing, the thread will block waiting for the flush
// to finish (the gateway count to become 0 - which as moved from gateway.data
// to a last sefGatewayWait in the list pointed to by gateway.data).  Once
// the flush is complete, it retries to enter, which may fail or may succeed.
//
int sefGatewayEnter(struct sefGateway* gateway)
{
    struct sefGateway tempGateway;
    uint64_t oldVal;

    // get the gateway value
    oldVal = atomic_load(&gateway->val);
    do
    {
        tempGateway.val = oldVal;

        if (tempGateway.gate.state == kSEFGatewayFlush)
        {
            sefGatewayPushWait(gateway, &oldVal);
            continue;
        }

        // don't let any entries if closed
        if (tempGateway.gate.state != kSEFGatewayOpen)
        {
            return EACCES;
        }

        tempGateway.gate.data++;
    } while (!atomic_compare_exchange_weak(&gateway->val, &oldVal, tempGateway.val));

    return 0;
}

// A caller that has called sefGetwayEnter() with a 0 return status is required
// to call sefGatewayLeave() to indicate they no longer have a reference to the
// resource protected by the gateway.
//
// When the gateway is in the open state, this simply decrements the value of
// gateway.data.
//
// If it's not in the open state, it's assumed it's in a waiting state and
// gateway.data is a pointer (with the bottom two bits missing) pointing to a
// list of sefGatewayWait structures.  The count to decrement is instead held
// in the last entry of the list.  If that count becomes zero, the list is
// popped off by zeroing gateway.data and setting the state to the state in the
// last sefGatewayWait struct.  Then the events in each sefGatewayWait struct
// are set to let other threads know their wait is over.
//
// Calling leave on a close gateway is undefined.
//
int sefGatewayLeave(struct sefGateway* gateway)
{
    struct sefGateway tempGateway;
    uint64_t oldVal;

    for (;;)
    {
        // get the gateway value
        oldVal = atomic_load(&gateway->val);
        tempGateway.val = oldVal;

        // decrement the ref count if open
        if (tempGateway.gate.state == kSEFGatewayOpen)
        {
            tempGateway.gate.data--;

            if (!atomic_compare_exchange_strong(&gateway->val, &oldVal, tempGateway.val))
            {
                continue;
            }

            return 0;
        }

        // decrement the waiting ref count if draining
        struct sefGatewayWait* wait = (void*)((uintptr_t)tempGateway.gate.data << 2);
        while (wait->next)
        {
            wait = wait->next;
        }
        if (atomic_fetch_sub(&wait->count, 1) - 1 == 0)
        {    // detach list/set new state, signal waiters
            struct sefGateway postWait = {.gate.state = wait->state};
            tempGateway.val = atomic_exchange(&gateway->val, postWait.val);
            wait = (void*)((uintptr_t)tempGateway.gate.data << 2);
            do
            {
                struct sefGatewayWait* next = wait->next;
                SefEventSet(&wait->waitEvent);    // wait is invalid after this
                wait = next;
            } while (wait);
            break;
        }
        break;
    }
    return 0;
}
