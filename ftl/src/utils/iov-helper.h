/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * iov-helper.h
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
#ifndef IOV_HELPER_H
#define IOV_HELPER_H

#include <stdint.h>
#include <sys/uio.h>

/**
 * @brief Calculates the length of a scatter/gather list
 *
 * @param iov       Pointer to a scatter/gather list
 * @param iovcnt    Numer of elements in the scatter/gather list
 *
 * @return ssize_t  Length in bytes of iov
 */
ssize_t lenIov(struct iovec *iov, int iovcnt);

/**
 *  @brief      This function initializes the iov, starting at iovOffset with the given value for count bytes.
 *
 *  @param      iov              A pointer to the scatter/gather list
 *  @param      iovcnt           The number of elements in the scatter/gather list
 *  @param      iovOffset        The offset for the scatter/gather list
 *  @param      value            The value that is used to replace the iov
 *  @param      count            The number of iovs that should be replaced with the value
 */
void memsetIov(struct iovec *iov, int iovcnt, uint32_t iovOffset, char value, int count);

/**
 *  @brief      This function copies the source iov to the destination, starting at iovOffset for count bytes.
 *
 *  @param      srcIov           A pointer to the source scatter/gather list
 *  @param      srcIovcnt        The number of elements in the source scatter/gather list
 *  @param      srcIovOffset     The offset for the source scatter/gather list
 *  @param      destIov          A pointer to the destination scatter/gather list
 *  @param      destIovcnt       The number of elements in the destination scatter/gather list
 */
void copyIov(const struct iovec *srcIov,
             int srcIovcnt,
             size_t srcIovOffset,
             struct iovec **destIov,
             uint16_t *destIovcnt);

#endif /* IOV_HELPER_H */
