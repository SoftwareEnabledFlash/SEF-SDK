/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * config.h
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
#ifndef CONFIG_H
#define CONFIG_H

#define CC_DIE_STATS 1    //!< Track die access stats
#define CC_HEATMAP   0    //!< Sub-option of including per die block stats

#define CC_DEBUG_COUNTERS 1    //!< Include code coverage debug counters

#define CC_MANUAL_NLC     1    //!< Adding manual nameless copy implementation
#define CC_GC_SINGLE_PLID 0    //!< GC will stay within a single placement ID
#define CC_WFQ_ZENO \
    0                          //!< 0 - GC weight is static based on op (1/op -1)
                               //   1 - GC weight based on on 1/lg2(nFree)
#define CC_PER_SB_LATENCY 0    //!< Enable to track latency per super block
#define CC_DEMO_DP        1    //!< Enable counters for data placement demo

#define SEF_DEFAULT_OP   7           //!< Over provisioning percentage used when auto configured */
#define SEF_LUT_META_TAG 0x904d60    //!< Metadata value used for persisted LUT
#define SEF_USR_META_TAG 0x8574c6    //!< Metadata value used for user data

#define FTL_PDL_ROOT_INDEX        0    //!< The root pointer index used to store the root of the PDL tree
#define FTL_PDL_QOS_DOMAINS_INDEX 1    //!< The root pointer index used to store the num QoS Domains

#define SEF_EXIT_ON_NO_Meme 1    //!< Should exit if memory allocation fails

#define SEFBlockGUID                                                                                   \
    {                                                                                                  \
        0x57, 0x38, 0x48, 0x4a, 0xc9, 0x4b, 0x25, 0x45, 0xb8, 0xce, 0x43, 0x01, 0x83, 0xe6, 0xd8, 0xef \
    }

#define Block_Default_Config                                                                     \
    {                                                                                            \
        .overprovisioning = SEF_DEFAULT_OP, .numDomains = 1, .blockOption = Block_Default_Option \
    }

#define Block_Default_Option                                                \
    {                                                                       \
        .logLevel = 2, .instrumentationPath = Block_Default_Instrumentation \
    }

#define Block_Default_Log "/tmp/SEFLog.%d.%d"

#define Block_Default_Instrumentation "/tmp/SEFFTLDomain.%d.%d"

#endif /* CONFIG_H */
