/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * str-builder.h
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
#pragma once
#ifndef util_strbuilder_h
#define util_strbuilder_h

#include <stddef.h>

#include "dlist.h"

struct StrBuilder
{
    TmaDList str_list;
    size_t cap;    // default cap for appended fragments.
};

/**
 *  @brief     Initializes the string builder object
 *
 *  @param     strb        a pointer to the string builder object to initialize
 *  @param	   cap		   initial capacity of the string
 */
void StrBldInit(struct StrBuilder *strb, size_t cap);

/**
 *  @brief     Cleans up the string builder object
 *
 *  @param     strb        a pointer to the string builder object
 */
void StrBldCleanup(struct StrBuilder *strb);

/**
 *  @brief     Returns the current size of the string
 *
 *  @param     strb        a pointer to the string builder object
 *
 *  @returns   Size of the constructed string
 */
size_t StrBldGetLen(struct StrBuilder *strb);

/**
 *  @brief     Allocates the memory and returns the address for the constructed string
 *
 *  @param     strb        a pointer to the string builder object
 *
 *  @returns   Pointer to the allocated memory containing constructed string
 */
char *StrBldToString(struct StrBuilder *strb);

/**
 *  @brief     Appends to the end of the constructed string
 *
 *  @param     strb        a pointer to the string builder object
 *  @param     s           the string to be appended
 */
void StrBldAppend(struct StrBuilder *strb, const char *s);

/**
 *  @brief     Appends to the end of the constructed string with format
 *
 *  @param     strb        a pointer to the string builder object
 *  @param     fmt         the string to be appended with support for the format with given data
 */
void StrBldAppendFormat(struct StrBuilder *strb, const char *fmt, ...);

/**
 *  @brief     Deletes part of the constructed string
 *
 *  @param     strb        a pointer to the string builder object
 *  @param     begin       the index at which to begin deleting
 *  @param     end         the index at which to end deleting; end of SIZE_MAX to delete to eos
 */
void StrBldDelete(struct StrBuilder *strb, size_t begin, size_t end);

/**
 *  @brief     Inserts a string at the given index position
 *
 *  @param     strb        a pointer to the string builder object
 *  @param     pos         the index at which the string should be inserted
 *  @param     s           the string to be inserted
 */
void StrBldInsert(struct StrBuilder *strb, size_t pos, char *s);

/**
 *  @brief     Inserts a string at the given index position with format
 *
 *  @param     strb        a pointer to the string builder object
 *  @param     pos         the index at which the string should be inserted
 *  @param     fmt         the string to be inserted with support for the format with given data
 */
void StrBldInsertFormat(struct StrBuilder *strb, size_t pos, const char *fmt, ...);

#endif    // def util_strbuilder_h
