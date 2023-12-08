/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * str-builder.c
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
#include "str-builder.h"

#include <assert.h>
#include <memory.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "dlist.h"

// LCOV_EXCL_START

struct str_frag
{
    TmaDListEntry link;
    size_t len;
    size_t cap;
    char str[];
};

struct str_frag *strBldMkFrag(size_t cap, TmaDListEntry *ip)
{
    struct str_frag *sf = malloc(cap + sizeof(struct str_frag));

    sf->cap = cap;
    sf->len = 0;
    utl_DListInitEntry(&sf->link);
    utl_DListInsertAfter(ip, &sf->link);
    return sf;
}

struct str_frag *strBldNextFrag(struct StrBuilder *strb, struct str_frag *sf)
{
    return utl_DListNextAs(&strb->str_list, sf, struct str_frag, link);
}

struct str_frag *strBldGetTail(struct StrBuilder *strb)
{
    return utl_DListGetTailAs(&strb->str_list, struct str_frag, link);
}

struct str_frag *strBldFindFrag(struct StrBuilder *strb, size_t pos, size_t *off)
{
    struct str_frag *sf = strBldNextFrag(strb, NULL);
    size_t len = sf ? sf->len : 0;

    while (len < pos && (sf = strBldNextFrag(strb, sf)))
    {
        len += sf->len;
    }
    if (off && sf)
    {
        *off = pos - (len - sf->len);
    }
    return sf;
}

struct str_frag *strBldSplitFrag(struct str_frag *sf, size_t pos)
{
    struct str_frag *tail = sf;

    if (pos && pos < sf->len)
    {
        size_t len = sf->len - pos;
        tail = strBldMkFrag(len, &sf->link);
        memcpy(tail->str, sf->str + pos, len);
        tail->len = len;
        sf->len = pos;
    }
    return tail;
}

struct str_frag *strBldDeleteFrag(struct StrBuilder *strb, struct str_frag *sf)
{
    struct str_frag *next;

    next = utl_DListNextAs(&strb->str_list, sf, struct str_frag, link);
    free(utl_DListRemove(&sf->link));
    return next;
}

void strBldInsert(struct StrBuilder *strb, struct str_frag *sf, const char *s, int len)
{
    int n;

    len = len ?: strlen(s);
    sf = sf ?: strBldMkFrag(len, &strb->str_list.head);
    n = sf->cap - sf->len;
    if (n > len)
    {
        n = len;
    }
    memcpy(sf->str + sf->len, s, n);
    sf->len += n;
    len -= n;
    if (len)
    {
        sf = strBldMkFrag(len, &sf->link);
        memcpy(sf->str, s + n, len);
        sf->len = len;
    }
}

void StrBldInit(struct StrBuilder *strb, size_t cap)
{
    utl_DListInit(&strb->str_list);
    strb->cap = cap;

    if (cap)
    {
        strBldMkFrag(cap, &strb->str_list.head);
    }
}

void StrBldCleanup(struct StrBuilder *strb)
{
    TmaDListEntry *n = utl_DListPopHead(&strb->str_list);

    while (n)
    {
        free(n);
        n = utl_DListPopHead(&strb->str_list);
    }
}

size_t StrBldGetLen(struct StrBuilder *strb)
{
    struct str_frag *sf = NULL;
    size_t len = 0;

    while ((sf = strBldNextFrag(strb, sf)))
    {
        len += sf->len;
    }
    return len;
}

char *StrBldToString(struct StrBuilder *strb)
{
    struct str_frag *sf = NULL;
    size_t len = StrBldGetLen(strb);
    char *p, *s = malloc(len + 1);

    p = s;
    while ((sf = utl_DListNextAs(&strb->str_list, sf, struct str_frag, link)))
    {
        memcpy(p, sf->str, sf->len);
        p += sf->len;
    }
    assert(p == s + len);
    s[len] = '\0';
    return s;
}

void StrBldAppend(struct StrBuilder *strb, const char *s)
{
    strBldInsert(strb, strBldGetTail(strb), s, 0);
}

void StrBldAppendFormat(struct StrBuilder *strb, const char *fmt, ...)
{
    va_list vl;
    size_t len;

    va_start(vl, fmt);
    len = vsnprintf(NULL, 0, fmt, vl);
    va_end(vl);

    char s[len + 1];
    va_start(vl, fmt);
    vsnprintf(s, len + 1, fmt, vl);
    va_end(vl);
    strBldInsert(strb, strBldGetTail(strb), s, len);
}

void StrBldDelete(struct StrBuilder *strb, size_t begin, size_t end)
{
    size_t off = 0;
    size_t len = end - begin;
    struct str_frag *sf = strBldFindFrag(strb, begin, &off);

    if (sf)
    {
        sf = strBldSplitFrag(sf, off);
    }
    while (sf && sf->len <= len)
    {
        len -= sf->len;
        sf = strBldDeleteFrag(strb, sf);
    }
    if (sf && len)
    {
        strBldSplitFrag(sf, len);
        strBldDeleteFrag(strb, sf);
    }
}

void StrBldInsert(struct StrBuilder *strb, size_t pos, char *s)
{
    size_t off = 0;
    struct str_frag *sf = strBldFindFrag(strb, pos, &off);

    if (sf)
    {
        strBldSplitFrag(sf, off);
    }
    else
    {
        sf = strBldGetTail(strb);
    }
    strBldInsert(strb, sf, s, 0);
}

void StrBldInsertFormat(struct StrBuilder *strb, size_t pos, const char *fmt, ...)
{
    va_list vl;
    size_t len;

    va_start(vl, fmt);
    len = vsnprintf(NULL, 0, fmt, vl);
    va_end(vl);

    char s[len + 1];
    va_start(vl, fmt);
    vsnprintf(s, len + 1, fmt, vl);
    va_end(vl);
    StrBldInsert(strb, pos, s);
}
// LCOV_EXCL_STOP
