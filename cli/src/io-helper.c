/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * io-helper.c
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
#include "io-helper.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int globalMessageLevel = kInfo;
int globalOutputWidth = 80;

void CIHSetLevel(int level)
{
    globalMessageLevel = level;
}

void CIHSetOutputWidth(int new_width)
{
    globalOutputWidth = new_width;
}

void CIHMessagePrint(int Level, int frontIndent, int secondLineOffset, const char *Format, ...)
{
    if (Level >= globalMessageLevel && Level > -1 && Level <= kError)
    {
        va_list args;

        if (frontIndent >= 0 && secondLineOffset >= 0)
        {
            int effectiveTextWidth;    // number of characters (not including indent) allowed in a line before wrapping
            int lengthToPrint;    // current length of a line of output being formed prior to printing (wrapped)
            char *priorSpace;    // pointer to the last space found in a line
            char *nextSpace;     // pointer to space following last space in a line
            int messageSize;     // length of message being printed
            char *thisLine;    // pointer to current line being printed (for text-wrapping purposes)

            // find length of message being printed, for text-wrapping purposes
            va_start(args, Format);
            messageSize = vsnprintf(NULL, 0, Format, args) + 1;
            va_end(args);

            // place message being printed into buffer
            char buffer[messageSize];
            va_start(args, Format);
            vsprintf(buffer, Format, args);
            va_end(args);

            // find number of characters allowed per line given presence of front indent
            effectiveTextWidth = globalOutputWidth - frontIndent;

            // if message exceeds effective text width (found above), wrap text
            if (messageSize >= effectiveTextWidth)
            {
                thisLine = buffer;    // point thisLine at the beginning of the message to print
                while (thisLine)    // continue to wrap as long as characters remain in the message to print
                {
                    priorSpace = strchr(thisLine, ' ');       // find first space in current line
                    lengthToPrint = priorSpace - thisLine;    // calculate length of first word in line
                    while (lengthToPrint < effectiveTextWidth && priorSpace)
                    {    // continue adding words to current line until it just fits in additional column width
                        if (strlen(thisLine) < effectiveTextWidth)
                        {    // if remainder of text in thisLine doesn't exceed column width, break loop and print as-is
                            priorSpace = nextSpace = NULL;
                            lengthToPrint = strlen(thisLine);
                            break;
                        }
                        if (priorSpace)    // if more characters present after last space, look for another space/word
                        {
                            nextSpace = strchr(priorSpace + sizeof(char), ' ');
                        }
                        if (nextSpace && nextSpace - thisLine < effectiveTextWidth)
                        {    // if more words after next space, and can fit them in this line, add them
                            priorSpace = nextSpace;
                            lengthToPrint = nextSpace - thisLine;
                        }
                        else
                        {    // if no more words can fit or there are no more words in this line, break loop
                            lengthToPrint = priorSpace - thisLine;
                            break;
                        }
                    }

                    if (thisLine ==
                        buffer)    // first line may have its own formatting, so it's handled uniquely
                    {
                        printf("%*.s%.*s\n", frontIndent, "", lengthToPrint, thisLine);

                        // put indent meant for wrapping offset into effect after first line is printed
                        effectiveTextWidth -= secondLineOffset;
                    }
                    else    // all lines after first include the second-line wrapping offset
                    {
                        printf("%*.s%*.s %.*s\n", frontIndent, "", secondLineOffset, "",
                               lengthToPrint, thisLine);
                    }

                    if (priorSpace)    // if more words in message to print, continue searching
                                       // after current line is printed
                    {
                        thisLine = priorSpace + sizeof(char);
                    }
                    else    // if no more words (i.e. whole original message has been printed),
                            // priorSpace will be NULL, and so should thisLine be
                    {
                        thisLine = priorSpace;
                    }
                }
            }
            else
            {    // if text already fits without any wrapping needed, print it as-is
                printf("%*.s%s\n", frontIndent, "", buffer);
            }
        }
        else    // if neither text wrapping nor front indent was ever desired, print normally
        {
            va_start(args, Format);
            vprintf(Format, args);
            va_end(args);
            printf("\n");
        }
    }
}

bool CIHGetBoolInput(const char *MessagePrompt, ...)
{
    char userInput, userString[256];
    va_list args;

    va_start(args, MessagePrompt);
    vprintf(MessagePrompt, args);
    va_end(args);

    printf(" (y/[n])");

    fgets(userString, 256, stdin);
    sscanf(userString, "%c%*s\n", &userInput);

    // user choice defaults to "no"
    return (userInput == 'Y' || userInput == 'y');
}

bool CIHIsOutOfRange(const char *PropertyName,
                     char PropertyShortName,
                     uint64_t Value,
                     uint64_t MinValue,
                     uint64_t MaxValue)
{
    if (Value < MinValue || Value > MaxValue)
    {
        if (PropertyShortName == '-')
        {
            CIHMessageError("Error: invalid argument, \"%" PRIu64
                            "\", for option '--%s', is out of range (%" PRIu64 "- %" PRIu64 ")",
                            Value, PropertyName, MinValue, MaxValue);
        }
        else
        {
            CIHMessageError("Error: invalid argument, \"%" PRIu64
                            "\", for option '--%s' (`-%c`), is out of range (%" PRIu64 "- %" PRIu64
                            ")",
                            Value, PropertyName, PropertyShortName, MinValue, MaxValue);
        }

        return true;
    }

    return false;
}

bool CIHIsNotGiven(const char *PropertyName, char PropertyShortName, bool IsGiven)
{
    if (!IsGiven)
    {
        if (PropertyShortName == '-')
        {
            CIHMessageError("Error: '--%s' option required", PropertyName);
        }
        else
        {
            CIHMessageError("Error: '--%s' (`-%c`) option required", PropertyName, PropertyShortName);
        }

        return true;
    }

    return false;
}
