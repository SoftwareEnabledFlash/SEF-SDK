/*
 * SOFTWARE-ENABLED FLASH (“SEF”)
 * Software Development Kit (SDK)
 * shell.c
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
#include "../cli-engine.h"
#include "../engine-option.h"

#define PY_SSIZE_T_CLEAN
#define _GNU_SOURCE 1

#include <Python.h>
#include <SEFAPI.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../io-helper.h"
#include "../sef-helper.h"
#include "../utils/str-error.h"

// python module structs
struct SefUnitEntry
{
    int unitIndex;
    SEFHandle sefHandle;
    struct QosDomainEntry *selectedQosDomain;
    TmaDListEntry link;
};

struct QosDomainEntry
{
    struct SEFQoSDomainID qosDomainId;
    SEFQoSHandle qosHandle;
    struct SefUnitEntry *parentUnit;
    TmaDListEntry link;
};

struct SuperblockEntry
{
    struct SEFFlashAddress superblock;
    struct QosDomainEntry *parentQosDomain;
    TmaDListEntry link;
};

struct SefCliPyModule
{
    int numSEFUnit;
    PyObject *cliError;
    wchar_t *pythonProgram;

    // selected units
    struct SefUnitEntry *selectedSefUnit;
    struct QosDomainEntry *selectedQosDomain;

    // list of active units
    TmaDList activeSefUnit;
    TmaDList activeQosDomain;
    TmaDList activeSuperblock;
};

// engine globals
struct SefCliPyModule scpm;

// python helper functions
static void printPythonHelp();

static void updateStatusLine()
{
    char *statusLinePtr, statusLine[40];
    statusLinePtr = statusLine;

    // print selected unit information
    if (scpm.selectedQosDomain != NULL)
    {
        statusLinePtr += sprintf(statusLinePtr, "[%5d@", scpm.selectedQosDomain->qosDomainId.id);
    }
    else
    {
        statusLinePtr += sprintf(statusLinePtr, "[<qos>@");
    }

    if (scpm.selectedSefUnit != NULL)
    {
        statusLinePtr += sprintf(statusLinePtr, "%5d]", scpm.selectedSefUnit->unitIndex);
    }
    else
    {
        statusLinePtr += sprintf(statusLinePtr, "<sef>]");
    }

    sprintf(statusLinePtr, " >>> ");

    // Python Limited API
    //_Py_IDENTIFIER(ps1);
    //_PySys_SetObjectId(&PyId_ps1, PyUnicode_FromString(statusLine));
    PySys_SetObject("ps1", PyUnicode_FromString(statusLine));
}

static bool isQosDomainOpen()
{
    struct QosDomainEntry *entry = NULL;

    if (utl_DListIsEmpty(&scpm.activeQosDomain))
    {
        return false;
    }

    while ((entry = utl_DListNextAs(&scpm.activeQosDomain, entry, struct QosDomainEntry, link)))
    {
        if (scpm.selectedSefUnit->unitIndex == entry->parentUnit->unitIndex)
        {
            return true;
        }
    }

    return false;
}

static bool isStatusValid(bool requiresSEF, bool requiresQosDomain)
{
    // validate if sef unit is selected
    if (requiresSEF && scpm.selectedSefUnit == NULL)
    {
        PyErr_SetString(scpm.cliError, "There is no selected SEF Units");
        return false;
    }

    // validate if any qos domains exist
    if (requiresQosDomain && !isQosDomainOpen())
    {
        PyErr_SetString(scpm.cliError, "There is no open QoS Domain");
        return false;
    }

    // validate if qos domain is selected
    if (requiresQosDomain && scpm.selectedQosDomain == NULL)
    {
        PyErr_SetString(scpm.cliError, "There is no selected QoS Domain");
        return false;
    }

    return true;
}

// python module functions
static PyObject *sefCliHelp(PyObject *self)
{
    printPythonHelp();

    Py_RETURN_NONE;
}

static PyObject *selectSEF(PyObject *self, PyObject *args, PyObject *kwds)
{
    int sefUnitIndex;
    struct SefUnitEntry *entry = NULL, *newEntry;
    static char *keywords[] = {"SEFIndex", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "i", keywords, &sefUnitIndex))
    {
        return NULL;
    }

    // check sef unit index
    if (sefUnitIndex < 0 || sefUnitIndex > scpm.numSEFUnit)
    {
        PyErr_SetString(scpm.cliError, "The SEF Unit index is not valid");
        return NULL;
    }

    // check if SEF Unit open
    while ((entry = utl_DListNextAs(&scpm.activeSefUnit, entry, struct SefUnitEntry, link)))
    {
        if (entry->unitIndex == sefUnitIndex)
        {
            scpm.selectedSefUnit = entry;
            scpm.selectedQosDomain = entry->selectedQosDomain;

            // update python status line
            updateStatusLine();

            Py_RETURN_NONE;
        }
    }

    // open sef unit if not open
    // prepare new object
    newEntry = malloc(sizeof(struct SefUnitEntry));
    newEntry->unitIndex = sefUnitIndex;
    newEntry->selectedQosDomain = NULL;

    // open sef unit
    newEntry->sefHandle = SEFGetHandle(sefUnitIndex);
    if (newEntry->sefHandle == NULL)
    {
        PyErr_SetString(scpm.cliError, "Was unable to open SEF Unit");
        free(newEntry);
        return NULL;
    }

    // reset selected qos domain
    scpm.selectedSefUnit = newEntry;
    scpm.selectedQosDomain = NULL;

    // add sef unit to the available units
    utl_DListInitEntry(&newEntry->link);
    utl_DListPushHead(&scpm.activeSefUnit, &newEntry->link);

    // update python status line
    updateStatusLine();

    Py_RETURN_NONE;
}

static PyObject *listSEF(PyObject *self)
{
    int i;
    PyObject *returnSEFIndex;

    // prepare the return array
    returnSEFIndex = PyList_New(scpm.numSEFUnit);
    for (i = 0; i < scpm.numSEFUnit; i++)
    {
        PyList_SET_ITEM(returnSEFIndex, i, Py_BuildValue("i", i));
    }

    return returnSEFIndex;
}

static PyObject *openQoS(PyObject *self, PyObject *args, PyObject *kwds)
{
    bool isLabel = false;
    int qosDomainId = 0;
    struct SIFLabel label = {{0, 0}};
    struct SEFStatus status;
    struct SEFQoSDomainID sefQosDomainId;
    struct QosDomainEntry *entry = NULL, *newEntry;
    static char *keywords[] = {"QoSDomainId", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "i", keywords, &qosDomainId))
    {
        static char *labelKeywords[] = {"label1", "label2", NULL};

        PyErr_Clear();
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "KK", labelKeywords, &label.data[0],
                                         &label.data[1]))
        {
            return NULL;
        }

        isLabel = true;
    }

    if (!isStatusValid(true, false))
    {
        return NULL;
    }

    if (isLabel)
    {
        int qosListSize;
        struct SEFQoSDomainList *qosList;

        qosListSize = sizeof(struct SEFQoSDomainList) + sizeof(struct SEFQoSDomainID);
        qosList = (struct SEFQoSDomainList *)malloc(qosListSize);

        status = SUGetQosDomainId(scpm.selectedSefUnit->sefHandle, label, qosList, qosListSize);
        if (status.error)
        {
            PyErr_SetString(scpm.cliError, "Was unable to get the associated QoS Domain");
            free(qosList);
            return NULL;
        }
        else if (qosList->numQoSDomains == 0)
        {
            PyErr_SetString(scpm.cliError, "The label is not associated with any QoS Domain");
            free(qosList);
            return NULL;
        }
        else if (qosList->numQoSDomains > 1)
        {
            PyErr_SetString(scpm.cliError, "The label is associated with more than 1 QoS Domain");
            free(qosList);
            return NULL;
        }

        qosDomainId = qosList->QoSDomainID[0].id;
    }

    // check against active qos domains
    while ((entry = utl_DListNextAs(&scpm.activeQosDomain, entry, struct QosDomainEntry, link)))
    {
        if (entry->qosDomainId.id == qosDomainId &&
            entry->parentUnit->unitIndex == scpm.selectedSefUnit->unitIndex)
        {
            PyErr_SetString(scpm.cliError, "The QoS Domain is open already");
            return NULL;
        }
    }

    // validate QoS Domain
    sefQosDomainId.id = qosDomainId;
    if (!CSHIsQosDomainIdValid(scpm.selectedSefUnit->sefHandle, sefQosDomainId))
    {
        PyErr_SetString(scpm.cliError, "The QoS Domain Id is not valid");
        return NULL;
    }

    // create new qos domain entry
    newEntry = malloc(sizeof(struct QosDomainEntry));
    newEntry->parentUnit = scpm.selectedSefUnit;
    newEntry->qosDomainId.id = qosDomainId;

    // open qos domain
    status = SEFOpenQoSDomain(newEntry->parentUnit->sefHandle, newEntry->qosDomainId, NULL, NULL,
                              NULL, &newEntry->qosHandle);
    if (status.error)
    {
        switch (status.error)
        {
            case EACCES:
                PyErr_SetString(scpm.cliError,
                                "You do not have the permissions needed to access this Qos Domain");
                break;

            default:
                PyErr_SetString(scpm.cliError, "Was unable to open QoS Domain");
                break;
        }

        free(newEntry);
        return NULL;
    }

    // reset selected qos domain
    scpm.selectedSefUnit->selectedQosDomain = newEntry;
    scpm.selectedQosDomain = newEntry;

    // add qos domain to the available qos domain handle
    utl_DListInitEntry(&newEntry->link);
    utl_DListPushHead(&scpm.activeQosDomain, &newEntry->link);

    // update python status line
    updateStatusLine();

    Py_RETURN_NONE;
}

static PyObject *closeQoS(PyObject *self)
{
    struct SEFStatus status;
    struct SuperblockEntry *superblockEntry = NULL;

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // close qos domain
    status = SEFCloseQoSDomain(scpm.selectedQosDomain->qosHandle);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to close QoS Domain");
        return NULL;
    }

    // remove closed superblocks from list
    while ((superblockEntry = utl_DListNextAs(&scpm.activeSuperblock, superblockEntry,
                                              struct SuperblockEntry, link)))
    {
        if (superblockEntry->parentQosDomain->qosDomainId.id == scpm.selectedQosDomain->qosDomainId.id &&
            superblockEntry->parentQosDomain->parentUnit->unitIndex == scpm.selectedSefUnit->unitIndex)
        {
            struct SuperblockEntry *closedSuperBlock = superblockEntry;

            superblockEntry = utl_DListPrevAs(&scpm.activeSuperblock, superblockEntry,
                                              struct SuperblockEntry, link);

            utl_DListRemove(&closedSuperBlock->link);
            free(closedSuperBlock);
        }
    }

    // remove from active qos doamins
    utl_DListRemove(&scpm.selectedQosDomain->link);

    // reset selected qos domain
    free(scpm.selectedQosDomain);
    scpm.selectedQosDomain = NULL;
    scpm.selectedSefUnit->selectedQosDomain = NULL;

    // update python status line
    updateStatusLine();

    Py_RETURN_NONE;
}

static PyObject *selectQoS(PyObject *self, PyObject *args, PyObject *kwds)
{
    int qosDomainId;
    struct QosDomainEntry *entry = NULL;
    static char *keywords[] = {"QoSDomainId", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "i", keywords, &qosDomainId))
    {
        return NULL;
    }

    if (!isStatusValid(true, false))
    {
        return NULL;
    }

    if (utl_DListIsEmpty(&scpm.activeQosDomain))
    {
        PyErr_SetString(scpm.cliError, "There are no open QoS Domains");
        return NULL;
    }

    // change sef unit handle
    while ((entry = utl_DListNextAs(&scpm.activeQosDomain, entry, struct QosDomainEntry, link)))
    {
        if (entry->qosDomainId.id == qosDomainId &&
            entry->parentUnit->unitIndex == scpm.selectedSefUnit->unitIndex)
        {
            scpm.selectedSefUnit->selectedQosDomain = entry;
            scpm.selectedQosDomain = entry;

            // update python status line
            updateStatusLine();

            Py_RETURN_NONE;
        }
    }

    PyErr_SetString(scpm.cliError,
                    "Was unable to select QoS domains, the QoS Domain Id is not valid");
    return NULL;
}

static PyObject *listQoS(PyObject *self)
{
    int i, qosDomainListSize;
    const struct SEFInfo *sefInfo;
    struct SEFStatus status;
    struct SEFQoSDomainList *qosDomainList;
    PyObject *returnQoSId;

    if (!isStatusValid(true, false))
    {
        return NULL;
    }

    // get sef info
    sefInfo = SEFGetInformation(scpm.selectedSefUnit->sefHandle);

    // get all qos domains
    qosDomainListSize = struct_size(qosDomainList, QoSDomainID, sefInfo->numQoSDomains);
    qosDomainList = malloc(qosDomainListSize);
    status = SEFListQoSDomains(scpm.selectedSefUnit->sefHandle, qosDomainList, qosDomainListSize);
    if (status.error)
    {
        free(qosDomainList);

        PyErr_SetString(scpm.cliError, "Was unable to get list of QoS Domains for SEF Unit Index");
        return NULL;
    }

    // prepare the return array
    returnQoSId = PyList_New(qosDomainList->numQoSDomains);
    for (i = 0; i < qosDomainList->numQoSDomains; i++)
    {
        struct QosDomainEntry *entry = NULL;
        struct SIFLabel qoslabel;
        int isOpen = 0;

        // check if sef unit is open
        while ((entry = utl_DListNextAs(&scpm.activeQosDomain, entry, struct QosDomainEntry, link)))
        {
            if (qosDomainList->QoSDomainID[i].id == entry->qosDomainId.id &&
                scpm.selectedSefUnit->unitIndex == entry->parentUnit->unitIndex)
            {
                isOpen = 1;
                break;
            }
        }

        status = SUGetLabel(scpm.selectedSefUnit->sefHandle, qosDomainList->QoSDomainID[i], &qoslabel);
        if (status.error)
        {
            free(qosDomainList);

            PyErr_SetString(scpm.cliError, "Was unable to get QoS Domain information");
            return NULL;
        }

        if (isOpen)
        {
            PyList_SET_ITEM(returnQoSId, i,
                            Py_BuildValue("(i,[K,K],O)", qosDomainList->QoSDomainID[i].id,
                                          qoslabel.data[0], qoslabel.data[1], Py_True));
        }
        else
        {
            PyList_SET_ITEM(returnQoSId, i,
                            Py_BuildValue("(i,[K,K],O)", qosDomainList->QoSDomainID[i].id,
                                          qoslabel.data[0], qoslabel.data[1], Py_False));
        }
    }

    // free dynamic memory
    free(qosDomainList);

    return returnQoSId;
}

static PyObject *allocateSuperBlock(PyObject *self, PyObject *args, PyObject *kwds)
{
    struct SEFStatus status;
    struct SuperblockEntry *newEntry;
    enum SEFSuperBlockType selectedType = kForWrite;
    static char *keywords[] = {"SuperBlockType", NULL};
    char *type = NULL;

    // validate input
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|s", keywords, &type))
    {
        return NULL;
    }

    if (type != NULL)
    {
        if (strcmp(type, "kForWrite") == 0)
        {
            selectedType = kForWrite;
        }
        else if (strcmp(type, "kForPSLCWrite") == 0)
        {
            selectedType = kForPSLCWrite;
        }
        else
        {
            PyErr_SetString(scpm.cliError, "SuperBlock Type input is not valid");
            return NULL;
        }
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // create new superblock entry
    newEntry = malloc(sizeof(struct SuperblockEntry));
    newEntry->parentQosDomain = scpm.selectedQosDomain;

    // open qos domain
    status = SEFAllocateSuperBlock(scpm.selectedQosDomain->qosHandle, &newEntry->superblock,
                                   selectedType, NULL, NULL);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to allocate superblock");
        return NULL;
    }

    // add qos domain to the available qos domain handle
    utl_DListInitEntry(&newEntry->link);
    utl_DListPushHead(&scpm.activeSuperblock, &newEntry->link);

    return Py_BuildValue("K", newEntry->superblock.bits);
}

static PyObject *closeSuperBlock(PyObject *self, PyObject *args, PyObject *kwds)
{
    unsigned long long inputFlashAddress;
    struct SEFStatus status;
    struct SEFFlashAddress flashAddress;
    struct SuperblockEntry *entry = NULL;
    static char *keywords[] = {"SuperBlockAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "K", keywords, &inputFlashAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // close superblock
    flashAddress.bits = inputFlashAddress;
    status = SEFCloseSuperBlock(scpm.selectedQosDomain->qosHandle, flashAddress);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to close SuperBlock");
        return NULL;
    }

    // remove from active superblock list
    while ((entry = utl_DListNextAs(&scpm.activeSuperblock, entry, struct SuperblockEntry, link)))
    {
        if (entry->superblock.bits == inputFlashAddress)
        {
            utl_DListRemove(&entry->link);
            free(entry);

            break;
        }
    }

    Py_RETURN_NONE;
}

static PyObject *listSuperBlock(PyObject *self)
{
    int i = 0;
    struct SEFStatus status;
    struct SEFSuperBlockList *superBlockList;
    PyObject *returnSuperblock;
    ssize_t listSize;

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get superblock records
    superBlockList = NULL;
    listSize = 0;
    status.info = 0;
    do
    {
        if (status.info > listSize)
        {
            free(superBlockList);
            listSize = status.info;
            superBlockList = malloc(status.info);
        }

        // get list
        status = SEFGetSuperBlockList(scpm.selectedQosDomain->qosHandle, superBlockList, listSize);
        if (status.error)
        {
            PyErr_SetString(
                scpm.cliError,
                "Error: Was unable to get list of the super blocks owned by the QoS Domain");
            PyErr_SetString(scpm.cliError, SEFStrError("SEFGetSuperBlockList", status.error));
            return NULL;
        }
    } while (status.info);

    // prepare the return array
    returnSuperblock = PyList_New(superBlockList->numSuperBlocks);
    for (i = 0; i < superBlockList->numSuperBlocks; i++)
    {
        struct SuperblockEntry *entry = NULL;
        int isOpen = 0;

        // check if sef unit is open
        while ((entry = utl_DListNextAs(&scpm.activeSuperblock, entry, struct SuperblockEntry, link)))
        {
            if (entry->superblock.bits == superBlockList->superBlockRecords[i].flashAddress.bits &&
                entry->parentQosDomain->qosDomainId.id == scpm.selectedQosDomain->qosDomainId.id &&
                entry->parentQosDomain->parentUnit->unitIndex == scpm.selectedSefUnit->unitIndex)
            {
                PyList_SET_ITEM(
                    returnSuperblock, i,
                    Py_BuildValue("(K,O)", superBlockList->superBlockRecords[i].flashAddress.bits,
                                  Py_True));
                isOpen = 1;
                break;
            }
        }

        if (!isOpen)
        {
            PyList_SET_ITEM(returnSuperblock, i,
                            Py_BuildValue("(K,O)", superBlockList->superBlockRecords[i].flashAddress.bits,
                                          Py_False));
        }
    }

    // free dynamic
    free(superBlockList);

    return returnSuperblock;
}

static PyObject *nextFlashAddress(PyObject *self, PyObject *args, PyObject *kwds)
{
    unsigned long long inputFlashAddress;
    struct SEFFlashAddress flashAddress;
    static char *keywords[] = {"FlashAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "K", keywords, &inputFlashAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // release superblock
    flashAddress.bits = inputFlashAddress;
    flashAddress = SEFNextFlashAddress(scpm.selectedQosDomain->qosHandle, flashAddress);

    return Py_BuildValue("K", flashAddress.bits);
}

static PyObject *releaseSuperBlock(PyObject *self, PyObject *args, PyObject *kwds)
{
    unsigned long long inputFlashAddress;
    struct SEFStatus status;
    struct SEFFlashAddress flashAddress;
    struct SuperblockEntry *entry = NULL;
    static char *keywords[] = {"SuperBlockAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "K", keywords, &inputFlashAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // release superblock
    flashAddress.bits = inputFlashAddress;
    status = SEFReleaseSuperBlock(scpm.selectedQosDomain->qosHandle, flashAddress);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to release SuperBlock");
        return NULL;
    }

    // remove from active superblock list
    while ((entry = utl_DListNextAs(&scpm.activeSuperblock, entry, struct SuperblockEntry, link)))
    {
        if (entry->superblock.bits == inputFlashAddress)
        {
            utl_DListRemove(&entry->link);
            free(entry);

            break;
        }
    }

    Py_RETURN_NONE;
}

static PyObject *getSuperBlockInfo(PyObject *self, PyObject *args, PyObject *kwds)
{
    struct SEFSuperBlockInfo sbInfo;
    unsigned long long inputFlashAddress;
    struct SEFStatus status;
    struct SEFFlashAddress flashAddress;
    static char *keywords[] = {"SuperBlockAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "K", keywords, &inputFlashAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get superblock Info
    flashAddress.bits = inputFlashAddress;
    status = SEFGetSuperBlockInfo(scpm.selectedQosDomain->qosHandle, flashAddress, 0, &sbInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get SuperBlock info");
        return NULL;
    }

    PyObject *returnSBInfo;
    returnSBInfo = PyDict_New();

    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "flashAddress"),
                   Py_BuildValue("K", sbInfo.flashAddress));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "eraseOrder"),
                   Py_BuildValue("I", sbInfo.eraseOrder));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "writableADUs"),
                   Py_BuildValue("I", sbInfo.writableADUs));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "writtenADUs"),
                   Py_BuildValue("I", sbInfo.writtenADUs));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "placementID"),
                   Py_BuildValue("i", sbInfo.placementID.id));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "PEIndex"), Py_BuildValue("i", sbInfo.PEIndex));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "PEIndex"), Py_BuildValue("i", sbInfo.PEIndex));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "type"), Py_BuildValue("i", sbInfo.type));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "state"), Py_BuildValue("i", sbInfo.state));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "numDefects"),
                   Py_BuildValue("i", sbInfo.numDefects));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "timeLeft"), Py_BuildValue("i", sbInfo.timeLeft));
    PyDict_SetItem(returnSBInfo, Py_BuildValue("s", "integrity"), Py_BuildValue("i", sbInfo.integrity));

    return returnSBInfo;
}

static PyObject *listRootPointer(PyObject *self)
{
    int i;
    struct SEFStatus status;
    struct SEFQoSDomainInfo qosInfo;
    PyObject *returnFlashAddress;

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(scpm.selectedSefUnit->sefHandle,
                                        scpm.selectedQosDomain->qosDomainId, &qosInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get QoS Domain information");
        return NULL;
    }

    // prepare the return array
    returnFlashAddress = PyList_New(SEFMaxRootPointer);
    for (i = 0; i < SEFMaxRootPointer; i++)
    {
        PyList_SET_ITEM(returnFlashAddress, i, Py_BuildValue("K", qosInfo.rootPointers[i].bits));
    }

    return returnFlashAddress;
}

static PyObject *clearRootPointer(PyObject *self, PyObject *args, PyObject *kwds)
{
    int rootpointerIndex;
    struct SEFStatus status;
    struct SEFQoSDomainInfo qosInfo;
    static char *keywords[] = {"RootPointerIndex", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "i", keywords, &rootpointerIndex))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(scpm.selectedSefUnit->sefHandle,
                                        scpm.selectedQosDomain->qosDomainId, &qosInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get QoS Domain information");
        return NULL;
    }

    if ((rootpointerIndex >= SEFMaxRootPointer) || (rootpointerIndex < 0))
    {
        PyErr_SetString(scpm.cliError, "The root pointer index is not valid");
        return NULL;
    }

    // clear root pointer
    status = SEFSetRootPointer(scpm.selectedQosDomain->qosHandle, rootpointerIndex,
                               (struct SEFFlashAddress){0});
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to clear the root pointers for QoS Domain");
        return NULL;
    }

    Py_RETURN_NONE;
}

static PyObject *setRootPointer(PyObject *self, PyObject *args, PyObject *kwds)
{
    int rootpointerIndex;
    struct SEFStatus status;
    struct SEFQoSDomainInfo qosInfo;
    struct SEFFlashAddress flashAddress;
    unsigned long long inputFlashAddress;
    static char *keywords[] = {"RootPointerIndex", "FlashAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "iK", keywords, &rootpointerIndex, &inputFlashAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(scpm.selectedSefUnit->sefHandle,
                                        scpm.selectedQosDomain->qosDomainId, &qosInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get QoS Domain information");
        return NULL;
    }

    if ((rootpointerIndex >= SEFMaxRootPointer) || (rootpointerIndex < 0))
    {
        PyErr_SetString(scpm.cliError, "The root pointer index is not valid");
        return NULL;
    }

    // set root pointer
    flashAddress.bits = inputFlashAddress;
    status = SEFSetRootPointer(scpm.selectedQosDomain->qosHandle, rootpointerIndex, flashAddress);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to set the root pointers for QoS Domain");
        return NULL;
    }

    Py_RETURN_NONE;
}

static PyObject *listUserAddress(PyObject *self, PyObject *args, PyObject *kwds)
{
    int userAddressListSize, i, isAddressValid;
    struct SEFStatus status;
    struct SEFSuperBlockInfo superblockInfo;
    struct SEFFlashAddress flashAddress;
    struct SEFUserAddressList *userAddressList;
    PyObject *returnUserAddress;
    unsigned long long inputFlashAddress;
    struct SEFSuperBlockList *superBlockList;
    ssize_t listSize;

    static char *keywords[] = {"SuperBlockAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "K", keywords, &inputFlashAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get super block records
    superBlockList = NULL;
    listSize = 0;
    status.info = 0;
    do
    {
        if (status.info > listSize)
        {
            free(superBlockList);
            listSize = status.info;
            superBlockList = malloc(status.info);
        }

        // get list
        status = SEFGetSuperBlockList(scpm.selectedQosDomain->qosHandle, superBlockList, listSize);
        if (status.error)
        {
            free(superBlockList);
            PyErr_SetString(
                scpm.cliError,
                "Error: Was unable to get list of the super blocks owned by the QoS Domain");
            PyErr_SetString(scpm.cliError, SEFStrError("SEFGetSuperBlockList", status.error));
            return NULL;
        }
    } while (status.info);

    isAddressValid = 0;
    flashAddress.bits = inputFlashAddress;
    for (int i = 0; i < superBlockList->numSuperBlocks; i++)
    {
        if (SEFIsEqualFlashAddress(superBlockList->superBlockRecords[i].flashAddress, flashAddress))
        {
            isAddressValid = 1;
        }
    }

    free(superBlockList);

    if (!isAddressValid)
    {
        PyErr_SetString(scpm.cliError, "The Flash Address for the superblock is invalid");
        return NULL;
    }

    // get superblock info
    status = SEFGetSuperBlockInfo(scpm.selectedQosDomain->qosHandle, flashAddress, 0, &superblockInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get superblock information");
        return NULL;
    }

    // get user address list
    userAddressListSize =
        struct_size(userAddressList, userAddressesRecovery, superblockInfo.writableADUs);
    userAddressList = malloc(userAddressListSize);
    status = SEFGetUserAddressList(scpm.selectedQosDomain->qosHandle, flashAddress, userAddressList,
                                   userAddressListSize);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get superblock's user addresses");
        free(userAddressList);
        return NULL;
    }

    // prepare the return array
    returnUserAddress = PyList_New(superblockInfo.writableADUs);
    for (i = 0; i < superblockInfo.writableADUs; i++)
    {
        PyList_SET_ITEM(
            returnUserAddress, i,
            Py_BuildValue("K", le64toh(userAddressList->userAddressesRecovery[i].unformatted)));
    }

    free(userAddressList);

    return returnUserAddress;
}

static PyObject *writeAdu(PyObject *self, PyObject *args, PyObject *kwds)
{
    int numADU, inputPlacementId, i;
    uint64_t inputUserAddress, inputSuperBlockAddress;
    struct SEFPlacementID placementId;
    struct SEFUserAddress userAddress;
    struct SEFFlashAddress superBlockAddress;
    struct SEFQoSDomainInfo qosInfo;
    struct SEFFlashAddress *permanentFlashAddress;
    struct SEFSuperBlockInfo superBlockInfo;
    struct SEFStatus status;
    struct iovec iov;
    Py_buffer inputData;
    PyObject *returnPermanentFlashAddress;
    static char *keywords[] = {"NumAdus",           "Data", "UserAddress", "PlacementId",
                               "SuperBlockAddress", NULL};

    // set the default values
    inputPlacementId = 0;
    inputSuperBlockAddress = SEFAutoAllocate.bits;

    // read values from the arguments
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "iy*K|iK", keywords, &numADU, &inputData,
                                     &inputUserAddress, &inputPlacementId, &inputSuperBlockAddress))
    {
        return NULL;
    }

    // prepare data
    userAddress.unformatted = htole64(inputUserAddress);
    superBlockAddress.bits = inputSuperBlockAddress;
    placementId.id = inputPlacementId;

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(scpm.selectedSefUnit->sefHandle,
                                        scpm.selectedQosDomain->qosDomainId, &qosInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get QoS Domain information");
        return NULL;
    }

    // check dataSize against ADU Count
    if (inputData.len < qosInfo.ADUsize.data * numADU)
    {
        PyErr_SetString(scpm.cliError, "Input data size is less than required amount");
        return NULL;
    }

    // check if dataSize is bigger than available space
    if (!SEFIsEqualFlashAddress(superBlockAddress, SEFAutoAllocate))
    {
        status = SEFGetSuperBlockInfo(scpm.selectedQosDomain->qosHandle, superBlockAddress, 0,
                                      &superBlockInfo);
        if (status.error)
        {
            PyErr_SetString(scpm.cliError, "Was unable to get Super Block Information");
            return NULL;
        }

        if (inputData.len > superBlockInfo.writableADUs * qosInfo.ADUsize.data)
        {
            PyErr_SetString(scpm.cliError,
                            "Input data size is more than available data in Super Block");
            return NULL;
        }
    }

    // prepare the write
    iov.iov_base = inputData.buf;
    iov.iov_len = inputData.len;
    permanentFlashAddress = malloc(sizeof(struct SEFFlashAddress) * numADU);

    // write data to the SEF Unit
    status = SEFWriteWithoutPhysicalAddress(scpm.selectedQosDomain->qosHandle, superBlockAddress,
                                            placementId, userAddress, numADU, &iov, 1, NULL,
                                            permanentFlashAddress, NULL, NULL);
    if (status.error)
    {
        free(permanentFlashAddress);

        PyErr_SetString(scpm.cliError, "Was unable to write to the SEF Unit");
        return NULL;
    }

    // prepare the return array
    returnPermanentFlashAddress = PyList_New(numADU);
    for (i = 0; i < numADU; i++)
    {
        PyList_SET_ITEM(returnPermanentFlashAddress, i,
                        Py_BuildValue("K", permanentFlashAddress[i].bits));
    }

    free(permanentFlashAddress);

    return returnPermanentFlashAddress;
}

static PyObject *readAdu(PyObject *self, PyObject *args, PyObject *kwds)
{
    int numADU;
    uint64_t inputFlashAddress, inputUserAddress;
    struct iovec iov;
    struct SEFQoSDomainInfo qosInfo;
    struct SEFStatus status;
    struct SEFUserAddress userAddress;
    struct SEFFlashAddress flashAddress;
    PyObject *readObj;
    static char *keywords[] = {"NumAdus", "FlashAddress", "UserAddress", NULL};

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "iKK", keywords, &numADU, &inputFlashAddress,
                                     &inputUserAddress))
    {
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        return NULL;
    }

    // get qos domain information
    status = SEFGetQoSDomainInformation(scpm.selectedSefUnit->sefHandle,
                                        scpm.selectedQosDomain->qosDomainId, &qosInfo);
    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to get QoS Domain information");
        return NULL;
    }

    // prepare the read
    flashAddress.bits = inputFlashAddress;
    userAddress.unformatted = htole64(inputUserAddress);
    iov.iov_base = malloc(numADU * qosInfo.ADUsize.data);
    iov.iov_len = numADU * qosInfo.ADUsize.data;

    // read data from the flash
    status = SEFReadWithPhysicalAddress(scpm.selectedQosDomain->qosHandle, flashAddress, numADU,
                                        &iov, 1, 0, userAddress, NULL, NULL);
    if (status.error)
    {
        free(iov.iov_base);

        PyErr_SetString(scpm.cliError, "Was unable to read from the SEF Unit");
        return NULL;
    }

    readObj = Py_BuildValue("y#", iov.iov_base, iov.iov_len);

    free(iov.iov_base);
    return readObj;
}

static PyObject *copyAdus(PyObject *self, PyObject *args, PyObject *kwds)
{
    struct SEFFlashAddress srcFA, dstFA;
    uint32_t numADU, i;
    Py_buffer bitMap;
    struct SEFStatus status;
    PyObject *resultObj = NULL;
    static char *keywords[] = {"srcFA", "dstFA", "bitMap", NULL};
    struct SEFCopySource copySource = {};
    struct SEFAddressChangeRequest *acr = NULL;
    PyObject *addressUpdate;

    if (!PyArg_ParseTupleAndKeywords(args, kwds, "KKy*", keywords, &srcFA.bits, &dstFA.bits, &bitMap))
    {
        PyBuffer_Release(&bitMap);
        return NULL;
    }

    if (!isStatusValid(true, true))
    {
        PyBuffer_Release(&bitMap);
        return NULL;
    }

    // not sure if "y*" returns a contiguous buffer or not
    if (!PyBuffer_IsContiguous(&bitMap, 'C'))
    {
        PyErr_SetString(scpm.cliError, "bitMap is not contiguous");
        PyBuffer_Release(&bitMap);
        return NULL;
    }

    numADU = 8 * bitMap.len;    // all bit sets, max num possible to copy
    // max possible # to copy.
    acr = malloc(sizeof(*acr) + sizeof(acr->addressUpdate[0]) * numADU);

    copySource.format = kBitmap;
    copySource.arraySize = (bitMap.len + 7) / 8;    // rounding up - scary, maybe should copy
    copySource.srcFlashAddress = srcFA;
    copySource.validBitmap = bitMap.buf;

    status = SEFNamelessCopy(scpm.selectedQosDomain->qosHandle, copySource,
                             scpm.selectedQosDomain->qosHandle, dstFA, NULL, NULL, numADU, acr);

    if (status.error)
    {
        PyErr_SetString(scpm.cliError, "Was unable to copy ADUs");
        PyBuffer_Release(&bitMap);
        free(acr);
        return NULL;
    }

    addressUpdate = PyList_New(acr->numProcessedADUs);
    for (i = 0; i < numADU; i++)
    {
        PyObject *entryObj = PyDict_New();

        PyDict_SetItem(entryObj, Py_BuildValue("s", "userAddress"),
                       Py_BuildValue("K", le64toh(acr->addressUpdate[i].userAddress.unformatted)));
        PyDict_SetItem(entryObj, Py_BuildValue("s", "oldFlashAddress"),
                       Py_BuildValue("K", acr->addressUpdate[i].oldFlashAddress.bits));
        PyDict_SetItem(entryObj, Py_BuildValue("s", "newFlashAddress"),
                       Py_BuildValue("K", acr->addressUpdate[i].newFlashAddress.bits));
        PyList_SET_ITEM(addressUpdate, i, entryObj);
    }

    resultObj = PyDict_New();

    PyDict_SetItem(resultObj, Py_BuildValue("s", "numProcessedADUs"),
                   Py_BuildValue("I", acr->numProcessedADUs));
    PyDict_SetItem(resultObj, Py_BuildValue("s", "nextADUOffset"),
                   Py_BuildValue("I", acr->nextADUOffset));
    PyDict_SetItem(resultObj, Py_BuildValue("s", "numReadErrorADUs"),
                   Py_BuildValue("I", acr->numReadErrorADUs));
    PyDict_SetItem(resultObj, Py_BuildValue("s", "copyStatus"), Py_BuildValue("I", acr->copyStatus));
    PyDict_SetItem(resultObj, Py_BuildValue("s", "addressUpdate"), addressUpdate);

    // free memory
    PyBuffer_Release(&bitMap);
    free(acr);

    return resultObj;
}

// python declarations
static PyMethodDef sefCliMethods[] = {
    {"sefCliHelp", (PyCFunction)sefCliHelp, METH_NOARGS,
     "() -> No Return; Prints the help for the SEF-CLI's interactive shell"},
    {"selectSEF", (PyCFunction)selectSEF, METH_VARARGS | METH_KEYWORDS,
     "(SEFIndex) -> No Return; Switches between open SEF Units by selecting one as primary"},
    {"listSEF", (PyCFunction)listSEF, METH_NOARGS,
     "() -> [SEFIndex, ...]; Returns an array of available SEF Units and if they are open"},
    {"openQoS", (PyCFunction)openQoS, METH_VARARGS | METH_KEYWORDS,
     "(QoSDomainId) -> No Return; Opens a QoS Domain and selects it as the primary QoS Domain"},
    {"closeQoS", (PyCFunction)closeQoS, METH_NOARGS,
     "() -> No Return; Closes the primary QoS Domain; All superblocks under the QoS Domain should "
     "be closed before "
     "closing QoS Domain"},
    {"selectQoS", (PyCFunction)selectQoS, METH_VARARGS | METH_KEYWORDS,
     "(QoSDomainId or QoSDomainLabel) -> No Return; Switches between open QoS Domains by selecting "
     "one as primary"},
    {"listQoS", (PyCFunction)listQoS, METH_NOARGS,
     "() -> [(QoSDomainId, QoSDomainLabel, IsQosDomainOpen), ...]; Returns an array of QoS Domains "
     "and if they are open, within the "
     "selected SEF Unit"},
    {"allocateSuperBlock", (PyCFunction)allocateSuperBlock, METH_VARARGS | METH_KEYWORDS,
     "(SuperBlockType) -> (SuperBlockAddress); Allocates a new superblock to be used for write"},
    {"closeSuperBlock", (PyCFunction)closeSuperBlock, METH_VARARGS | METH_KEYWORDS,
     "(SuperBlockAddress) -> No Return; Closes the manually allocated selected superblock"},
    {"listSuperBlock", (PyCFunction)listSuperBlock, METH_NOARGS,
     "() -> [(FlashAddress, IsManuallyWritable), ...]; Returns an array of SuperBlocks and if they "
     "can be written to, within "
     "the selected QoS Domain"},
    {"nextFlashAddress", (PyCFunction)nextFlashAddress, METH_VARARGS | METH_KEYWORDS,
     "(FlashAddress) -> (FlashAddress) ; Returns the next flash address"},
    {"releaseSuperBlock", (PyCFunction)releaseSuperBlock, METH_VARARGS | METH_KEYWORDS,
     "(SuperBlockAddress) -> No Return; Release the superblock with te selected QoS Domain"},
    {"getSuperBlockInfo", (PyCFunction)getSuperBlockInfo, METH_VARARGS | METH_KEYWORDS,
     "(SuperBlockAddress) -> {FlashAddress, eraseOrder, ...}; Returns information about a "
     "superblock"},
    {"listRootPointer", (PyCFunction)listRootPointer, METH_NOARGS,
     "() -> [FlashAddress, ...]; Returns an array of root pointers for the selected QoS Domain"},
    {"clearRootPointer", (PyCFunction)clearRootPointer, METH_VARARGS | METH_KEYWORDS,
     "(RootPointerIndex) -> No Return; Clears root pointer for the selected QoS Domain"},
    {"setRootPointer", (PyCFunction)setRootPointer, METH_VARARGS | METH_KEYWORDS,
     "(RootPointerIndex, FlashAddress) -> No Return; Sets root pointer for the selected QoS "
     "Domain"},
    {"listUserAddress", (PyCFunction)listUserAddress, METH_VARARGS | METH_KEYWORDS,
     "(SuperBlockAddress) -> [UserAddress, ...]; Returns an array of user addresses in a Super "
     "block"},
    {"writeAdu", (PyCFunction)writeAdu, METH_VARARGS | METH_KEYWORDS,
     "(NumAdus, Data, UserAddress, <PlacementId>, <SuperBlockAddress>) -> [FlashAddress, ...]; "
     "Write an ADU to an auto "
     "allocated superblock"},
    {"readAdu", (PyCFunction)readAdu, METH_VARARGS | METH_KEYWORDS,
     "(NumAdus, FlashAddress, UserAddress) -> (BinaryData); Read from a SEF Unit"},
    {"copyAdus", (PyCFunction)copyAdus, METH_VARARGS | METH_KEYWORDS,
     "(<srcFA>,<dstFA>,<bitMap>) -> "
     "{numProcessed, nextAduOffset, numReadErrors ,copyStatus, [{UA,oldFA,newFA},...]}"},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef sefCliModule = {PyModuleDef_HEAD_INIT, "sefCli", "", -1, sefCliMethods};

static /*PyMODINIT_FUNC*/ PyObject *PyInit_sefCli(void)
{
    PyObject *sefCliObj;

    sefCliObj = PyModule_Create(&sefCliModule);
    if (sefCliObj == NULL)
    {
        return NULL;
    }

    scpm.cliError = PyErr_NewException("sefCli.error", NULL, NULL);
    Py_XINCREF(scpm.cliError);
    if (PyModule_AddObject(sefCliObj, "error", scpm.cliError) < 0)
    {
        Py_XDECREF(scpm.cliError);
        Py_CLEAR(scpm.cliError);
        Py_DECREF(sefCliObj);
        return NULL;
    }

    return sefCliObj;
}

// engine prototypes
static int EPSInteractive(void **context, void *options);

static int EPSExecute(void **context, void *options);

// engine options
struct EPSOptions
{
    void *pad;
    char *pythonScript;
    bool pythonScriptGiven;
    bool scriptPipe;
};

static struct CEOEngineOption optionsCEO[] = {
    CEOString('-',
              "python-script",
              "The Python Script to be executed by the SEF CLI Shell",
              EPSOptions,
              pythonScript,
              pythonScriptGiven),
    CEOFlag('-', "script-pipe", "", EPSOptions, scriptPipe)};

// engine setup/cleanup
static int EPSSetup(void **engineContext, void *engineOptions)
{
    int statusInt;
    struct SEFStatus status;

    // init global python object
    scpm.selectedSefUnit = NULL;
    scpm.selectedQosDomain = NULL;
    utl_DListInit(&scpm.activeSefUnit);
    utl_DListInit(&scpm.activeQosDomain);
    utl_DListInit(&scpm.activeSuperblock);

    // init sef library
    status = SEFLibraryInit();
    if (status.error)
    {
        CIHMessageError("Error: Was unable to init the SEF Library");
        CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFLibraryInit", status.error));
        return EIO;
    }

    if (!status.info)
    {
        CIHMessageError("Error: Was unable to find any SEF Units");
        return ENODEV;
    }

    // assign sef unit count
    scpm.numSEFUnit = status.info;

    // init python module
    scpm.pythonProgram = Py_DecodeLocale("sef-cli", NULL);
    Py_SetProgramName(scpm.pythonProgram);

    // register sefCli python module
    statusInt = PyImport_AppendInittab("sefCli", PyInit_sefCli);
    if (statusInt)
    {
        CIHMessageError("Error: Was unable to register the python module");
        return -EIO;
    }

    Py_Initialize();

    // import and set up python
    PyRun_SimpleString("from sefCli import *");

    return 0;
}

static int EPSCleanup(void **engineContext, void *engineOptions)
{
    struct SefUnitEntry *sefUnitEntry = NULL;
    struct QosDomainEntry *qosDomainEntry = NULL;
    struct SuperblockEntry *superblockEntry = NULL;

    // clean up python
    if (Py_IsInitialized())
    {
        int statusInt;

        // clean up sef's python module
        statusInt = PyState_RemoveModule(&sefCliModule);
        if (statusInt)
        {
            CIHMessageError("Error: Was unable to clean up the python sef-cli module");
        }
#if PY_MINOR_VERSION > 5    // Python 3.6+
        statusInt = Py_FinalizeEx();
        if (statusInt)
        {
            CIHMessageError("Error: Was unable to clean up the python embedded module");
        }
#else
        Py_Finalize();
#endif
    }

    PyMem_RawFree(scpm.pythonProgram);

    // clean python object and close SEF instances
    while ((superblockEntry = utl_DListPopHeadAs(&scpm.activeSuperblock, struct SuperblockEntry, link)))
    {
        // close sef superblock
        struct SEFStatus status = SEFCloseSuperBlock(superblockEntry->parentQosDomain->qosHandle,
                                                     superblockEntry->superblock);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to close Superblock");
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseSuperBlock", status.error));

            free(superblockEntry);
        }
    }

    while ((qosDomainEntry = utl_DListPopHeadAs(&scpm.activeQosDomain, struct QosDomainEntry, link)))
    {
        // close sef qos domain
        struct SEFStatus status = SEFCloseQoSDomain(qosDomainEntry->qosHandle);
        if (status.error)
        {
            CIHMessageError("Error: Was unable to close QoS Domain %d", qosDomainEntry->qosDomainId.id);
            CIHMessageInfo("Detailed Error: %s", SEFStrError("SEFCloseQoSDomain", status.error));
        }
        free(qosDomainEntry);
    }

    while ((sefUnitEntry = utl_DListPopHeadAs(&scpm.activeSefUnit, struct SefUnitEntry, link)))
    {
        free(sefUnitEntry);
    }

    SEFLibraryCleanup();

    return 0;
}

// engine functions
static void printPythonHelp()
{
    int i = 0;

    CIHMessageData(
        "The interactive shell works by targeting a selected SEF Unit, QoS Domain or SuperBlock");
    CIHMessageData("You can open and select units using open and allocate functions.");
    CIHMessageData("You also have the option to switch between open units using select functions.");

    CIHMessageData("\nsefCli interactive mode supports the following functions:");

    CIHMessageDataNoWrap("*  %-20s %s", "quit", "() -> No Return");
    CIHMessageDataNoWrap("   %-20s %s\n", "", "Exists the interactive shell");

    while (sefCliMethods[i].ml_name != NULL)
    {
        char *doc, *tok, *rest;
        doc = strdup(sefCliMethods[i].ml_doc);
        rest = doc;

        tok = strtok_r(rest, ";", &rest);
        CIHMessageDataNoWrap("*  %-20s %s", sefCliMethods[i].ml_name, tok);

        tok = strtok_r(rest, ";", &rest);
        CIHMessageDataNoWrap("   %-20s%s\n", "", tok);

        free(doc);

        i++;
    }
}

static int EPSInteractive(void **context, void *options)
{
    // import and set up python shell
    PyRun_SimpleString("import readline");
    PyRun_SimpleString("import rlcompleter");
    PyRun_SimpleString("readline.parse_and_bind(\"tab: complete\")");

    // print info on the screen
    CIHMessageData(">>> sefCli has been loaded, use sefCliHelp() to learn about its use cases");
    CIHMessageData(">>> use function quit() to exit the interactive mode");
    updateStatusLine();

    // start python interactive shell
    PyRun_InteractiveLoop(stdin, "stdin");

    return 0;
}

static int EPSExecute(void **context, void *options)
{
    int statusInt;
    struct EPSOptions *sho = (struct EPSOptions *)options;

    if (!sho->scriptPipe && CIHIsNotGiven("python-script", '-', sho->pythonScriptGiven))
    {
        return EINVAL;
    }

    if (!sho->scriptPipe)
    {
        FILE *fptr;

        // open input file
        fptr = fopen(sho->pythonScript, "rb");
        if (fptr == NULL)
        {
            CIHMessageError("Error: Was unable to access the file %s", sho->pythonScript);
            return -EIO;
        }

        // start python interactive shell
        statusInt = PyRun_SimpleFile(fptr, sho->pythonScript);
        if (statusInt)
        {
            CIHMessageError("Error: Was unable to execute the python script");
        }

        // close input file
        fclose(fptr);
    }
    else
    {
        // start python interactive shell
        statusInt = PyRun_SimpleFile(stdin, "stdin");
        if (statusInt)
        {
            CIHMessageError("Error: Was unable to execute the python script");
        }
    }

    return 0;
}

// engine declarations
static struct CEHEngineAction actionsCEO[] = {
    newEngineAction(
        "interactive", "An Interactive shell to interact with the device in realtime", &EPSInteractive),
    newEngineAction("execute", "Execute a Python script to interact with the device", &EPSExecute),
};

static struct CEHEngineConfig configCEO = newEngineWithSetup(
    "shell",
    "A Python Shell extended with SEF functionality for interaction with the device",
    EPSOptions,
    optionsCEO,
    actionsCEO,
    &EPSSetup,
    &EPSCleanup);

// registering the engine
static void sefEngineInit shellInit()
{
    CEHRegisterEngine(&configCEO);
}
