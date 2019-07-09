/*
 Copyright (c) 2012 Arduino LLC.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <MassStorage.h>

uint8_t wait(uint8_t tries, uint32_t delay_time) {
    if (0 == tries)
        return 1;
    delay(delay_time);
    return 0;
}

uint32_t MassStorage::bulkInTransfer(EpInfo *pep, uint32_t nak_limit,
        uint32_t *nbytesptr, uint8_t* data) {
    uint32_t usberr, tries = nak_limit, delay_wait = 100;
    if (nullptr == nbytesptr || nullptr == data)
        return MASS_STORAGE_NULL_PARAM;
    TRACE_USBHOST_SERIAL3(Serial.println("bulkIntransfer: perform it"););

    do {
        while((usberr = pUsb->inTransfer(bAddress, pep->deviceEpNum,
                        nbytesptr, data) == USB_ERROR_HOST_BUSY)) {
            if (wait(tries--, delay_wait)) {
                TRACE_USBHOST_SERIAL3(Serial.println("bulkIntransfer:host is busy couldnt send scsi write cmd:"););
                return usberr;
            }
        }

        if (BULK_ERR_NAK == usberr) {
            TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction:: Error is 1");)
            if (Is_uhd_pipe_frozen(pep->hostPipeNum)) {
                TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction:: unfreezing the pipe");)
                uhd_unfreeze_pipe(pep->hostPipeNum);
            }
            if (--nak_limit == 0)
                return BULK_ERR_NAK;
            delay(100);
        } else
            return usberr;
    } while (1);
    TRACE_USBHOST_SERIAL3(Serial.println("Intransfer:Number of bytes read");)
    TRACE_USBHOST_SERIAL3(Serial.println(*nbytesptr);)
    return BULK_SUCCESS;
}



void MassStorage::create_cbw_packet_cdb6(MS_CommandBlockWrapper_t *cbw,
        uint32_t tag, uint32_t transfer_len, SCSI_CDB6_t *cdb,
        uint8_t direction) {
    cbw->signature = MS_CBW_SIGNATURE;
    cbw->tag = tag;
    cbw->lun = 0; // keep the commands to lun 0
    cbw->dataTransferLength = transfer_len;
    cbw->flags = direction;
    cbw->SCSICommandLength = transfer_len;
    memcpy(cbw->SCSICommandData, cdb, cbw->SCSICommandLength);
}


MassStorage::MassStorage(USBHost *p) : pUsb(p)
{

    // Initialize endpoint data structures
    for (uint32_t i = 0; i < MS_MAX_ENDPOINTS; ++i)
    {
        epInfo[i].deviceEpNum   = 0;
        epInfo[i].hostPipeNum   = 0;
        epInfo[i].maxPktSize    = (i) ? 0 : 8;
        epInfo[i].epAttribs     = 0;
        epInfo[i].bmNakPower    = (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
    }
    cbwTag = 0;
    bMaxLUN = 0;
    bTheLUN = 0;
    qNextPollTime = 0;
    bPollEnable = false;
    // always have control endpoint
    bNumEP = 1;
    bConfNum = 0;
    bIfaceNum = 0;
    bNumIface = 0;
    // start with 0 endpoint
    bAddress = 0;

    for(uint8_t i = 0; i < MS_MAX_SUPPORTED_LUN; i++) {
        //LUNOk[i] = false;
        //WriteOk[i] = false;
        // 32 bits
        for (uint8_t j = 0; j < 4; j++) {
            currentCapacity[i].blocks[j] = 0;
            currentCapacity[i].blockSize[j] = 0;
        }
    }
    // Register in USB subsystem
    if (pUsb)
    {
        pUsb->RegisterDeviceClass(this);
    }
}

/**
 * \brief Initialize connection to a Mass Storage device
 *
 * \param parent USB device address of the Parent device.
 * \param port USB device base address.
 * \param lowspeed USB device speed.
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t MassStorage::Init(uint32_t parent, uint32_t port, uint32_t lowspeed)
{

    const uint32_t constBufSize = sizeof(USB_DEVICE_DESCRIPTOR);
    uint8_t     buf[constBufSize];
    uint32_t    rcode = 0;
    UsbDevice   *p = 0;
    EpInfo      *oldep_ptr = 0;
    uint32_t    len = 0;

    uint32_t    num_of_conf = 0;    // number of configurations

    AddressPool &addrPool = pUsb->GetAddressPool();

    TRACE_USBHOST_SERIAL3(Serial.println("MassStorage::Init\r\n");)
    if (bAddress)
        return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;

    // Get pointer to pseudo device with address 0 assigned
    p = addrPool.GetUsbDevicePtr(0);

    if (!p)
        return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

    if (!p->epinfo)
    {
        TRACE_USBHOST_SERIAL3(Serial.println("MS::Init : epinfo is null!\r\n");)
        return USB_ERROR_EPINFO_IS_NULL;
    }
    // Save old pointer to EP_RECORD of address 0
    oldep_ptr = p->epinfo;

    // Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
    p->epinfo = epInfo;

    p->lowspeed = lowspeed;
    // Get device descriptor
    rcode = pUsb->getDevDescr(0, 0, constBufSize, (uint8_t*)buf);
    if (!rcode)
        len = (buf[0] > constBufSize) ? constBufSize : buf[0];

    if (rcode)
    {
        // Restore p->epinfo
        p->epinfo = oldep_ptr;
        goto FailGetDevDescr;
    }

    // Restore p->epinfo
    p->epinfo = oldep_ptr;

    // Allocate new address according to device class
    bAddress = addrPool.AllocAddress(parent, false, port);

    if (!bAddress)
        return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

    // Extract Max Packet Size from the device descriptor
    epInfo[0].maxPktSize = (uint8_t)((USB_DEVICE_DESCRIPTOR*)buf)->bMaxPacketSize0;

    // Assign new address to the device
    rcode = pUsb->setAddr(0, 0, bAddress);

    if (rcode)
    {
        p->lowspeed = false;
        addrPool.FreeAddress(bAddress);
        bAddress = 0;
        TRACE_USBHOST_SERIAL3(Serial.println("MassStorage::Init : setAddr failed with rcode \r\n");)
        TRACE_USBHOST_SERIAL3(Serial.println(rcode);)
        return rcode;
    }
    TRACE_USBHOST_SERIAL3(Serial.println("MassStorage::Init : device address is now");)
    TRACE_USBHOST_SERIAL3(Serial.println(bAddress);)
    p->lowspeed = false;

    p = addrPool.GetUsbDevicePtr(bAddress);

    if (!p)
        return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

    p->lowspeed = lowspeed;
    if (len)
        rcode = pUsb->getDevDescr(bAddress, 0, len, (uint8_t*)buf);

    if(rcode)
        goto FailGetDevDescr;

    num_of_conf = ((USB_DEVICE_DESCRIPTOR*)buf)->bNumConfigurations;
    TRACE_USBHOST_SERIAL3(Serial.println("MassStorage::Init : number of configuration is \r\n");)
    TRACE_USBHOST_SERIAL3(Serial.println(num_of_conf);)

    // Assign epInfo to epinfo pointer
    // its going to assign usbdevice the ep info entry with num of endpoints to
    // be 1 as control ep has been added
    rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
    for (uint32_t i = 0; i < num_of_conf; ++i)
    {
        ConfigDescParser<USB_CLASS_MASS_STORAGE,
                         UHS_BULK_SUBCLASS_SCSI,
                         UHS_STOR_PROTO_BBB, // bulk only transport
                         0> confDescrParser(this);

        // this code figures out the bNumEP
        rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);

        if (rcode)
        {
            goto FailGetConfDescr;
        }

    }
    if (bNumEP < 2)
        return USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;
    // Assign epInfo to epinfo pointer - this time all 3 endpoins
    rcode = pUsb->setEpInfoEntry(bAddress, bNumEP, epInfo);
    if (rcode)
        goto FailSetDevTblEntry;
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Init : bConfNum");)
    TRACE_USBHOST_SERIAL3(Serial.println(bConfNum);)

    // Set Configuration Value
    // determines which configuration to set
    rcode = pUsb->setConf(bAddress, 0, bConfNum);

    if (rcode)
        goto FailSetConfDescr;

    TRACE_USBHOST_SERIAL3(Serial.println("MS::Init : bIfaceNum");)
    TRACE_USBHOST_SERIAL3(Serial.println(bIfaceNum);)

    TRACE_USBHOST_SERIAL3(Serial.println("MassStorage::Init: device configured successfully\r\n");)
    bPollEnable = true;
    GetMaxLUN(&bMaxLUN);
    if (bMaxLUN == 0) {
        TRACE_USBHOST_SERIAL3(Serial.println("ERROR:MS::maxLun reported 0\r\n");)
    }
    return 0;

FailGetDevDescr:
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Init getDevDescr : ");)
    goto Fail;

FailSetDevTblEntry:
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Init setDevTblEn : ");)
    goto Fail;

FailGetConfDescr:
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Init getConf : ");)
    goto Fail;

FailSetConfDescr:
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Init setConf : ");)
    goto Fail;

Fail:
    TRACE_USBHOST_SERIAL3(Serial.println("error code");)
    TRACE_USBHOST_SERIAL3(Serial.println(rcode);)
    Release();
    return rcode;
}

/**
 * \brief Extract bulk-IN and bulk-OUT endpoint information from configuration
 * descriptor.
 *
 * \param conf Configuration number.
 * \param iface Interface number.
 * \param alt Alternate setting.
 * \param proto Protocol version used.
 * \param pep Pointer to endpoint descriptor.
 */
void MassStorage::EndpointXtract(uint32_t conf, uint32_t iface, uint32_t alt,
                                 uint32_t proto, const USB_ENDPOINT_DESCRIPTOR *pep)
{
    if (bNumEP == MS_MAX_ENDPOINTS) {
        TRACE_USBHOST_SERIAL3(Serial.println("Completed extraction of endpoints. Total ep:");)
        return;
    }

    bConfNum = conf;
    bIfaceNum = iface;
    uint32_t index = 0;
    uint32_t pipe = 0;

    // ctrl EP address is allocated in ctrlReq so no need to allocate here
    //
    if ((pep->bmAttributes & 0x03) == USB_TRANSFER_TYPE_BULK)
        index = ((pep->bEndpointAddress & 0x80) == USB_SETUP_DEVICE_TO_HOST) ? \
                bEpDataInIndex : bEpDataOutIndex;
    else if (((pep->bmAttributes & 0x03) == USB_TRANSFER_TYPE_CONTROL) && \
            (pep->bEndpointAddress & 0x80) == USB_SETUP_HOST_TO_DEVICE) {
        // code shouldnt reach here!!!
        TRACE_USBHOST_SERIAL3(Serial.println("ERROR:MS::EndpointXtract : Unknwon endpoint\r\n");)
        return;
    }
    else {
        TRACE_USBHOST_SERIAL3(Serial.println("ERROR:MS::EndpointXtract : Unknwon endpoint\r\n");)
        return;
    }

    // Fill in the endpoint info structure
    epInfo[index].deviceEpNum = pep->bEndpointAddress & 0x0F;
    epInfo[index].maxPktSize = pep->wMaxPacketSize;


    TRACE_USBHOST_SERIAL3(Serial.println("MS::EndpointXtract : Found new endpoint\r\n");)
    TRACE_USBHOST_SERIAL3(Serial.println("MS::EndpointXtract : deviceEpNum\r\n");)
    TRACE_USBHOST_SERIAL3(Serial.println(epInfo[index].deviceEpNum);)
    TRACE_USBHOST_SERIAL3(Serial.println("MS::EndpointXtract : maxPktSize:");)
    TRACE_USBHOST_SERIAL3(Serial.println(epInfo[index].maxPktSize);)
    TRACE_USBHOST_SERIAL3(Serial.println("MS::EndpointXtract : index:");)
    TRACE_USBHOST_SERIAL3(Serial.println(index);)

    // single bank pipe UOTGHS_HSTPIPCFG_PBK_1_BANK
    // This indicates that the pipe should have one single bank, which
    // requires less USB FIFO memory but  results in slower transfers as only
    // one USB device (the LPC or the attached device) can access the pipe's
    // bank at the one time
    if (index == bEpDataInIndex)
        pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum,
                UOTGHS_HSTPIPCFG_PTYPE_BLK, UOTGHS_HSTPIPCFG_PTOKEN_IN,
                epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);
    else if (index == bEpDataOutIndex)
        pipe = UHD_Pipe_Alloc(bAddress, epInfo[index].deviceEpNum,
                UOTGHS_HSTPIPCFG_PTYPE_BLK, UOTGHS_HSTPIPCFG_PTOKEN_OUT,
                epInfo[index].maxPktSize, 0, UOTGHS_HSTPIPCFG_PBK_1_BANK);

    // Ensure pipe allocation is okay
    if (0 == pipe)
    {
        TRACE_USBHOST_SERIAL3(Serial.println("MS::EndpointXtract : Pipe allocation failure\r\n");)
        // Enumeration failed, so user should not perform write/read since isConnected will return false
        return;
    }
    epInfo[index].hostPipeNum = pipe;
    bNumEP++;
}

/**
 * \brief Release USB allocated resources (pipes and address).
 *
 * \note Release call is made from USBHost.task() on disconnection events.
 * \note Release call is made from Init() on enumeration failure.
 *
 * \return Always 0.
 */
uint32_t MassStorage::Release()
{
    // Free allocated host pipes
    UHD_Pipe_Free(epInfo[bEpDataInIndex].hostPipeNum);
    UHD_Pipe_Free(epInfo[bEpDataOutIndex].hostPipeNum);

    // Free allocated USB address
    pUsb->GetAddressPool().FreeAddress(bAddress);

    bConfNum      = 0;
    bNumIface     = 0;
    bIfaceNum     = 0;
    // in this driver the index starts from 1
    // as controller doesnt need to be allocated ??
    bNumEP        = 1;
    bAddress      = 0;
    qNextPollTime = 0;
    bPollEnable   = false;
    return 0;
}

/**
 *  Resets the hdd
 */
void MassStorage::Reset(void) {
    if(!bAddress)
        return;
    uint8_t tries = NAK_LIMIT;
    while(pUsb->ctrlReq(bAddress, 0, bmREQ_MS_OUT,
          REQ_MassStorageReset, 0, 0, bIfaceNum, 0, 0, nullptr,
          nullptr) != 0x0) {
            delay(100);
            tries--;
            if (tries == 0) {
                TRACE_USBHOST_SERIAL3(Serial.println("\r\nCouldnt reset the drive\r\n"));
                break;
            }
    }
    delay(2000);
}

/**
 *
 * @param plun
 * @return
 */
uint8_t MassStorage::GetMaxLUN(uint8_t *plun) {
    if(!bAddress)
        return BULK_ERR_DEVICE_DISCONNECTED;
    TRACE_USBHOST_SERIAL3(Serial.println("Called GetMaxLUN. bIfaceNum"));
    TRACE_USBHOST_SERIAL3(Serial.println(bIfaceNum));
    pUsb->setConf(bAddress, 0, bConfNum);
    uint32_t rcode = pUsb->ctrlReq(bAddress, 0, bmREQ_MS_IN, REQ_GetMaxLUN,
                                   0x0, 0, bIfaceNum, 1, 1, plun, nullptr);
    if(rcode == HOST_ERROR_STALL) {
        *plun = 0;
        TRACE_USBHOST_SERIAL3(Serial.println("\r\nGetMaxLUN Stalled\r\n"));
    }
    TRACE_USBHOST_SERIAL3(Serial.println("Finished GetMaxLUN. lun:"));
    TRACE_USBHOST_SERIAL3(Serial.println(*plun));
    return 0;
}


void printCSW(MS_CommandStatusWrapper_t *pcsw) {
    TRACE_USBHOST_SERIAL3(Serial.println("signature:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcsw->signature);)
    TRACE_USBHOST_SERIAL3(Serial.println("tag:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcsw->tag);)
    TRACE_USBHOST_SERIAL3(Serial.println("dataTransferResidue:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcsw->dataTransferResidue);)
    TRACE_USBHOST_SERIAL3(Serial.println("status:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcsw->status);)
    TRACE_USBHOST_SERIAL3(Serial.println("Done printing CSW");)
}

void printCBW(MS_CommandBlockWrapper_t *pcbw) {
    TRACE_USBHOST_SERIAL3(Serial.println("signature:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcbw->signature);)
    TRACE_USBHOST_SERIAL3(Serial.println("tag:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcbw->tag);)
    TRACE_USBHOST_SERIAL3(Serial.println("dataTransferLength:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcbw->dataTransferLength);)
}
/**
 * For driver use only.
 *
 * @param pcsw
 * @param pcbw
 * @return
 */
bool MassStorage::IsValidCSW(MS_CommandStatusWrapper_t *pcsw,
                             MS_CommandBlockWrapper_t *pcbw) {
    if(!bAddress)
        return false;
    printCSW(pcsw);
    TRACE_USBHOST_SERIAL3(Serial.println(pcsw->tag);)
    TRACE_USBHOST_SERIAL3(Serial.println(pcbw->tag);)
    if(pcsw->signature != MS_CSW_SIGNATURE) {
        TRACE_USBHOST_SERIAL3(Serial.println("MS::csw signature error:");)
        TRACE_USBHOST_SERIAL3(Serial.println(pcsw->tag);)
        return false;
    }
    if(pcsw->tag != pcbw->tag) {
        TRACE_USBHOST_SERIAL3(Serial.println("MS::csw wrong tag csw: cbw:");)
        TRACE_USBHOST_SERIAL3(Serial.println(pcbw->tag);)
        return false;
    }
    return true;
}

/**
 * For driver use only.

/**
 *
 * @param pcbw
 * @param buf_size
 * @param buf
 * @return
 */
uint8_t MassStorage::Transaction(MS_CommandBlockWrapper_t *pcbw,
                                 uint16_t buf_size, void *buf) {
    if(!bAddress)
        return BULK_ERR_DEVICE_DISCONNECTED;
    uint16_t bytes = buf_size;
    bool write = (pcbw->flags & USB_SETUP_DEVICE_TO_HOST) != USB_SETUP_DEVICE_TO_HOST;
    uint32_t usberr, read_bytes;
    uint32_t num_tries = NAK_LIMIT;
    uint32_t tries = num_tries;
    MS_CommandStatusWrapper_t csw;
    // set current lun, might be racy
    bTheLUN = pcbw->lun;
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction:dcbwTag:");)
    TRACE_USBHOST_SERIAL3(Serial.println(pcbw->tag));

    while ((usberr = pUsb->outTransfer(bAddress,
                    epInfo[bEpDataOutIndex].deviceEpNum,
                    sizeof(MS_CommandBlockWrapper_t),
                    (uint8_t*)pcbw)) == USB_ERROR_HOST_BUSY) {
        if (wait(tries--, 100)) {
            TRACE_USBHOST_SERIAL3(Serial.println("\r\nMS:host is busy couldnt send scsi cmd");)
            TRACE_USBHOST_SERIAL3(Serial.println(usberr);)
            break;
        }
    }
    tries = num_tries;

    if(usberr) {
        TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction::Host busy error on CBW");)
        TRACE_USBHOST_SERIAL3(Serial.println(usberr);)
        return usberr;
    }
    else {
        if(bytes) {
            if(!write) {
                TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction:Executing IN command");)
                usberr = bulkInTransfer(&epInfo[bEpDataInIndex], NAK_LIMIT, &read_bytes,
                                        (uint8_t *)buf);
            } else {
                TRACE_USBHOST_SERIAL3(Serial.print("MS::Transaction:Executing OUT command: ");)
                TRACE_USBHOST_SERIAL3(Serial.println(bytes);)
                for (int o = 0 ; o < bytes; o++) {
                    TRACE_USBHOST_SERIAL3(Serial.print(" "););
                    TRACE_USBHOST_SERIAL3(Serial.print(*(((uint8_t *)buf) + o)););
                }
                TRACE_USBHOST_SERIAL3(Serial.println(" ");)
                while((usberr = pUsb->outTransfer(bAddress, epInfo[bEpDataOutIndex].deviceEpNum, bytes,
                                (uint8_t*)buf)) == USB_ERROR_HOST_BUSY) {
                    if (wait(tries--, 100)) {
                        TRACE_USBHOST_SERIAL3(Serial.println("\r\nMS:host is busy couldnt send scsi write cmd:"););
                        break;
                    }
                }
            }
        }
    }
    if (usberr)
        return usberr;
    uint32_t read = sizeof(csw);
    TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction:: Reading CSW");)
    TRACE_USBHOST_SERIAL3(Serial.println(bEpDataInIndex);)
    usberr = bulkInTransfer(&epInfo[bEpDataInIndex], NAK_LIMIT, &read, (uint8_t *)&csw);
    if(usberr)
        return usberr;
    if(IsValidCSW(&csw, pcbw)) {
        TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction:Correct CSW"););
        if (csw.status != SCSI_Command_Pass) {
            TRACE_USBHOST_SERIAL3(Serial.println("MS::Transaction::Status was not successful"););
            TRACE_USBHOST_SERIAL3(Serial.println(csw.status);)
        }
        return csw.status;
    } else {
        Reset();
        return BULK_ERR_INVALID_CSW;
    }
    return 0;
}


// disable polling
void MassStorage::disablePoll() {
    noInterrupts();
    bPollEnable = false;
    __DSB();
    interrupts();
}

// enable polling
void MassStorage::enablePoll() {
    noInterrupts();
    bPollEnable = true;
    __DSB();
    interrupts();
}

void MassStorage::fill_scsi_cdb6(SCSI_CDB6_t *cdb,  uint8_t opcode,
                    uint8_t lun, uint32_t lba,
                    uint8_t allocation_length, uint8_t control) {
    if (nullptr == cdb)
        return;
    cdb->opcode = opcode;
    cdb->LUN = lun;
    cdb->LBAMSB = UINT8_BYTE2(lba) & 0x1f;
    cdb->LBAHB = UINT8_BYTE1(lba);
    cdb->LBALB = UINT8_BYTE0(lba);
    cdb->LBALB = UINT8_BYTE0(lba);
    cdb->control = control;
    cdb->allocationLength = allocation_length;
}

/**
 *
 * @param lun Logical Unit Number
 * @return
 */
uint8_t MassStorage::TestUnitReady(uint8_t lun) {
        if(!bAddress)
            return BULK_ERR_UNIT_NOT_READY;
        TRACE_USBHOST_SERIAL3(Serial.println("MS::TestUnitReady Address Assigned \r\n"););


        SCSI_CDB6_t cdb;
        fill_scsi_cdb6(&cdb, SCSI_CMD_TEST_UNIT_READY, lun, 0, 0, 0);

        return SCSITransaction6(&cdb, 0, NULL, MS_COMMAND_DIR_DATA_IN);
}

/**
 *
 * @param lun Logical Unit Number
 * @return
 */
uint8_t MassStorage::omgDoCAN(uint8_t *can_frame, uint32_t can_frame_size) {
    TRACE_USBHOST_SERIAL3(Serial.println("MS::omgDoCAN"););
    if(!bAddress)
        return BULK_ERR_UNIT_NOT_READY;
    if (can_frame_size > MAX_CAN_SIZE)
        return BULK_ERR_OOB_CAN_SIZE;

    SCSI_CDB6_t cdb;
    fill_scsi_cdb6(&cdb, SCSI_CMD_CAN_PASSTHROUGH, 0, 0, 0, 0);
    TRACE_USBHOST_SERIAL3(Serial.println("MS::fill scsi cdb6"););

    return SCSITransaction6(&cdb, can_frame_size, can_frame,
                            MS_COMMAND_DIR_DATA_OUT);
}

/**
 * Wrap and execute a SCSI CDB with length of 6
 *
 * @param cdb CDB to execute
 * @param buf_size Size of expected transaction
 * @param buf Buffer
 * @param dir MASS_CMD_DIR_IN | MASS_CMD_DIR_OUT
 * @return
 */
uint8_t MassStorage::SCSITransaction6(SCSI_CDB6_t *cdb, uint16_t buf_size,
                                      void *buf, uint8_t dir) {
        if(!bAddress)
            return BULK_ERR_DEVICE_DISCONNECTED;
        disablePoll();
        // promote buf_size to 32bits.
        MS_CommandBlockWrapper_t cbw;
        create_cbw_packet_cdb6(&cbw, ++cbwTag, (uint32_t)buf_size, cdb, dir);

        uint8_t v = Transaction(&cbw, buf_size, buf);
        enablePoll();
        TRACE_USBHOST_SERIAL3(Serial.println("MS::finished scsi transaction 6"););
        return v;
}


/**
 * \brief Poll USB device activity.
 *
 * \note Poll call is periodically made from USBHost.task().
 *
 * \return 0 on success, error code otherwise.
 */
uint32_t MassStorage::Poll() {
    uint32_t rcode = 0;

    if (!bPollEnable)
        return rcode;
    return rcode;
}
