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

#ifndef MASS_STORAGE_H
#define MASS_STORAGE_H

#include <stdint.h>
#include "usb_ch9.h"
#include "Usb.h"
#include <Arduino.h>
#include "SCSI.h"
#include "confdescparser.h"
#define Serial Serial3

// Requests type
#define bmREQ_MS_IN     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_MS_OUT     USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE


/* Bulk error codes */
#define HOST_ERROR_NONE 0x0U
#define HOST_ERROR_UNPLUGGED 0xDEU
#define HOST_ERROR_STALL 0x5U
#define              BULK_ERR_SUCCESS HOST_ERROR_NONE
#define          BULK_ERR_PHASE_ERROR 0x22U
#define       BULK_ERR_UNIT_NOT_READY 0x23U
#define            BULK_ERR_UNIT_BUSY 0x24U
#define                BULK_ERR_STALL 0x25U
#define    BULK_ERR_CMD_NOT_SUPPORTED 0x26U
#define          BULK_ERR_INVALID_CSW 0x27U
#define             BULK_ERR_NO_MEDIA 0x28U
#define              BULK_ERR_BAD_LBA 0x29U
#define        BULK_ERR_MEDIA_CHANGED 0x2AU
#define  BULK_ERR_DEVICE_DISCONNECTED HOST_ERROR_UNPLUGGED
#define    BULK_ERR_UNABLE_TO_RECOVER 0x32U // Reset recovery error
#define          BULK_ERR_INVALID_LUN 0x33U
#define          BULK_ERR_WRITE_STALL 0x34U
#define            BULK_ERR_READ_NAKS 0x35U
#define           BULK_ERR_WRITE_NAKS 0x36U
#define      BULK_ERR_WRITE_PROTECTED 0x37U
#define      BULK_ERR_NOT_IMPLEMENTED 0xFDU
#define   BULK_ERR_GENERAL_SCSI_ERROR 0xF0U
#define    BULK_ERR_GENERAL_USB_ERROR 0xFFU
#define                 BULK_ERR_USER 0xA0U // For subclasses to define their own error codes

// Subclasses
#define           UHS_BULK_SUBCLASS_SCSI_NOT_REPORTED 0x00U   // De facto use
#define                         UHS_BULK_SUBCLASS_RBC 0x01U
#define                       UHS_BULK_SUBCLASS_ATAPI 0x02U   // MMC-5 (ATAPI)
#define                   UHS_BULK_SUBCLASS_OBSOLETE1 0x03U   // Was QIC-157
#define                         UHS_BULK_SUBCLASS_UFI 0x04U   // Specifies how to interface Floppy Disk Drives to USB
#define                   UHS_BULK_SUBCLASS_OBSOLETE2 0x05U   // Was SFF-8070i
#define                        UHS_BULK_SUBCLASS_SCSI 0x06U   // SCSI Transparent Command Set
#define                       UHS_BULK_SUBCLASS_LSDFS 0x07U   // Specifies how host has to negotiate access before trying SCSI
#define                    UHS_BULK_SUBCLASS_IEEE1667 0x08U
// Protocols
#define                            UHS_STOR_PROTO_CBI 0x00U   // CBI (with command completion interrupt)
#define                     UHS_STOR_PROTO_CBI_NO_INT 0x01U   // CBI (without command completion interrupt)
#define                       UHS_STOR_PROTO_OBSOLETE 0x02U
#define                            UHS_STOR_PROTO_BBB 0x50U   // Bulk Only Transport
#define                            UHS_STOR_PROTO_UAS 0x62U

#define MS_MAX_SUPPORTED_LUN       15

/** Mass Storage class-specific request to reset the Mass Storage interface, ready for the next command. */
#define REQ_MassStorageReset       0xFF

/** Mass Storage class-specific request to retrieve the total number of Logical Units (drives) in the SCSI device. */
#define REQ_GetMaxLUN              0xFE

/** Magic signature for a Command Block Wrapper used in the Mass Storage Bulk-Only transport protocol. */
#define MS_CBW_SIGNATURE           0x43425355UL

/** Magic signature for a Command Status Wrapper used in the Mass Storage Bulk-Only transport protocol. */
#define MS_CSW_SIGNATURE           0x53425355UL

/** Mask for a Command Block Wrapper's flags attribute to specify a command with data sent from host-to-device. */
#define MS_COMMAND_DIR_DATA_OUT    (0 << 7)

/** Mask for a Command Block Wrapper's flags attribute to specify a command with data sent from device-to-host. */
#define MS_COMMAND_DIR_DATA_IN     (1 << 7)

/** Error code for some Mass Storage Host functions, indicating a logical (and not hardware) error. */
#define MS_ERROR_LOGICAL_CMD_FAILED                    0x80

#define MASS_STORE_CLASS               0x08
#define MASS_STORE_SUBCLASS            0x06
#define MASS_STORE_PROTOCOL            0x50

#define REQ_MassStorageReset           0xFF
#define REQ_GetMaxLUN                  0xFE

#define CBW_SIGNATURE                  0x43425355UL
#define CSW_SIGNATURE                  0x53425355UL

#define COMMAND_DIRECTION_DATA_OUT     (0 << 7)
#define COMMAND_DIRECTION_DATA_IN      (1 << 7)

#define COMMAND_DATA_TIMEOUT_MS        10000

#define MS_FOUND_DATAPIPE_IN           (1 << 0)
#define MS_FOUND_DATAPIPE_OUT          (1 << 1)

#define MS_MAX_ENDPOINTS 3 //endpoint 0, bulk_IN, bulk_OUT


/*
 * Mass Storage Class Command Block Wrapper(CBW).
 * Used in Bulk Only Transport(BOT).
 * Direction: Host->Device
 *
 */
typedef struct
{
    // Must be MS_CBW_SIGNATURE to indicate valid CBW
    uint32_t signature;
    // associate CSW with CBW
    uint32_t tag;
    // Number of bytes of data host expect to transfer on the Bulk-in or
    // Bulk-out endpoint. If its 0 then no data should be transferred.
    uint32_t dataTransferLength;
    // Bit 7:(0: Data out, 1: Data in)
    uint8_t flags;
    // logical unit number this command is issued to
    uint8_t lun;
    // length of the issued command in the Command Data array
    uint8_t SCSICommandLength;
    uint8_t SCSICommandData[16];
} MS_CommandBlockWrapper_t;

typedef struct
{
    // Must be MS_CSW_SIGNATURE to indicate valid CSW
    uint32_t signature;
    // associate CSW with CBW
    uint32_t tag;
    // Number of bytes of data not processed in the SCSI command
    uint32_t dataTransferResidue;
    // status code of the issued command
    uint8_t status;
} MS_CommandStatusWrapper_t;

enum MS_CommandStatusCodes_t
{
    // Command completed with no error
    SCSI_Command_Pass = 0,
    // Command failed to complete - host may check the exact error via a
    // SCSI REQUEST SENSE command.
    SCSI_Command_Fail = 1,
    // Command failed due to being invalid in the current phase. */
    SCSI_Phase_Error  = 2
};


/** Mass Storage Class Host Mode Configuration and State Structure.
 *
 *  Class state structure. An instance of this structure should be made within the user application,
 *  and passed to each of the Mass Storage class driver functions as the MSInterfaceInfo parameter. This
 *  stores each Mass Storage interface's configuration and state information.
 */
typedef struct
{
    const struct
    {
        uint8_t  DataINPipeNumber; /**< Pipe number of the Mass Storage interface's IN data pipe. */
        bool     DataINPipeDoubleBank; /**< Indicates if the Mass Storage interface's IN data pipe should use double banking. */

        uint8_t  DataOUTPipeNumber; /**< Pipe number of the Mass Storage interface's OUT data pipe. */
        bool     DataOUTPipeDoubleBank; /**< Indicates if the Mass Storage interface's OUT data pipe should use double banking. */
    } Config; /**< Config data for the USB class interface within the device. All elements in this section
               *   <b>must</b> be set or the interface will fail to enumerate and operate correctly.
               */
    struct
    {
        bool IsActive; /**< Indicates if the current interface instance is connected to an attached device, valid
                        *   after \ref MS_Host_ConfigurePipes() is called and the Host state machine is in the
                        *   Configured state.
                        */
        uint8_t InterfaceNumber; /**< Interface index of the Mass Storage interface within the attached device. */

        uint16_t DataINPipeSize; /**< Size in bytes of the Mass Storage interface's IN data pipe. */
        uint16_t DataOUTPipeSize;  /**< Size in bytes of the Mass Storage interface's OUT data pipe. */

        uint32_t TransactionTag; /**< Current transaction tag for data synchronizing of packets. */
    } State; /**< State data for the USB class interface within the device. All elements in this section
              *   <b>may</b> be set to initial values, but may also be ignored to default to sane values when
              *   the interface is enumerated.
              */
} USB_ClassInfo_MS_Host_t;


enum MS_Host_EnumerationFailure_ErrorCodes_t
{
    MS_HOST_ERROR_NoError                  = 0, /**< Configuration Descriptor was processed successfully. */
    MS_HOST_ERROR_InvalidConfigDescriptor  = 1, /**< The device returned an invalid Configuration Descriptor. */
    MS_HOST_ERROR_NoMSInterfaceFound       = 2, /**< A compatible Mass Storage interface was not found in the device's Configuration Descriptor. */
    MS_HOST_ERROR_EndpointsNotFound        = 3, /**< Compatible Mass Storage endpoints were not found in the device's interfaces. */
};


// USBDeviceConfig, UsbConfigXtracter => abstract classes
class MassStorage : public USBDeviceConfig, public UsbConfigXtracter
{
private:
    // keeps state of interfaces of device.
    //USB_ClassInfo_MS_Host_t interface_info;
    uint8_t bMaxLUN;
    volatile uint32_t cbwTag; // Tag
    static const uint8_t bEpDataInIndex = 1; // DataIn endpoint index
    static const uint8_t bEpDataOutIndex = 2; // DataOUT endpoint index:<2

    volatile uint8_t bTheLUN;   // current lun
    SCSI_Capacity_t currentCapacity[MS_MAX_SUPPORTED_LUN];
    uint32_t    qNextPollTime;          // next poll time
    bool        bPollEnable;            // poll enable flag

    bool IsValidCSW(MS_CommandStatusWrapper_t *pcsw, MS_CommandBlockWrapper_t *pcbw);
    void Reset(void);
    //uint8_t Inquiry(uint8_t lun, uint16_t size, uint8_t *buf);
    //uint8_t MediaCTL(uint8_t lun, uint8_t ctl);
    uint8_t Transaction(MS_CommandBlockWrapper_t *cbw, uint16_t bsize, void *buf);
    void disablePoll();
    void enablePoll();
    void fill_scsi_cdb6(SCSI_CDB6_t *cdb,  uint8_t opcode,
                    uint8_t lun, uint32_t lba,
                    uint8_t allocation_length, uint8_t control);
    void create_cbw_packet_cdb6(MS_CommandBlockWrapper_t *cbw, uint32_t tag,
                                uint32_t transfer_len, SCSI_CDB6_t *cdb,
                                uint8_t direction);
    uint8_t GetMaxLUN(uint8_t *max_lun);


protected:
    USBHost     *pUsb;            // USB class instance pointer
    uint32_t    bAddress;         // new address assigned to usb during enumeration
    volatile uint8_t bConfNum;    // configuration number in use as there could be multiple
    volatile uint8_t bIfaceNum;    // interface number in use as there could be multiple
    // total number of EP in the configuration. Incremented by EndpointExtractor
    volatile uint8_t bNumEP;
    volatile uint8_t bNumIface;

    void PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR* ep_ptr);
    /* Endpoint data structure describing the device EP */
    EpInfo      epInfo[MS_MAX_ENDPOINTS];

public:
    MassStorage(USBHost *pusb);
    const USBHost* GetUsb() { return pUsb; };
        uint8_t GetbMaxLUN(void) {
                return bMaxLUN; // Max LUN
        }

        uint8_t GetbTheLUN(void) {
                return bTheLUN; // Active LUN
        }
    // USBDeviceConfig implementation
    virtual uint32_t Init(uint32_t parent, uint32_t port, uint32_t lowspeed);
    virtual uint32_t Release();
    virtual uint32_t Poll();
    virtual uint32_t GetAddress() { return bAddress; };
    uint8_t TestUnitReady(uint8_t lun);

    // UsbConfigXtracter implementation
    virtual void EndpointXtract(uint32_t conf, uint32_t iface, uint32_t alt,
                                uint32_t proto, const USB_ENDPOINT_DESCRIPTOR *ep);
    //uint8_t Read(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, uint8_t *buf);
    //uint8_t Write(uint8_t lun, uint32_t addr, uint16_t bsize, uint8_t blocks, const uint8_t *buf);
    //uint32_t GetCapacity(uint8_t lun);
    uint8_t SCSITransaction6(SCSI_CDB6_t *cdb, uint16_t buf_size, void *buf, uint8_t dir);


};

#endif
