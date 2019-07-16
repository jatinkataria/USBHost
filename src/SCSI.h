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

#ifndef SCSI_H
#define SCSI_H

#include <stdint.h>
#define UINT8_BYTE0(__usi__)  ((uint8_t)((__usi__) & 0xff ))
#define UINT8_BYTE1(__usi__)  ((uint8_t)(((__usi__) >> 8) & 0xff))
#define UINT8_BYTE2(__usi__)  ((uint8_t)(((__usi__) >> 16) & 0xff))
#define UINT8_BYTE3(__usi__)  ((uint8_t)(((__usi__) >> 24) & 0xff))
#define UINT8_BYTE4(__usi__)  ((uint8_t)(((__usi__) >> 32) & 0xff))
#define UINT8_BYTE5(__usi__)  ((uint8_t)(((__usi__) >> 40) & 0xff))
#define UINT8_BYTE6(__usi__)  ((uint8_t)(((__usi__) >> 48) & 0xff))
#define UINT8_BYTE7(__usi__)  ((uint8_t)(((__usi__) >> 56) & 0xff))

/** SCSI Command Code for an INQUIRY command. */
#define SCSI_CMD_INQUIRY                               0x12

/** SCSI Command Code for a REQUEST SENSE command. */
#define SCSI_CMD_REQUEST_SENSE                         0x03

#define SCSI_CMD_TEST_UNIT_READY                       0x00


/** SCSI Command Code for a READ CAPACITY (10) command. */
#define SCSI_CMD_READ_CAPACITY_10                      0x25

/** SCSI Command Code for a SEND DIAGNOSTIC command. */
#define SCSI_CMD_SEND_DIAGNOSTIC                       0x1D

/** SCSI Command Code for a PREVENT ALLOW MEDIUM REMOVAL command. */
#define SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL          0x1E

/** SCSI Command Code for a WRITE (10) command. */
#define SCSI_CMD_WRITE_10                              0x2A

/** SCSI Command Code for a READ (10) command. */
#define SCSI_CMD_READ_10                               0x28

/** SCSI Command Code for a WRITE (6) command. */
#define SCSI_CMD_WRITE_6                               0x0A

/** SCSI Command Code for a READ (6) command. */
#define SCSI_CMD_READ_6                                0x08

/** SCSI Command Code for a VERIFY (10) command. */
#define SCSI_CMD_VERIFY_10                             0x2F

/** SCSI Command Code for a MODE SENSE (6) command. */
#define SCSI_CMD_MODE_SENSE_6                          0x1A

/** SCSI Command Code for a MODE SENSE (10) command. */
#define SCSI_CMD_MODE_SENSE_10                         0x5A

/** SCSI Sense Code to indicate no error has occurred. */
#define SCSI_SENSE_KEY_GOOD                            0x00

/** SCSI Sense Code to indicate that the device has recovered from an error. */
#define SCSI_SENSE_KEY_RECOVERED_ERROR                 0x01

/** SCSI Sense Code to indicate that the device is not ready for a new command. */
#define SCSI_SENSE_KEY_NOT_READY                       0x02

/** SCSI Sense Code to indicate an error whilst accessing the medium. */
#define SCSI_SENSE_KEY_MEDIUM_ERROR                    0x03

/** SCSI Sense Code to indicate a hardware has occurred. */
#define SCSI_SENSE_KEY_HARDWARE_ERROR                  0x04

/** SCSI Sense Code to indicate that an illegal request has been issued. */
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST                 0x05

/** SCSI Sense Code to indicate that the unit requires attention from the host to indicate
 *  a reset event, medium removal or other condition.
 */
#define SCSI_SENSE_KEY_UNIT_ATTENTION                  0x06

/** SCSI Sense Code to indicate that a write attempt on a protected block has been made. */
#define SCSI_SENSE_KEY_DATA_PROTECT                    0x07

/** SCSI Sense Code to indicate an error while trying to write to a write-once medium. */
#define SCSI_SENSE_KEY_BLANK_CHECK                     0x08

/** SCSI Sense Code to indicate a vendor specific error has occurred. */
#define SCSI_SENSE_KEY_VENDOR_SPECIFIC                 0x09

/** SCSI Sense Code to indicate that an EXTENDED COPY command has aborted due to an error. */
#define SCSI_SENSE_KEY_COPY_ABORTED                    0x0A

/** SCSI Sense Code to indicate that the device has aborted the issued command. */
#define SCSI_SENSE_KEY_ABORTED_COMMAND                 0x0B

/** SCSI Sense Code to indicate an attempt to write past the end of a partition has been made. */
#define SCSI_SENSE_KEY_VOLUME_OVERFLOW                 0x0D

/** SCSI Sense Code to indicate that the source data did not match the data read from the medium. */
#define SCSI_SENSE_KEY_MISCOMPARE                      0x0E

/** SCSI Additional Sense Code to indicate no additional sense information is available. */
#define SCSI_ASENSE_NO_ADDITIONAL_INFORMATION          0x00

/** SCSI Additional Sense Code to indicate that the logical unit (LUN) addressed is not ready. */
#define SCSI_ASENSE_LOGICAL_UNIT_NOT_READY             0x04

/** SCSI Additional Sense Code to indicate an invalid field was encountered while processing the issued command. */
#define SCSI_ASENSE_INVALID_FIELD_IN_CDB               0x24

/** SCSI Additional Sense Code to indicate that an attempt to write to a protected area was made. */
#define SCSI_ASENSE_WRITE_PROTECTED                    0x27

/** SCSI Additional Sense Code to indicate an error whilst formatting the device medium. */
#define SCSI_ASENSE_FORMAT_ERROR                       0x31

/** SCSI Additional Sense Code to indicate an invalid command was issued. */
#define SCSI_ASENSE_INVALID_COMMAND                    0x20

/** SCSI Additional Sense Code to indicate a write to a block out outside of the medium's range was issued. */
#define SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x21

/** SCSI Additional Sense Code to indicate that no removable medium is inserted into the device. */
#define SCSI_ASENSE_MEDIUM_NOT_PRESENT                 0x3A

/** SCSI Additional Sense Qualifier Code to indicate no additional sense qualifier information is available. */
#define SCSI_ASENSEQ_NO_QUALIFIER                      0x00

/** SCSI Additional Sense Qualifier Code to indicate that a medium format command failed to complete. */
#define SCSI_ASENSEQ_FORMAT_COMMAND_FAILED             0x01

/** SCSI Additional Sense Qualifier Code to indicate that an initializing command must be issued before the issued
 *  command can be executed.
 */
#define SCSI_ASENSEQ_INITIALIZING_COMMAND_REQUIRED     0x02

/** SCSI Additional Sense Qualifier Code to indicate that an operation is currently in progress. */
#define SCSI_ASENSEQ_OPERATION_IN_PROGRESS             0x07


/** SCSI Command Code for CAN passthrough  */
#define SCSI_CMD_CAN_PASSTHROUGH                      0xdf


/** Mass Storage Class SCSI Sense Structure
 *  Type define for a SCSI Sense structure. Structures of this type are filled out by the
 *  device via the \ref MS_Host_RequestSense() function, indicating the current sense data of the
 *  device (giving explicit error codes for the last issued command).
 *  For details of the structure contents, refer to the SCSI specifications.
 */
typedef struct
{
    uint8_t       responseCode;
    uint8_t       segmentNumber;
    unsigned char senseKey            : 4;
    unsigned char reserved            : 1;
    unsigned char ILI                 : 1;
    unsigned char EOM                 : 1;
    unsigned char fileMark            : 1;

    uint8_t      information[4];
    // additional sense length should be >= 244 making max data to 252 bytes
    uint8_t      additionalLength;
    uint8_t      cmdSpecificInformation[4];
    uint8_t      additionalSenseCode;
    uint8_t      additionalSenseQualifier;
    uint8_t      fieldReplaceableUnitCode;
    uint8_t      senseKeySpecific[3];
    // 18-n where n <=252 additional bytes can be degined later
} SCSI_Request_Sense_Response_ti __attribute__((packed));

/** Mass Storage Class SCSI Inquiry Structure.
 *  Type define for a SCSI Inquiry structure. Structures of this type are filled out by the
 *  device via the \ref MS_Host_GetInquiryData() function, retrieving the attached device's
 *  information.
 *  For details of the structure contents, refer to the SCSI specifications.
 */
typedef struct
{
    unsigned char deviceType          : 5;
    unsigned char peripheralQualifier : 3;

    unsigned char reserved            : 7;
    unsigned char removable           : 1;

    uint8_t      version;

    unsigned char responseDataFormat  : 4;
    unsigned char reserved2           : 1;
    unsigned char normACA             : 1;
    unsigned char trmTsk              : 1;
    unsigned char AERC                : 1;

    uint8_t      additionalLength;
    uint8_t      reserved3[2];

    unsigned char softReset           : 1;
    unsigned char cmdQue              : 1;
    unsigned char reserved4           : 1;
    unsigned char linked              : 1;
    unsigned char sync                : 1;
    unsigned char wideBus16Bit        : 1;
    unsigned char wideBus32Bit        : 1;
    unsigned char relAddr             : 1;

    uint8_t      vendorID[8];
    uint8_t      productID[16];
    uint8_t      revisionID[4];
    // 96-143 copyright notice can be defined later
} SCSI_Inquiry_Response_t;

// Command descriptor Block to send command.
// Each CDB can be a total of 6, 10, 12, or 16 bytes, but later versions of the SCSI standard also allow for variable-length CDBs
typedef struct __attribute__((packed)) SCSI_CDB6 {
        uint8_t opcode;

        unsigned LBAMSB : 5;
        unsigned LUN : 3;

        uint8_t LBAHB;
        uint8_t LBALB;
        uint8_t allocationLength;
        uint8_t control;
} SCSI_CDB6_t;

typedef struct __attribute__((packed)) SCSI_CDB12 {
        uint8_t opcode;
        uint8_t service_action:4;
        uint8_t misc_cdb:4;
        uint32_t lba;
        uint32_t length;
        uint8_t misc_cdb1;
        uint8_t control;
} SCSI_CDB12_t;



/** SCSI Device LUN Capacity Structure.
 *
 *  SCSI capacity structure, to hold the total capacity of the device in both the number
 *  of blocks in the current LUN, and the size of each block. This structure is filled by
 *  the device when the \ref MS_Host_ReadDeviceCapacity() function is called.
 */
typedef struct
{
    uint8_t blocks[4]; /**< Number of blocks in the addressed LUN of the device. */
    uint8_t blockSize[4]; /**< Number of bytes in each block in the addressed LUN. */
} SCSI_Capacity_t;

#endif
