// the setup function runs once when you press reset or power the board
// Required libraries

#include <variant.h>
#include <Arduino.h>
#include <due_can.h>
#include "Usb.h"
#include "MassStorage.h"
#include "sata_car.h"


#define Serial Serial3


USBHost usb;
MassStorage ms(&usb);

static int setup_success;
static size_t packet_count;
static uint32_t last_time;

void blink_led(unsigned int count, unsigned int interval) {
    for (unsigned int i = 0; i < count; i++) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(interval);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(interval);
    }
}

void printFrame(CAN_FRAME &frame) {
    Serial.print("ID: 0x");
    Serial.print(frame.id, HEX);
    Serial.print(" Len: ");
    Serial.print(frame.length);
    Serial.print(" Data: 0x");
    for (int count = 0; count < frame.length; count++) {
        Serial.print(frame.data.bytes[count], HEX);
        Serial.print(" ");
    }
    Serial.print("\r\n");
}

void setup() {
    cpu_irq_enable();
    setup_success = 0;
    packet_count = 0;
    last_time = millis();
    Serial.begin(9600);
    Serial.println("Sata Car setting up");
    pinMode(LED_BUILTIN, OUTPUT);
    delay(200);
    uint32_t can0_success;

    Serial.println("Doing Auto Baud scan on CAN0");
    if (!(can0_success = Can0.beginAutoSpeed())) {
        Serial.println("auto baud failed for CAN0");
    }

    if (!can0_success) {
        Serial.println("setup failed");
        setup_success = 0;
        return;
    } else {
        setup_success = 1;
    }
    //By default there are 7 mailboxes for each device that are RX boxes
    //This sets each mailbox to have an open filter that will accept extended
    //or standard frames
    int filter;
    //extended
    for (filter = 0; filter < 3; filter++) {
        Can0.setRXFilter(filter, 0, 0, true);
    }
    //standard
    for (int filter = 3; filter < 7; filter++) {
        Can0.setRXFilter(filter, 0, 0, false);
    }
    Serial.println("setup done");


}


// wrap can message to scsi
void pass_can_through_scsi(CAN_FRAME &frame) {
   Serial.println("can passthrough start");
    if (SATA_ARB_ID != frame.id) {
        Serial.print("SATA ARB ID required");
        //return;
    }
    SATA_CAR car;
    uint32_t i = 0;
    uint8_t status = 0;
   Serial.print("frame length: ");
   Serial.println(frame.length);
    while (i < frame.length) {
        Serial.print(frame.data.bytes[i], HEX);
        Serial.print(" ");
        Serial.print(frame.data.bytes[i+1], HEX);
        car.cyl_num = frame.data.bytes[i];
        car.rpm = frame.data.bytes[i+1];
        status = ms.omgDoCAN((uint8_t *)&car, sizeof(car));
        if (status)
            break;
        i += sizeof(car);
       Serial.println("i: ");
       Serial.print(i);
    }
   Serial.println("can passthrough successful");
}
int send = 0;

// the loop function runs over and over again forever
void loop() {
    Serial.println("looping\n");
    if (!setup_success) {
        blink_led(1, 1000);
        //return;
    }
    //blink_led(4, 20);
    CAN_FRAME incoming;
    uint32_t cur_time;
    uint8_t lun, status;
    uint8_t max_lun = ms.GetbMaxLUN();
    if (Can0.available() > 0) {
        Can0.read(incoming);
        packet_count++;
        cur_time = millis();

        if (cur_time >= (last_time + 1000)) {
            Serial.print(packet_count);
            Serial.print(" packets per second\r\n");
            last_time = cur_time;
            packet_count = 0;
        }
        printFrame(incoming);
    }
    else {
        incoming.length = 2;
        incoming.data.bytes[0] = 0x10;
        incoming.data.bytes[1] = 0x10;
    }

    // Pass Through as USBMS command
    usb.Task();
    Serial.println("maxlun from mass:");
    Serial.println(max_lun);
    //for (lun = 0; lun < ms.GetbMaxLUN(); lun++) {
    //    status = ms.TestUnitReady(lun);
    //    digitalWrite(LED_BUILTIN, LOW);
    //}
    if (ms.TestUnitReady(0))
        return;
    pass_can_through_scsi(incoming);
    delay(1000);
}
