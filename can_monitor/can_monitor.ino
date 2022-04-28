/**
 * @file can_monitor.ino
 * 
 * @author Joseph Telaak
 * 
 * @brief Arduino that monitor and injects frames into a CAN bus
 * 
 * @version 0.1
 * 
 * @date 2022-03-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Arduino
#include "Arduino.h"

// CAN
#include "mcp2515.h"
MCP2515* can_trans;

// CAN Pins
#define CAN_CS 10
#define CAN_INT 2

//  Message Buffer
struct can_frame can_msg_in;
struct can_frame can_msg_out;

// CAN Info
uint32_t m_can_id = 0x000;
uint8_t m_can_dlc = 8;

// LEDs
#define TX_LED 6
#define RX_LED 5

// Debugging
//#define DEBUG

/**
 * @brief Main Arduino Setup
 * 
 */

void setup() {
    // LEDs
    pinMode(TX_LED, OUTPUT);
    pinMode(RX_LED, OUTPUT);

    digitalWrite(TX_LED, HIGH);
    digitalWrite(RX_LED, HIGH);

    // Init serial port
    Serial.begin(115200);

    // Init
    can_trans = new MCP2515(CAN_CS);

    // Reset and set
    can_trans -> reset();
    can_trans -> setBitrate(CAN_125KBPS);
    can_trans -> setNormalMode();

    // Setup Interupts
    attachInterrupt(digitalPinToInterrupt(CAN_INT), canLoop, FALLING);

    delay(200);
    digitalWrite(TX_LED, LOW);
    digitalWrite(RX_LED, LOW);

}

/**
 * @brief Main Arduino Loop
 * 
 */

void loop() {
    if (Serial.available() > 0) {
        String message = Serial.readString();

        if (message.indexOf(">") != -1) {
            noInterrupts();
            digitalWrite(TX_LED, HIGH);

            adapterSendMessage(message.substring(message.indexOf(">")+1));

            digitalWrite(TX_LED, LOW);
            interrupts();
        }
    }
}

/** @brief CAN Message Handling (Runs on interupt) */
void canLoop() {
    // Get message
    if (getCANMessage()) { printReceivedCANMessage(); }

}

/**
 * @brief Get the CAN message if the id is correct
 * 
 * @return true If message is valid and id's match
 * @return false If message is invalid or id's do not match
 */

bool getCANMessage() {
    if (can_trans -> readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        if (can_msg_in.can_id == m_can_id) {
            // Print Message
            digitalWrite(RX_LED, HIGH);
            printReceivedCANMessage();
            digitalWrite(RX_LED, LOW);

            return true;

        } 
    }

    return false;

}

/** @brief Print out the received can frame*/
void printReceivedCANMessage() {
    // Start log
    Serial.print("CAN-RX: (" + String(can_msg_in.can_id) + ") ");

    // Print data
    for (int i = 0; i < can_msg_in.can_dlc; i++) {
        Serial.print(String(can_msg_in.data[i]) + " ");

    }

    // New Line
    Serial.println();

}

/**
 * @brief Sends a message through the CAN Adapter
 * 
 * @param drive_com_msg Message to send (8x bytes separated by spaces)
 */

void adapterSendMessage(String drive_com_msg) {
    // Get the ID indexes
    String id_msg = drive_com_msg.substring(0, drive_com_msg.indexOf(")") + 2);
    id_msg.replace("(", "");
    id_msg.replace(")", "");
    id_msg.replace(" ", "");

    // Convert ID
    const char *nptr = id_msg.c_str();
    char *endptr;
    uint32_t id = strtoul(nptr, &endptr, 10);

    // Serial.print("ID: ");
    // Serial.println(id, HEX);

    // Split the data string
    char string[128];
    drive_com_msg.substring(drive_com_msg.indexOf(")") + 2).toCharArray(string, sizeof(string));
    String data_points[8];

    char *p;
    char delimiter[] = " ";

    int i = 0;
    p = strtok(string, delimiter);

    while(p && i < 8) {
        data_points[i] = p;
        p = strtok(NULL, delimiter);
        ++i;

    }

    uint8_t data[8];

    for (i = 0; i < 8; ++i) {
        data[i] = data_points[i].toInt();
        
    }

    // Serial.print("DATA: ");

    // for (i = 0; i < 8; ++i) {
    //     Serial.print(data[i], HEX);
    //     Serial.print(", ");
        
    // }

    // Serial.println();

    // Send message
    sendCANMessage(id, data);

}

/**
 * @brief Send a message of the can bus
 * 
 * @param id ID of the CAN device to send message to
 * @param m_data Data to send to the CAN device
 */

void sendCANMessage(uint32_t id, uint8_t m_data[8]) {
    // Assign ID
    can_msg_out.can_id = id;

    // Assign dlc
    can_msg_out.can_dlc = m_can_dlc;

    // Map data
    for (int i = 0; i < m_can_dlc; i++) {
        can_msg_out.data[i] = m_data[i];

    }

    // Send message
    can_trans -> sendMessage(&can_msg_out);

}

/** @brief Print out the outgoing message */
void printOutgoingCANMessage() {
    // Start log
    Serial.print("CAN-TX: (" + String(can_msg_out.can_id) + ") ");

    // Print data
    for (int i = 0; i < can_msg_out.can_dlc; i++) {
        Serial.print(String(can_msg_out.data[i]) + " ");

    }

    // New Line
    Serial.println();

}