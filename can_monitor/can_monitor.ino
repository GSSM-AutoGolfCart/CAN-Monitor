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

// CAN Lib
#include <mcp2515.h>

// CAN Frames
struct can_frame can_msg_in;
struct can_frame can_msg_out;

// CAN Pins
#define CAN_CS 10
#define CAN_INT 2

// CAN
MCP2515 mcp2515(CAN_CS);
uint8_t m_can_dlc = 8;

/** @brief Arduino Setup */
void setup() {
  // Serial
  Serial.begin(115200);
  
  // CAN Setup
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(CAN_INT), canFunc, FALLING);
  
  // Print header
  Serial.println("CAN Injector");
  Serial.println("Example Message: (0) 0 0 0 0 0 0 0 0");

}

/** @brief Arduino Loop */
void loop() {
    if (Serial.available() > 0) {
        String message = Serial.readString();
        
        if (message.indexOf("CMD-Send: ") != -1) {
            message.replace("CMD-Send: ", "");

            // Send Message
            sendMessage(message);

        }
    }
}

/** @brief CAN received */
void canFunc() {
    if (!getCANMessage()) { return; }

}

/**
 * @brief Get the CAN message if the id is correct
 * 
 * @return true If message is valid and id's match
 * @return false If message is invalid or id's do not match
 */

bool getCANMessage() {
    if (mcp2515.readMessage(&can_msg_in) == MCP2515::ERROR_OK) {
        // Print Message
        printReceivedCANMessage();

        return true;
        
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

void sendMessage(String drive_com_msg) {
    // Get the ID indexes
    int id_begin_index = drive_com_msg.indexOf("(");
    int id_end_index = drive_com_msg.indexOf(")");

    // Check ID
    if (id_begin_index == -1 || id_end_index == -1) {
        Serial.println("Err: CAN message is missing ID");
        return;

    }

    // Get the ID
    char* str_id = drive_com_msg.substring(id_begin_index, id_end_index - 1).c_str();
    char* ptr;
    uint32_t set_id = strtoul(str_id, &ptr, 16);

    // Clear ID
    drive_com_msg.replace(drive_com_msg.substring(0, id_end_index + 2), "");

    // Get data
    uint8_t data[8];
    int count = 0;
    
    while (drive_com_msg.length() > 0) {
        int index = drive_com_msg.indexOf(' ');
        
        if (index == -1) {
            data[count++] = atoi(drive_com_msg.c_str());
          
        } else {
            data[count++] = atoi(drive_com_msg.substring(0, index).c_str());
            drive_com_msg = drive_com_msg.substring(index + 1);
            
        }
    }

    // Send message
    sendCANMessage(set_id, data);

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

    // Start log
    Serial.print("CAN-TX: (" + String(can_msg_out.can_id) + ") ");

    // Print data
    for (int i = 0; i < can_msg_out.can_dlc; i++) {
        Serial.print(String(can_msg_out.data[i]) + " ");

    }

    // New Line
    Serial.println();

    // Send message
    mcp2515.sendMessage(&can_msg_out);

}
