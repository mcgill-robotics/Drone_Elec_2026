#pragma once


// Private file include
#include <can_node.h>


// Send esc status regularly
// Returns the number of bits encoded or negative error value
// Check that it is greater than int16_t
void send_esc_status(void);

// Handle esc input in can_node when received
void handle_ESC_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer);


// esc_cmd carries motor command from -8192 to 8191
// last_update is incremented each time a new command is recieved
volatile static struct {
    int16_t esc_cmd;
    uint32_t last_update;
} esc[ESC_COUNT];