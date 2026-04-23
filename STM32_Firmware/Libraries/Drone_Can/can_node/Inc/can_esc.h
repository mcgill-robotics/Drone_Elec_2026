#pragma once


// Private file include
#include <can_node.h>


// Send esc status regularly
// Returns the number of bits encoded or negative error value
// Check that it is greater than int16_t
void send_esc_status(void);

// Handle esc input in can_node when received
void handle_ESC_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer);


// Motor input and last update time
static struct {
    int16_t esc_cmd;
    int32_t last_update;
} esc[ESC_COUNT];