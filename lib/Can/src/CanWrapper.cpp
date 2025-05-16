#include "CanWrapper.h"

void CanWrapper::begin() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);

    mcp2515.setConfigMode();// tell the MCP2515 next instructions are for configuration

    // enable filtering for 29 bit address on both RX buffers
    mcp2515.setFilterMask(MCP2515::MASK0, true, 0xFF00);
    mcp2515.setFilterMask(MCP2515::MASK1, true, 0xFF00);

    // set all filters to module's ID, so only packets for us get through
    mcp2515.setFilter(MCP2515::RXF0, true, CAN_ID << 8);
    mcp2515.setFilter(MCP2515::RXF1, true, CAN_ID << 8);
    mcp2515.setFilter(MCP2515::RXF2, true, CAN_ID << 8);
    mcp2515.setFilter(MCP2515::RXF3, true, CAN_ID << 8);
    mcp2515.setFilter(MCP2515::RXF4, true, CAN_ID << 8);
    mcp2515.setFilter(MCP2515::RXF5, true, CAN_ID << 8);

    mcp2515.setNormalMode();
}

bool CanWrapper::sendMessage(uint8_t id, const void* data, uint8_t length) {
    struct can_frame msg = {0};
    msg.can_id = CAN_ID | CAN_EFF_FLAG | (id << 16);
    msg.can_dlc = length;
    memcpy(msg.data, data, length);
    return mcp2515.sendMessage(&msg) == MCP2515::ERROR_OK;
}

bool CanWrapper::readMessage(uint8_t* id, byte* data) {
    struct can_frame msg = {0};
    if (mcp2515.readMessage(&msg) == MCP2515::ERROR_OK) {
        *id = ( msg.can_id >> 16 ) & 0xFF;
        memcpy(data, msg.data, msg.can_dlc);
        return true;
    }
    return false;
}