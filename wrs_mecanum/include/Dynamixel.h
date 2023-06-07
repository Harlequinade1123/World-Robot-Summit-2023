#pragma once

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <iostream>
#include <cmath>

class Dynamixel
{
    private:
    dynamixel::PortHandler * portHandler;
    dynamixel::PacketHandler * packetHandler;

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    const float    PROTOCOL_VERSION         = 2.0;
    const uint16_t ADDRESS_OPERATING_MODE   = 11;
    const uint16_t ADDRESS_TORQUE_ENABLE    = 64;
    const uint16_t ADDRESS_GOAL_CURRENT     = 102;
    const uint16_t ADDRESS_GOAL_VELOCITY    = 104;
    const uint16_t ADDRESS_GOAL_POSITION    = 116;
    const uint16_t ADDRESS_PRESENT_CURRENT  = 126;
    const uint16_t ADDRESS_PRESENT_VELOCITY = 128;
    const uint16_t ADDRESS_PRESENT_POSITION = 132;
    
    public:
    Dynamixel();
    ~Dynamixel();
    void begin(const char* port_name, const int baudrate);
    void torqueOn(uint8_t id);
    void torqueOff(uint8_t id);
    void writeCurrent(uint8_t id, int16_t data);
    void writeVelocity(uint8_t id, int32_t data);
    void writeBulkVelocity(int32_t val0, int32_t val1, int32_t val2, int32_t val3);
    void writeRPM(uint8_t id, float data);
    void writePosition(uint8_t id, int32_t data);
    
    bool readCurrent(uint8_t id, int16_t &data);
    bool readVelocity(uint8_t id, int32_t &data);
    bool readRPM(uint8_t id, float &data);
    bool readPosition(uint8_t id, int32_t &data);
};