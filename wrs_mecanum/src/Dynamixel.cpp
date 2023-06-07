#include "Dynamixel.h"

Dynamixel::Dynamixel()
{}

Dynamixel::~Dynamixel()
{}

void Dynamixel::begin(const char* port_name, const int baudrate)
{
    this->portHandler    = dynamixel::PortHandler::getPortHandler(port_name);
    this->packetHandler  = dynamixel::PacketHandler::getPacketHandler(this->PROTOCOL_VERSION);

    if (!this->portHandler->openPort())
    {
        std::cerr << "Failed to open the port!" << std::endl;
        exit(-1);
    }

    if (!this->portHandler->setBaudRate(baudrate))
    {
        std::cerr << "Failed to set the baudrate!" << std::endl;
        exit(-1);
    }
}

/**
 * WRITE
*/
void Dynamixel::torqueOn(uint8_t id)
{
    this->packetHandler->write1ByteTxRx(this->portHandler, id, this->ADDRESS_TORQUE_ENABLE, 1, &this->dxl_error);
}

void Dynamixel::torqueOff(uint8_t id)
{
    this->packetHandler->write1ByteTxRx(this->portHandler, id, this->ADDRESS_TORQUE_ENABLE, 0, &this->dxl_error);
}

void Dynamixel::writeCurrent(uint8_t id, int16_t data)
{
    this->packetHandler->write2ByteTxRx(this->portHandler, id, this->ADDRESS_GOAL_CURRENT, data, &this->dxl_error);
}

void Dynamixel::writeVelocity(uint8_t id, int32_t data)
{
    this->packetHandler->write4ByteTxRx(this->portHandler, id, this->ADDRESS_GOAL_VELOCITY, data, &this->dxl_error);
}

void Dynamixel::writeBulkVelocity(int32_t val0, int32_t val1, int32_t val2, int32_t val3)
{
    dynamixel::GroupBulkWrite groupBulkWrite(this->portHandler, this->packetHandler);
    groupBulkWrite.clearParam();
    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(val0));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(val0));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(val0));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(val0));
    groupBulkWrite.addParam(14, this->ADDRESS_GOAL_VELOCITY, 4, param_goal_position);
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(val1));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(val1));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(val1));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(val1));
    groupBulkWrite.addParam(11, this->ADDRESS_GOAL_VELOCITY, 4, param_goal_position);
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(val2));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(val2));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(val2));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(val2));
    groupBulkWrite.addParam(12, this->ADDRESS_GOAL_VELOCITY, 4, param_goal_position);
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(val3));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(val3));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(val3));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(val3));
    groupBulkWrite.addParam(13, this->ADDRESS_GOAL_VELOCITY, 4, param_goal_position);
    groupBulkWrite.txPacket();
}


void Dynamixel::writeRPM(uint8_t id, float data)
{
    this->writeVelocity(id, static_cast<int32_t>(data / 0.229));
}

void Dynamixel::writePosition(uint8_t id, int32_t data)
{
    this->packetHandler->write4ByteTxRx(this->portHandler, id, this->ADDRESS_GOAL_POSITION, data, &this->dxl_error);
}


/**
 * READ
*/
bool Dynamixel::readCurrent(uint8_t id, int16_t &data)
{
    this->dxl_comm_result = this->packetHandler->read2ByteTxRx(this->portHandler, id, this->ADDRESS_PRESENT_CURRENT, (uint16_t*)&data, &this->dxl_error);
    if (this->dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->packetHandler->getTxRxResult(this->dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->packetHandler->getRxPacketError(this->dxl_error));
        return false;
    }
    return true;
}

bool Dynamixel::readVelocity(uint8_t id, int32_t &data)
{
    this->dxl_comm_result = this->packetHandler->read4ByteTxRx(this->portHandler, id, this->ADDRESS_PRESENT_VELOCITY, (uint32_t*)&data, &this->dxl_error);
    if (this->dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->packetHandler->getTxRxResult(this->dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->packetHandler->getRxPacketError(this->dxl_error));
        return false;
    }
    return true;
}

bool Dynamixel::readBulkVelocity(int32_t &data0, int32_t &data1, int32_t &data2, int32_t &data3)
{
    dynamixel::GroupBulkRead groupBulkRead(this->portHandler, this->packetHandler);
    groupBulkRead.addParam(14, this->ADDRESS_GOAL_VELOCITY, 4);
    groupBulkRead.addParam(11, this->ADDRESS_GOAL_VELOCITY, 4);
    groupBulkRead.addParam(12, this->ADDRESS_GOAL_VELOCITY, 4);
    groupBulkRead.addParam(13, this->ADDRESS_GOAL_VELOCITY, 4);
    groupBulkRead.txRxPacket();
    bool check = true;
    check = check && groupBulkRead.isAvailable(14, this->ADDRESS_GOAL_VELOCITY, 4);
    check = check && groupBulkRead.isAvailable(11, this->ADDRESS_GOAL_VELOCITY, 4);
    check = check && groupBulkRead.isAvailable(12, this->ADDRESS_GOAL_VELOCITY, 4);
    check = check && groupBulkRead.isAvailable(13, this->ADDRESS_GOAL_VELOCITY, 4);
    if (!check)
    {
        return false;
    }
    data0 = groupBulkRead.getData(14, this->ADDRESS_GOAL_VELOCITY, 4);
    data1 = groupBulkRead.getData(11, this->ADDRESS_GOAL_VELOCITY, 4);
    data2 = groupBulkRead.getData(12, this->ADDRESS_GOAL_VELOCITY, 4);
    data3 = groupBulkRead.getData(13, this->ADDRESS_GOAL_VELOCITY, 4);
    return true;
}

bool Dynamixel::readRPM(uint8_t id, float &data)
{
    int32_t get_data;
    this->dxl_comm_result = this->packetHandler->read4ByteTxRx(this->portHandler, id, this->ADDRESS_PRESENT_VELOCITY, (uint32_t*)&get_data, &this->dxl_error);
    if (this->dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->packetHandler->getTxRxResult(this->dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->packetHandler->getRxPacketError(this->dxl_error));
        return false;
    }
    data  = static_cast<float>(get_data);
    data *= 0.229;
    return true;
}

bool Dynamixel::readPosition(uint8_t id, int32_t &data)
{
    this->dxl_comm_result = this->packetHandler->read4ByteTxRx(this->portHandler, id, this->ADDRESS_PRESENT_POSITION, (uint32_t*)&data, &this->dxl_error);
    if (this->dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", this->packetHandler->getTxRxResult(this->dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", this->packetHandler->getRxPacketError(this->dxl_error));
        return false;
    }
    return true;
}