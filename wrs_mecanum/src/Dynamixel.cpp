#include "Dynamixel.h"

Dynamixel::Dynamixel()
{}

Dynamixel::~Dynamixel()
{}

void Dynamixel::begin(const char* port_name, const int baudrate)
{
    this->portHandler = dynamixel::PortHandler::getPortHandler(port_name);
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(this->PROTOCOL_VERSION);

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