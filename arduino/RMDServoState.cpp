#include "Arduino.h"
#include "RMDServoState.h"


RMDServoState::RMDServoState(unsigned long can_id, MCP_CAN *can_master, int timeout_set)
{
    device_id = can_id;
    TIMEOUT = timeout_set;
    m_pcan_master = can_master;
    m_encoder_offset  = 0;
    m_absolute_pulses = 0;
    m_relative_pulses = 0;
    m_degrees_per_sec = 0;
    m_current = 0;
    m_voltage = 0.0;
    m_amperage = 0;
    m_temperature = 0;
    m_timeout_counter = micros();
}


long int RMDServoState::getAbsEncoder(void)
{
    return m_absolute_pulses;
}
long int RMDServoState::getRelEncoder(void)
{
    return m_relative_pulses;
}
long int RMDServoState::getEncoderOffset(void)
{
    return m_encoder_offset;
}
int RMDServoState::getTemperature(void)
{
    return m_temperature;
}
float RMDServoState::getVoltage(void)
{
    return m_voltage;
}
int RMDServoState::getCurrent(void)
{
    return m_current;
}


int RMDServoState::waitForFrame(void)
{
    m_timeout_counter = micros();
    unsigned char rxLen = 0;
    unsigned char rxBuf[8];
    long unsigned int rxId;
    while (micros()-m_timeout_counter < TIMEOUT) {
        if ( CAN_MSGAVAIL == m_pcan_master->checkReceive() ) {
            m_pcan_master->readMsgBuf(&rxId, &rxLen, rxBuf);
            if (rxId == device_id) {
                return processFrame(rxLen, rxBuf);
            }
        }
    }
    return 0;
}


int RMDServoState::processFrame(int len, unsigned char* buf)
{
    if (len != 8) { return 0; }
    for (int i = 0; i < 8; i++) {
        m_incoming_buf[i] = buf[i];
    }

    unsigned char cmdByte = m_incoming_buf[0];
    if (cmdByte == 0x90) { requestEncoderResponse(); }
    if (cmdByte == 0x9B) { requestClearErrorResponse(); }
    if (cmdByte == 0xA2) { requestVelocityResponse(); }
    if (cmdByte == 0x80) { requestMotorOffResponse(); }
    
    return 1;
}


int RMDServoState::requestEncoder(bool wait)
{
    unsigned char msg[8] = {
        0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };    
    if ( !(CAN_OK == m_pcan_master->sendMsgBuf(device_id, 0, 8, msg)) ) { 
        return 0; 
    } else {
        return wait ? waitForFrame() : 1;
    }
}
void RMDServoState::requestEncoderResponse(void)
{
    uint16_t encoder, encoderRaw, encoderOffset;
    *(uint8_t *)(&encoder)           = m_incoming_buf[2];
    *((uint8_t *)(&encoder)+1)       = m_incoming_buf[3];
    *(uint8_t *)(&encoderRaw)        = m_incoming_buf[4];
    *((uint8_t *)(&encoderRaw)+1)    = m_incoming_buf[5];
    *(uint8_t *)(&encoderOffset)     = m_incoming_buf[6];
    *((uint8_t *)(&encoderOffset)+1) = m_incoming_buf[7];

    m_relative_pulses = (int) encoder;
    m_absolute_pulses = (int) encoderRaw;
    m_encoder_offset  = (int) encoderOffset;
}


int RMDServoState::requestClearError(bool wait)
{
    unsigned char msg[8] = {
        0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };    
    if ( !(CAN_OK == m_pcan_master->sendMsgBuf(device_id, 0, 8, msg)) ) { 
        return 0; 
    } else {
        return wait ? waitForFrame() : 1;
    }
}
void RMDServoState::requestClearErrorResponse(void)
{
    int8_t temperature;
    uint16_t voltage;

    *(uint8_t *)(&temperature) = m_incoming_buf[1];
    *(uint8_t *)(&voltage)     = m_incoming_buf[3];
    *((uint8_t *)(&voltage)+1) = m_incoming_buf[4];

    m_temperature = temperature;
    m_voltage = 0.1*voltage;
}


int RMDServoState::requestVelocity(long int dps_100, bool wait)
{
    unsigned char msg[8] = {
        0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    int32_t speedControl = dps_100;
    msg[4] = *(uint8_t *)(&speedControl);
    msg[5] = *((uint8_t *)(&speedControl)+1);
    msg[6] = *((uint8_t *)(&speedControl)+2);
    msg[7] = *((uint8_t *)(&speedControl)+3);
    if ( !(CAN_OK == m_pcan_master->sendMsgBuf(device_id, 0, 8, msg)) ) { 
        return 0; 
    } else {
        return wait ? waitForFrame() : 1;
    }
}
void RMDServoState::requestVelocityResponse(void)
{
    int8_t temperature;
    int16_t iq, speed;
    uint16_t encoder;

    *(uint8_t *)(&temperature) = m_incoming_buf[1];
    *(uint8_t *)(&iq) = m_incoming_buf[2];
    *((uint8_t *)(&iq)+1) = m_incoming_buf[3];
    *(uint8_t *)(&speed) = m_incoming_buf[4];
    *((uint8_t *)(&speed)+1) = m_incoming_buf[5];
    *(uint8_t *)(&encoder) = m_incoming_buf[6];
    *((uint8_t *)(&encoder)+1) = m_incoming_buf[7];

    m_temperature = temperature;
    m_current = iq;
    m_degrees_per_sec = speed;
    // m_absolute_pulses = encoder;
}


int RMDServoState::requestMotorOff(bool wait)
{
    unsigned char msg[8] = {
        0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };    
    if ( !(CAN_OK == m_pcan_master->sendMsgBuf(device_id, 0, 8, msg)) ) { 
        return 0; 
    } else {
        return wait ? waitForFrame() : 1;
    }
}
void RMDServoState::requestMotorOffResponse(void)
{
    
}