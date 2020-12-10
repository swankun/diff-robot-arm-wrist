#ifndef RMDServoState_h
#define RMDServoState_h

#include "Arduino.h"
#include "mcp_can.h"

class RMDServoState
{
    public:
        RMDServoState(unsigned long can_id, MCP_CAN *can_master, int timeout_set);

        unsigned long device_id;
        int TIMEOUT;
        
        long int getAbsEncoder(void);
        long int getRelEncoder(void);
        long int getEncoderOffset(void);
        int getTemperature(void);
        float getVoltage(void);
        int getCurrent(void);

        int processFrame(int len, unsigned char *buf);
        int requestEncoder(bool wait=false);
        int requestClearError(bool wait=false);
        int requestPosition(int deg_100, bool wait=false);
        int requestVelocity(long int dps_100, bool wait=false);
        int requestMotorOff(bool wait=false);

    //     int get_temperature();
    //     int get_voltage();
    //     int get_angle();
    //     int get_velocity();
    //     int get_torque();
    //     int get_desired_angle();
    //     int get_desired_velocity();
    //     int get_desired_torque();

    //     int set_desired_angle();
    //     int set_desired_velocity();
    //     int set_desired_torque();
        

    private:
        MCP_CAN *m_pcan_master;
        unsigned char m_incoming_buf[8];
        long int m_encoder_offset;
        long int m_absolute_pulses;
        long int m_relative_pulses;
        long int m_degrees_per_sec;
        int m_current;
        float m_voltage;
        int m_amperage;
        int m_temperature;
        unsigned long m_timeout_counter;
        int waitForFrame(void);
        void requestEncoderResponse(void);
        void requestClearErrorResponse(void);
        void requestPositionResponse(void);
        void requestVelocityResponse(void);
        void requestMotorOffResponse(void);
};

#endif  // RMDServoState_h
