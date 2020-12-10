#include "RMDServoState.h"
#include "mcp_can.h"
#include <SPI.h>

#define CAN_INTERRUPT_PIN 2
#define JOY_VX_PIN A0
#define JOY_VY_PIN A2
#define JOY_SW_PIN 3
#define CAN_WAIT_TIMEOUT_US 500
#define CAN_SEND_PERIOD_US 10000     // 3333 gives about (reliably) 211 hz output to serial
#define DEBUG_PRINT 0
#define EFFECTIVE_GEAR_RATIO 0.14588859416445624f


long unsigned int rxId;
unsigned char flagRecv = 0;
unsigned char rxLen = 0;
unsigned char rxBuf[8];
unsigned long can_timer, read_timer, sw_timer;
volatile bool motor_enable = true;

int vx_bit=0, vy_bit=0;
float pitch=0.0, yaw=0.0, omega1=0.0, omega2=0.0;

MCP_CAN CAN(9); 
RMDServoState servo1(0x141, &CAN, CAN_WAIT_TIMEOUT_US);
RMDServoState servo2(0x142, &CAN, CAN_WAIT_TIMEOUT_US);


void setup() {
    Serial.begin(115200);

    pinMode(JOY_VX_PIN, INPUT);
    pinMode(JOY_VY_PIN, INPUT);
    pinMode(JOY_SW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(JOY_SW_PIN), ISR_motor_enable, FALLING);
    
    while (CAN_OK != CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ)) {           
        Serial.println("CAN BUS Shield init fail! Retrying...");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    CAN.setMode(MCP_NORMAL);

    if (
        !servo1.requestClearError(true) ||
        !servo1.requestEncoder(true)
    ) {
        Serial.println("Servo 1 init fail!");
    }
    if (
        !servo2.requestClearError(true) ||
        !servo2.requestEncoder(true)
    ) {
        Serial.println("Servo 2 init fail!");
    }

    /*
    * Motor status readings
    */
    // Serial.print("Voltage: ");
    Serial.print(servo1.getVoltage());
    Serial.print(", ");
    Serial.print(servo2.getVoltage());
    Serial.println();
    // Serial.print("Temperature: ");
    Serial.print(servo1.getTemperature());
    Serial.print(", ");
    Serial.print(servo2.getTemperature());
    Serial.println();
    // Serial.print("Encoder (absolute): ");
    Serial.print(servo1.getAbsEncoder());
    Serial.print(", ");
    Serial.print(servo2.getAbsEncoder());
    Serial.println();
    // Serial.print("Encoder (relative): ");
    Serial.print(servo1.getRelEncoder());
    Serial.print(", ");
    Serial.print(servo2.getRelEncoder());
    Serial.println();
    // Serial.print("Encoder offset: ");
    Serial.print(servo1.getEncoderOffset());
    Serial.print(", ");
    Serial.print(servo2.getEncoderOffset());
    Serial.println();

    sw_timer = millis();

    delay(100);

}


void ISR_motor_enable() {
    // if (millis() - sw_timer > 200) {
    //     if (motor_enable) {
    //         if (!servo1.requestMotorOff()) { 
    //             Serial.println("requestMotorOff to servo1 failed!");
    //         }
    //         if (!servo2.requestMotorOff()) { 
    //             Serial.println("requestMotorOff to servo2 failed!");
    //         }
    //     }
    //     sw_timer = millis();
    //     motor_enable = !motor_enable;
    // }   
    if (motor_enable) {
        Serial.println("Attempting to stop motors...");
        if (!servo1.requestMotorOff()) { 
            Serial.println("requestMotorOff to servo1 failed!");
        }
        if (!servo2.requestMotorOff()) { 
            Serial.println("requestMotorOff to servo2 failed!");
        }
    }
    motor_enable = !motor_enable;
}


void loop() {

    // Iterate over all frames in buffer 
    // read_timer = micros();  // elapsed time spent clearing buffer (~575us for 2 frames)
    // unsigned int timeout = 0;
    while (CAN_MSGAVAIL == CAN.checkReceive()) {
        // unsigned long current_loop_timer = micros();
        CAN.readMsgBuf(&rxId, &rxLen, rxBuf);
        int counter1 = 0, counter2 = 0;
        if (rxId == servo1.device_id) {
            counter1++;
            servo1.processFrame((int) rxLen, rxBuf);
        } 
        if (rxId == servo2.device_id) {
            counter2++;
            servo2.processFrame((int) rxLen, rxBuf);
        }
        // timeout += (micros() - current_loop_timer);
        if (DEBUG_PRINT) {
            Serial.print("Got ");
            Serial.print(counter1);
            Serial.println(" frames from 0x141!");
            Serial.print("Got ");
            Serial.print(counter2);
            Serial.println(" frames from 0x142!");
        }
    }
    // read_timer = micros() - read_timer;
    


    // Send commands to request encoder values
    if (motor_enable && (micros() - can_timer > CAN_SEND_PERIOD_US)) {

        // measure how long it takes to send command, typically ~8-10usec per msg
        // unsigned long send_timer = micros();

        // wait for response after sending request (slow, ~600usec per msg)
        // must be true if not checking for Rx in every loop()
        bool wait = false; 

        // if (!servo1.requestEncoder(wait)) { 
        //     Serial.println("requestEncoder to servo1 failed!");
        // }
        // if (!servo2.requestEncoder(wait)) {
        //     Serial.println("requestEncoder to servo2 failed!");
        // }

        readJoyStick();
        doInverseKinematics();
        if (!servo1.requestVelocity((long int) (omega1*100), wait)) { 
            Serial.println("requestVelocity to servo1 failed!");
        }
        if (!servo2.requestVelocity((long int) (omega2*100), wait)) { 
            Serial.println("requestVelocity to servo2 failed!");
        }
    
        // send_timer = micros() - send_timer;

        // Serial.print(send_timer);
        // Serial.print(", ");
        // Serial.print(read_timer);
        // Serial.print(", ");
        Serial.print(servo1.getCurrent());
        // Serial.print(omega1);
        Serial.print(", ");
        Serial.print(servo2.getCurrent());
        // Serial.print(omega2);
        Serial.println();

        can_timer = micros();
    }

}


void doInverseKinematics()
{
    omega1 = 0.5/EFFECTIVE_GEAR_RATIO*(pitch+yaw);
    omega2 = 0.5/EFFECTIVE_GEAR_RATIO*(-pitch+yaw);
}


void readJoyStick() 
{
    vx_bit = analogRead(JOY_VX_PIN);
    vy_bit = analogRead(JOY_VY_PIN);
    pitch = float(map(vx_bit, 0, 1023, -200, 200));
    yaw = float(map(vy_bit, 0, 1023, -230, 230));
    pitch = abs(pitch) < 40 ? 0.0 : pitch;
    yaw = abs(yaw) < 40 ? 0.0 : yaw;
}
