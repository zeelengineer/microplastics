#ifndef ACTUATOR_HPP
#define ACTUATOR_HPP

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"

namespace TMotorActuators
{
    typedef struct
    {
        float positionMin;
        float positionMax;

        float velocityMin;
        float velocityMax;

        float torqueMin;
        float torqueMax;

        float kpMin;
        float kpMax;

        float kdMin;
        float kdMax;
    } MotorParameters;

    const MotorParameters ak60_6_v1_80 = {-12.5,
                                       12.5,
                                       -41.87,
                                       41.87,
                                       -9,
                                       9,
                                       0,
                                       500,
                                       0,
                                       5};

    class Actuator{
        public:
            void enable(void);

            void disable(void);

            void setZeroPosition(void);

            void move(float position, float velocity, float torque, float kp, float kd);

            typedef void (*SendCanDataFunction)(uint32_t id, uint8_t dlc, uint8_t *data);

            typedef struct
            {
                uint16_t id;
                float position;
                float velocity;
                float torque;
            } MotorState;

            Actuator(uint8_t motorId, MotorParameters motorParas, SendCanDataFunction canSendFunc);

            MotorState parseMotorState(uint8_t *canData);

            uint8_t getId(void);

            static void init_can();

            void send_can_message();

            void control_actuator(const char *action);

        private:
            uint8_t mId;
            uint32_t can_id;                
            uint8_t can_data[8];          
            gpio_num_t gpio_pin;      
            static const char *TAG;

            MotorParameters mMotorParameters;

            SendCanDataFunction sendCanData;

            int32_t convFloatToUint(float val, float min, float max, uint8_t bits);

            float convUintToFloat(int32_t val, float min, float max, uint8_t bits);
    };

}

#endif // ACTUATOR_HPP