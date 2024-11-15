#include "actuator.hpp"

#define GPIO_TWAI_TX GPIO_NUM_20
#define GPIO_TWAI_RX GPIO_NUM_21

#include "esp_mac.h"
#include <string.h>
#include <iostream>
using namespace std;


namespace TMotorActuators
{
  Actuator::Actuator(uint8_t motorId, MotorParameters motorParas, SendCanDataFunction canSendFunc)
  {
    mId = motorId;
    mMotorParameters = motorParas;
    sendCanData = canSendFunc;
    //gpio_pin = pin;

    disable();
  }

  void Actuator::enable(void)
  {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    sendCanData(mId, 8, data);
  }

  void Actuator::disable(void)
  {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    sendCanData(mId, 8, data);
  }

  void Actuator::setZeroPosition(void)
  {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    sendCanData(mId, 8, data);
  }

  void Actuator::move(float position, float velocity, float torque, float kp, float kd = 0){
    int32_t pInt = convFloatToUint(position, mMotorParameters.positionMin, mMotorParameters.positionMax, 16);
    int32_t vInt = convFloatToUint(velocity, mMotorParameters.velocityMin, mMotorParameters.velocityMax, 12);
    int32_t tInt = convFloatToUint(torque, mMotorParameters.torqueMin, mMotorParameters.torqueMax, 12);
    int32_t kpInt = convFloatToUint(kp, mMotorParameters.kpMin, mMotorParameters.kpMax, 12);
    int32_t kdInt = convFloatToUint(kd, mMotorParameters.kdMin, mMotorParameters.kdMax, 12);

    uint8_t data[8];
    data[0] = pInt >> 8;
    data[1] = pInt & 0xFF;
    data[2] = vInt >> 4;
    data[3] = ((vInt & 0xF) << 4 | (kpInt >> 8));
    data[4] = kpInt & 0xFF;
    data[5] = kdInt >> 4;
    data[6] = ((kdInt & 0xF) << 4 | (tInt >> 8));
    data[7] = tInt & 0xFF;

    sendCanData(mId, 8, data);
  }

    Actuator::MotorState Actuator::parseMotorState(uint8_t *canData)
  {
    // Check if the canData pointer is valid
    if (canData == nullptr) {
        ESP_LOGE("parseMotorState", "Received null data pointer");
        MotorState invalidState = {0}; 
        return invalidState;
    }

    // Check that the data length is sufficient (assuming 6 bytes are expected)
    if (sizeof(canData) < 6) {
        ESP_LOGE("parseMotorState", "Received invalid data length");
        MotorState invalidState = {0}; 
        return invalidState;
    }

    uint16_t pInt = (canData[1] << 8) | canData[2];
    uint16_t vInt = (canData[3] << 4) | (canData[4] >> 4);
    uint16_t tInt = ((canData[4] & 0xF) << 8) | canData[5];
    
    ESP_LOGI("parseMotorState", "Raw Position: %d, Velocity: %d, Torque: %d", pInt, vInt, tInt);

    MotorState state;
    state.id = canData[0];
    state.position = convUintToFloat(pInt, mMotorParameters.positionMin, mMotorParameters.positionMax, 16);
    state.velocity = convUintToFloat(vInt, mMotorParameters.velocityMin, mMotorParameters.velocityMax, 12);
    state.torque = convUintToFloat(tInt, -mMotorParameters.torqueMax, mMotorParameters.torqueMax, 12); // XXX

    ESP_LOGI("parseMotorState", "Parsed Motor State -> ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f", 
             state.id, state.position, state.velocity, state.torque);

    return state;
  }

  int32_t Actuator::convFloatToUint(float val, float min, float max, uint8_t bits)
  {
    if (val > max)
    {
      val = max;
    }
    else if (val < min)
    {
      val = min;
    }

    float span = max - min;
    return (int32_t)((val - min) * ((float)((1 << bits) - 1)) / span);
  }

  float Actuator::convUintToFloat(int32_t val, float min, float max, uint8_t bits)
  {
    float span = max - min;
    return ((float)val) * span / ((float)((1 << bits) - 1)) + min;
  }

  uint8_t Actuator::getId(void)
  {
    return this->mId;
  }


  void Actuator::init_can() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_TWAI_TX, GPIO_TWAI_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); // set can speed to 1 mbits
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); 

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI("CAN", "CAN driver installed and started successfully");
    cout << "Actuator CAN driver installed and started" << endl;
  }

  void Actuator::send_can_message() {
    twai_message_t message;
    message.identifier = this->can_id;   // need to use the actuator's CAN ID
    message.extd = 0;                
    message.data_length_code = 8;        // the message has 8 bytes of data

    for (int i = 0; i < 8; i++) {
        message.data[i] = can_data[i];  
    }

    ESP_ERROR_CHECK(twai_transmit(&message, pdMS_TO_TICKS(1000))); // wait for 1 second
    printf("Actuator CAN message with ID 0x%x transmitted\n", (unsigned int)message.identifier);
  }

  void Actuator::control_actuator(const char *action) {
    if (strcmp(action, "move") == 0) {
        gpio_set_level(gpio_pin, 1);
        printf(TAG, "Actuator moving (GPIO %d set HIGH)", gpio_pin);
    } else if (strcmp(action, "stop") == 0) {
        gpio_set_level(gpio_pin, 0); 
        printf(TAG, "Actuator stopped (GPIO %d set LOW)", gpio_pin);
    }
    send_can_message();
  }

}
