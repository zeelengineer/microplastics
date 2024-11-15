#include <stdio.h>
#include <iostream>
#include <iomanip>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "actuator.hpp"
using namespace TMotorActuators;
using namespace std;


//#define MOTOR_ID 0x01  // Example motor ID
static const char *TAG = "Main";

void sendCanData(uint32_t id, uint8_t dlc, uint8_t *data) {
    twai_message_t message;
    message.identifier = id;
    message.extd = 0; // Standard frame
    message.data_length_code = dlc;

    for (int i = 0; i < dlc; i++) {
        message.data[i] = data[i];
    }

    ESP_ERROR_CHECK(twai_transmit(&message, pdMS_TO_TICKS(1000)));
    /*
    esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        cout << TAG << ": sendCanData - Failed to install CAN driver: %s" << esp_err_to_name(err) << endl;
    }  
    cout << TAG << ": sendCanData - CAN message with ID 0x" << hex << message.identifier << " transmitted" << endl;
    */
    ESP_LOGI(TAG, "sendCanData - CAN message with ID 0x%X transmitted", (unsigned int)message.identifier);

}

// receiving and logging motor state
void receiveMotorState(Actuator& motor) {
    twai_message_t message;

    esp_err_t err = twai_receive(&message, pdMS_TO_TICKS(1000)); // wait 1 second
    cout << TAG << ": " << err << endl;
    if (err == ESP_OK) {
        if (message.identifier == 1) { //need to change the 1 to motor_id
            Actuator::MotorState motorState = motor.parseMotorState(message.data);
            cout << TAG << ": receiveMotorState - Received Motor Position: " << fixed << setprecision(2) << motorState.position << ", Velocity: " << motorState.velocity 
            << ", Torque: " << motorState.torque << endl;
        }
    } else if (err == ESP_ERR_TIMEOUT) {
        cout << TAG << ": receiveMotorState - CAN receive timeout" << endl;
    } else {
        cout << TAG << ": receiveMotorState - CAN receive error: " << esp_err_to_name(err) << endl;
    }
}

extern "C" void app_main() {
    // 1. Initialize CAN
    Actuator::init_can();
    cout << TAG << ": CAN initialized" << endl;

    // 2. Set up GPIO control pin
    //gpio_set_direction(GPIO_MODE_OUTPUT);
    
    // 3. Set motor parameters and create an actuator instance
    MotorParameters motorParams = ak60_6_v1_80;  // Using predefined motor parameters
    Actuator motor(1, motorParams, sendCanData); //Have motor_id as first parameter

    // 4. Enable the motor and set zero position
    motor.enable();
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor.setZeroPosition();
    cout << TAG << ": Motor enabled and zero position set" << endl;
    /*
    // 5. Test sending a move command to the motor
    float testPosition = 0.5;   // Example position
    float testVelocity = 0.5;   // Example velocity
    float testTorque = 0.0;     // Example torque
    float testKp = 100.0;       // Example proportional gain
    float testKd = 1.0;         // Example derivative gain

    cout << TAG << ": Sending move command to motor" << endl;
    motor.move(testPosition, testVelocity, testTorque, testKp, testKd);


    receiveMotorState(motor);
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (true) {
        // You could read position feedback here if needed
        receiveMotorState(motor);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for repeated actions
    }
    */
    float velocity = 10.0; 
    float position = 0.0; 
    float torque = 0.0;   
    float kp = 0.0;       
    float kd = 0.0;      

    // 5. Loop to move the motor continuously for 30 seconds
    cout << TAG << ": Moving motor continuously for 30 seconds" << endl;
    int duration_ms = 30000; // 30 seconds in milliseconds
    int elapsed_time = 0;
    int command_interval = 10; // Interval to send commands in milliseconds (100 Hz)

    while (elapsed_time < duration_ms) {
        motor.move(position, velocity, torque, kp, kd);
        receiveMotorState(motor);
        vTaskDelay(pdMS_TO_TICKS(command_interval));
        elapsed_time += command_interval;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    motor.setZeroPosition();
    receiveMotorState(motor);
}