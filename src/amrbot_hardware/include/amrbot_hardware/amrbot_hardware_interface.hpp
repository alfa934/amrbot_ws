#ifndef AMRBOT_HARDWARE__AMRBOT_HARDWARE_INTERFACE_HPP_
#define AMRBOT_HARDWARE__AMRBOT_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <cstring>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include "amrbot_hardware/serialib.h"

namespace amrbot_hardware
{

#pragma pack(push, 1)
struct RxPayload
{
    uint8_t driver_ready;
    int16_t enc_l_dt;
    int16_t enc_r_dt;
    uint8_t gpio;
    float   yaw_deg;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TxPayload
{
    uint8_t driver_start;
    uint8_t motor_start;
    uint8_t enc_l_reverse;
    uint8_t enc_r_reverse;
    int16_t motor_l_speed;
    int16_t motor_r_speed;
};
#pragma pack(pop)

enum ParserState
{
    WAITING_A,
    WAITING_B,
    WAITING_C,
    READING_PAYLOAD
};

class AmrbotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(AmrbotHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time,
        const rclcpp::Duration & period) override;

private:
    bool openSerial(const char * port, int baudrate);
    bool processByteRx(char byte);
    void sendTxFrame();

    static constexpr const char * serial_port_ = "/dev/ttyUSB0";
    static constexpr int serial_baudrate_ = 115200;
    static constexpr double serial_timeout_sec_ = 2.0;

    serialib m_serial_;

    bool first_read_ = true;

    ParserState parse_state_ = WAITING_A;
    int payload_bytes_read_ = 0;
    char rx_buffer_[sizeof(RxPayload)] = {};
    RxPayload rx_data_{};
    TxPayload tx_data_{};
    char tx_buffer_[sizeof(TxPayload) + 3] = "ABC";

    double last_serial_rx_ = 0.0;

    std::string left_wheel_joint_;
    std::string right_wheel_joint_;

    std::vector<double> hw_position_states_;
    std::vector<double> hw_velocity_states_;
    std::vector<double> hw_velocity_commands_;
    

    double left_pos_ = 0.0;
    double right_pos_ = 0.0;
    double left_delta_ = 0.0;
    double right_delta_ = 0.0;

    double counts_to_rad_ = 0.00248;      // encoder counts -> wheel radians
    double rad_s_to_motor_ = 6.0;     // wheel rad/s -> motor speed units

    double hw_yaw_ = 0.0;
    double hw_gpio_ = 0.0;
    double hw_driver_ready_ = 0.0;

    bool yaw_offset_initialized_ = false;
    double yaw_offset_ = 0.0;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    void publishImu();

};

}  // namespace amrbot_hardware

#endif  // AMRBOT_HARDWARE__AMRBOT_HARDWARE_INTERFACE_HPP_