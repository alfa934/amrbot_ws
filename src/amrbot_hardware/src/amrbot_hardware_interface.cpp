#include "amrbot_hardware/amrbot_hardware_interface.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace amrbot_hardware
{

hardware_interface::CallbackReturn AmrbotHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    const auto & info = params.hardware_info;

    if (info.joints.size() != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AmrbotHardwareInterface"),
                     "Expected 2 joints, got %zu", info.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    bool found_left = false;
    bool found_right = false;

    for (const auto & joint : info.joints)
    {
        if (joint.name == "left_wheel_joint")
        {
            left_wheel_joint_ = joint.name;
            found_left = true;
        }
        else if (joint.name == "right_wheel_joint")
        {
            right_wheel_joint_ = joint.name;
            found_right = true;
        }
    }

    if (!found_left || !found_right)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AmrbotHardwareInterface"),
                     "Could not find required joints 'left_wheel_joint' and 'right_wheel_joint'");
        return hardware_interface::CallbackReturn::ERROR;
    }

    hw_position_states_.resize(2, 0.0);
    hw_velocity_states_.resize(2, 0.0);
    hw_velocity_commands_.resize(2, 0.0);

    auto node = this->get_node();
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    imu_timer_ = node->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&AmrbotHardwareInterface::publishImu, this));

    RCLCPP_INFO(rclcpp::get_logger("AmrbotHardwareInterface"),
                "IMU publisher created on topic /imu");

    RCLCPP_INFO(rclcpp::get_logger("AmrbotHardwareInterface"),
                "Initialized with left wheel '%s' and right wheel '%s'",
                left_wheel_joint_.c_str(), right_wheel_joint_.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
AmrbotHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
        left_wheel_joint_, hardware_interface::HW_IF_POSITION, &hw_position_states_[0]);
    state_interfaces.emplace_back(
        left_wheel_joint_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[0]);

    state_interfaces.emplace_back(
        right_wheel_joint_, hardware_interface::HW_IF_POSITION, &hw_position_states_[1]);
    state_interfaces.emplace_back(
        right_wheel_joint_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[1]);

    state_interfaces.emplace_back("sensor", "yaw", &hw_yaw_);

    state_interfaces.emplace_back("sensor", "gpio", &hw_gpio_);

    state_interfaces.emplace_back("sensor", "driver_ready", &hw_driver_ready_);

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
AmrbotHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
        left_wheel_joint_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[0]);
    command_interfaces.emplace_back(
        right_wheel_joint_, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[1]);

    return command_interfaces;
}

hardware_interface::CallbackReturn AmrbotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    if (!openSerial(serial_port_, serial_baudrate_))
    {
        RCLCPP_ERROR(rclcpp::get_logger("AmrbotHardwareInterface"),
                     "Failed to open serial port");
        return hardware_interface::CallbackReturn::ERROR;
    }

    parse_state_ = WAITING_A;
    payload_bytes_read_ = 0;
    memset(&rx_data_, 0, sizeof(rx_data_));
    last_serial_rx_ = rclcpp::Clock().now().seconds();

    first_read_ = true;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AmrbotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    m_serial_.closeDevice();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AmrbotHardwareInterface::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;

    if (rclcpp::Clock().now().seconds() - last_serial_rx_ > serial_timeout_sec_)
    {
        RCLCPP_WARN(rclcpp::get_logger("AmrbotHardwareInterface"),
                    "Serial timeout, restarting connection...");
        m_serial_.closeDevice();
        openSerial(serial_port_, serial_baudrate_);
        parse_state_ = WAITING_A;
        payload_bytes_read_ = 0;
        memset(&rx_data_, 0, sizeof(rx_data_));
        last_serial_rx_ = rclcpp::Clock().now().seconds();
        first_read_ = true;
        return hardware_interface::return_type::OK;
    }

    if (first_read_)
    {
        while (m_serial_.available() > 0)
        {
            char dummy;
            if (m_serial_.readBytes(&dummy, 1, 1, 1) != 1)
            {
                break;
            }
        }
        first_read_ = false;
        last_serial_rx_ = rclcpp::Clock().now().seconds();
        return hardware_interface::return_type::OK;
    }

    if (m_serial_.available() <= 0)
    {
        return hardware_interface::return_type::OK;
    }

    last_serial_rx_ = rclcpp::Clock().now().seconds();

    left_delta_ = 0.0;
    right_delta_ = 0.0;

    try
    {
        char byte;
        while (m_serial_.available() > 0)
        {
            if (m_serial_.readBytes(&byte, 1, 1, 1) != 1)
            {
                break;
            }

            if (processByteRx(byte)) //--- Full payload received
            {
                double left_delta_rad = rx_data_.enc_l_dt * counts_to_rad_;
                double right_delta_rad = rx_data_.enc_r_dt * counts_to_rad_;

                left_pos_ += left_delta_rad;
                right_pos_ += right_delta_rad;

                left_delta_ += left_delta_rad;
                right_delta_ += right_delta_rad;

                double raw_yaw = -rx_data_.yaw_deg;
                if (!yaw_offset_initialized_)
                {
                    yaw_offset_ = raw_yaw;
                    yaw_offset_initialized_ = true;
                    RCLCPP_INFO(rclcpp::get_logger("AmrbotHardwareInterface"),
                                "Yaw offset recorded: %.2f deg", yaw_offset_);
                }
                double actual_yaw = raw_yaw - yaw_offset_;

                while (actual_yaw > 180.0) actual_yaw -= 360.0;
                while (actual_yaw < -180.0) actual_yaw += 360.0;

                hw_yaw_ = actual_yaw;
                hw_gpio_ = rx_data_.gpio;
                hw_driver_ready_ = rx_data_.driver_ready;
            }
        }
    }
    catch (...)
    {
        RCLCPP_ERROR(rclcpp::get_logger("AmrbotHardwareInterface"),
                     "Exception while reading serial");
        parse_state_ = WAITING_A;
        payload_bytes_read_ = 0;
        return hardware_interface::return_type::ERROR;
    }

    hw_position_states_[0] = -left_pos_;
    hw_position_states_[1] = -right_pos_;

    if (period.seconds() > 0.0)
    {
        hw_velocity_states_[0] = left_delta_ / period.seconds();
        hw_velocity_states_[1] = right_delta_ / period.seconds();
    }
    else
    {
        hw_velocity_states_[0] = 0.0;
        hw_velocity_states_[1] = 0.0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type AmrbotHardwareInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    (void)time;
    (void)period;

    tx_data_.driver_start = 1;
    tx_data_.motor_start = 1;
    tx_data_.enc_l_reverse = 0;
    tx_data_.enc_r_reverse = 1;

    tx_data_.motor_l_speed =
        static_cast<int16_t>(-hw_velocity_commands_[0] * rad_s_to_motor_);
    tx_data_.motor_r_speed =
        static_cast<int16_t>(-hw_velocity_commands_[1] * rad_s_to_motor_);

    sendTxFrame();

    return hardware_interface::return_type::OK;
}

bool AmrbotHardwareInterface::openSerial(const char * port, int baudrate)
{
    int ret = m_serial_.openDevice(port, baudrate);
    if (ret == 1)
    {
        RCLCPP_INFO(rclcpp::get_logger("AmrbotHardwareInterface"),
                    "Serial port opened on %s at %d baud", port, baudrate);
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("AmrbotHardwareInterface"),
                     "openDevice() returned error %d", ret);
        return false;
    }
}

bool AmrbotHardwareInterface::processByteRx(char byte)
{
    bool payload_ready = false;

    switch (parse_state_)
    {
        case WAITING_A:
            if (byte == 'A')
            {
                parse_state_ = WAITING_B;
            }
            break;

        case WAITING_B:
            if (byte == 'B')
            {
                parse_state_ = WAITING_C;
            }
            else
            {
                parse_state_ = WAITING_A;
            }
            break;

        case WAITING_C:
            if (byte == 'C')
            {
                parse_state_ = READING_PAYLOAD;
                payload_bytes_read_ = 0;
            }
            else
            {
                parse_state_ = WAITING_A;
            }
            break;

        case READING_PAYLOAD:
            rx_buffer_[payload_bytes_read_++] = byte;

            if (payload_bytes_read_ == sizeof(rx_buffer_))
            {
                memcpy(&rx_data_, rx_buffer_, sizeof(RxPayload));
                payload_ready = true;
                parse_state_ = WAITING_A;
                payload_bytes_read_ = 0;
            }
            break;
    }

    return payload_ready;
}

void AmrbotHardwareInterface::sendTxFrame()
{
    // tx_buffer_ starts with "ABC"
    memcpy(tx_buffer_ + 3, &tx_data_.driver_start, 1);
    memcpy(tx_buffer_ + 4, &tx_data_.motor_start, 1);
    memcpy(tx_buffer_ + 5, &tx_data_.enc_l_reverse, 1);
    memcpy(tx_buffer_ + 6, &tx_data_.enc_r_reverse, 1);
    memcpy(tx_buffer_ + 7, &tx_data_.motor_l_speed, 2);
    memcpy(tx_buffer_ + 9, &tx_data_.motor_r_speed, 2);

    for (size_t i = 0; i < sizeof(tx_buffer_); i++)
    {
        if (m_serial_.isDeviceOpen())
        {
            try
            {
                m_serial_.writeChar(tx_buffer_[i]);
            }
            catch (...)
            {
                RCLCPP_WARN(rclcpp::get_logger("AmrbotHardwareInterface"),
                            "Unable to transmit serial");
            }
        }
    }
}

void AmrbotHardwareInterface::publishImu()
{
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->get_node()->now();
    imu_msg.header.frame_id = "imu_link";

    double yaw_rad = hw_yaw_ * M_PI / 180.0;
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = sin(yaw_rad / 2.0);
    imu_msg.orientation.w = cos(yaw_rad / 2.0);

    imu_msg.orientation_covariance[0] = 1e6;
    imu_msg.orientation_covariance[1] = 0.0;
    imu_msg.orientation_covariance[2] = 0.0;

    imu_msg.orientation_covariance[3] = 0.0;
    imu_msg.orientation_covariance[4] = 1e6;
    imu_msg.orientation_covariance[5] = 0.0;

    imu_msg.orientation_covariance[6] = 0.0;
    imu_msg.orientation_covariance[7] = 0.0;
    imu_msg.orientation_covariance[8] = 0.01;  

    imu_msg.angular_velocity_covariance[0] = 1e6;
    imu_msg.angular_velocity_covariance[4] = 1e6;
    imu_msg.angular_velocity_covariance[8] = 1e6;
    imu_msg.linear_acceleration_covariance[0] = 1e6;
    imu_msg.linear_acceleration_covariance[4] = 1e6;
    imu_msg.linear_acceleration_covariance[8] = 1e6;

    imu_pub_->publish(imu_msg);
}

}  // namespace amrbot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    amrbot_hardware::AmrbotHardwareInterface,
    hardware_interface::SystemInterface)