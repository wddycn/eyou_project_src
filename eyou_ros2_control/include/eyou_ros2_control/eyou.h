#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <cmath>
#include <cstdint>
#include <thread>
#include <chrono>
#include "vector"
#include <fcntl.h>
#include <sys/select.h>

class MotorCAN
{
public:
    explicit MotorCAN(const std::string& can_name = "can0")
        : can_name_(can_name), socket_fd_(-1)
    {}

    ~MotorCAN()
    {
        closeSocket();
    }

    /* =========================================================
     * 1. 激活 CAN 总线
     * sudo ip link set can0 type can bitrate 1000000
     * sudo ip link set can0 up
     * ========================================================= */
    bool bringUpCAN(int bitrate = 1000000)
    {
        std::string cmd1 =
            "sudo ip link set " + can_name_ +
            " type can bitrate " + std::to_string(bitrate);
        std::string cmd2 =
            "sudo ip link set " + can_name_ + " up";

        if (system(cmd1.c_str()) != 0)
        {
            std::cerr << "[MotorCAN] Failed to set CAN bitrate\n";
            return false;
        }

        if (system(cmd2.c_str()) != 0)
        {
            std::cerr << "[MotorCAN] Failed to bring CAN up\n";
            return false;
        }

        return true;
    }

    /* =========================================================
     * 2. 释放 CAN 总线
     * sudo ip link set can0 down
     * ========================================================= */
    bool bringDownCAN()
    {
        std::string cmd =
            "sudo ip link set " + can_name_ + " down";

        if (system(cmd.c_str()) != 0)
        {
            std::cerr << "[MotorCAN] Failed to bring CAN down\n";
            return false;
        }

        return true;
    }

    /* =========================================================
     * 打开 CAN Socket
     * ========================================================= */
    bool openSocket()
    {
        if (socket_fd_ != -1)
            return true;

        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0)
        {
            perror("[MotorCAN] socket");
            return false;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_name_.c_str(), IFNAMSIZ);

        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
        {
            perror("[MotorCAN] ioctl");
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        struct sockaddr_can addr {};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            perror("[MotorCAN] bind");
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        /* 新增：设置为非阻塞 */
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        return true;
    }
    /* =========================================================
     * 关闭 Socket
     * ========================================================= */
    void closeSocket()
    {
        if (socket_fd_ != -1)
        {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    /* =========================================================
     * 发送 CAN 帧
     * ========================================================= */
    bool sendFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
    {
        if (socket_fd_ < 0)
        {
            std::cerr << "[MotorCAN] Socket not opened\n";
            return false;
        }

        struct can_frame frame {};
        frame.can_id  = can_id;
        frame.can_dlc = dlc;
        std::memcpy(frame.data, data, dlc);

        ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
        return (nbytes == sizeof(frame));
    }

    /* =========================================================
     * 接收 CAN 帧（阻塞）
     * ========================================================= */
    bool receiveFrame(struct can_frame& frame, int timeout_ms = 100)
    {
        if (socket_fd_ < 0)
            return false;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(socket_fd_, &readfds);

        struct timeval timeout;
        timeout.tv_sec  = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(socket_fd_ + 1, &readfds, nullptr, nullptr, &timeout);

        if (ret <= 0)
        {
            // ret == 0 timeout
            // ret < 0 error
            return false;
        }

        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));

        return (nbytes == sizeof(frame));
    }

    bool receiveFrameID(struct can_frame& frame,
                        uint32_t expected_id,
                        int timeout_ms = 100)
    {
        auto start = std::chrono::steady_clock::now();

        while (true)
        {
            if (!receiveFrame(frame, timeout_ms))
                return false;

            if (frame.can_id == expected_id)
                return true;

            auto now = std::chrono::steady_clock::now();
            int elapsed =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - start).count();

            if (elapsed > timeout_ms)
                return false;
        }
    }
    /* =========================================================
     * 3. 使能某个电机
     * 示例：
     * ID   : 0x008
     * DATA : 01 10 00 00 00 01 00 00
     * ========================================================= */
    bool enableMotor(uint8_t motor_id)
    {
        if (!openSocket())
            return false;

        uint8_t data[8] = {
            0x01, 0x10, 0x00, 0x00,
            0x00, 0x01, 0x00, 0x00
        };

        uint32_t can_id = motor_id;

        if (!sendFrame(can_id, data, 8))
        {
            std::cerr << "[MotorCAN] Failed to send enable command\n";
            return false;
        }

        // 等待反馈
        struct can_frame recv_frame {};
        if (!receiveFrame(recv_frame, motor_id))
        {
            std::cerr << "[MotorCAN] No response from motor\n";
            return false;
        }

        // 简单校验反馈
        if (recv_frame.can_dlc == 3 &&
            recv_frame.data[0] == 0x02 &&
            recv_frame.data[1] == 0x10 &&
            recv_frame.data[2] == 0x01)
        {
            return true;
        }

        std::cerr << "[MotorCAN] Enable response invalid\n";
        return false;
    }

    /* =========================================================
     * 4. 失能某个电机
     * 示例：
     * ID   : 0x008
     * DATA : 01 10 00 00 00 00 00 00
     * ========================================================= */
    bool disableMotor(uint8_t motor_id)
    {
        if (!openSocket())
            return false;

        uint8_t data[8] = {
            0x01, 0x10, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        };

        uint32_t can_id = motor_id;

        if (!sendFrame(can_id, data, 8))
        {
            std::cerr << "[MotorCAN] Failed to send disable command\n";
            return false;
        }

        // 等待反馈
        struct can_frame recv_frame {};
        if (!receiveFrame(recv_frame, motor_id))
        {
            std::cerr << "[MotorCAN] No response from motor\n";
            return false;
        }

        // 简单校验反馈
        if (recv_frame.can_dlc == 3 &&
            recv_frame.data[0] == 0x02 &&
            recv_frame.data[1] == 0x10 &&
            recv_frame.data[2] == 0x01)
        {
            return true;
        }

        std::cerr << "[MotorCAN] Disable response invalid\n";
        return false;
    }



    /* =========================================================
    * 读取电机当前位置（单位：弧度）
    *
    * 发送:
    *   ID   : motor_id
    *   DATA : 03 07 00 00 00 00 00 00
    *
    * 接收:
    *   ID   : motor_id
    *   DATA : 04 07 [pos3 pos2 pos1 pos0] 00 00
    *
    * 65536 pulses = 360 deg
    * ========================================================= */
    double readPosition(uint8_t motor_id)
    {
        if (!openSocket())
            return false;

        uint8_t tx_data[8] = {
            0x03, 0x07,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        };

        if (!sendFrame(motor_id, tx_data, 8))
            return false;

        struct can_frame rx_frame {};
        if (!receiveFrame(rx_frame, motor_id))
            return false;

        if (rx_frame.can_dlc < 6 ||
            rx_frame.data[0] != 0x04 ||
            rx_frame.data[1] != 0x07)
            return false;

        /* ---------- 脉冲解析 ---------- */
        uint32_t angle_pulses =
            (static_cast<uint32_t>(rx_frame.data[2]) << 24) |
            (static_cast<uint32_t>(rx_frame.data[3]) << 16) |
            (static_cast<uint32_t>(rx_frame.data[4]) << 8)  |
            (static_cast<uint32_t>(rx_frame.data[5]));

        /* ---------- raw angle ---------- */
        double raw_angle = angle_pulses * 360.0 / 65536.0;

        /* ---------- wrap to [-180, 180) ---------- */
        double angle_mod = std::fmod(raw_angle + 180.0, 360.0);
        if (angle_mod < 0)
            angle_mod += 360.0;
        angle_mod -= 180.0;

        double angle_rad = angle_mod * M_PI / 180.0;
        return angle_rad; // 返回弧度值
    }

    /* =========================================================
    * 读取电机当前速度（单位：rad/s）
    * 
    * 修复说明:
    *   将解析变量从 uint32_t 改为 int32_t，以支持反向速度（负数）。
    *   协议中的速度值通常为 32位有符号整数 (Two's Complement)。
    * ========================================================= */
    double readSpeed(uint8_t motor_id)
    {
        if (!openSocket())
            return 0.0; // 失败返回 0.0

        uint8_t tx_data[8] = {
            0x03, 0x06,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        };

        if (!sendFrame(motor_id, tx_data, 8))
            return 0.0;

        struct can_frame rx_frame {};
        if (!receiveFrame(rx_frame, motor_id))
            return 0.0;

        // 建议：增加 ID 校验，防止收到其他电机的回复
        if (rx_frame.can_id != motor_id) {
            return 0.0;
        }

        if (rx_frame.can_dlc < 6 ||
            rx_frame.data[0] != 0x04 ||
            rx_frame.data[1] != 0x06)
        {
            return 0.0;
        }

        /* ---------- 脉冲解析 (关键修改) ---------- */
        // 使用 int32_t 接收，这样如果最高位是 1 (负数)，它会被正确解析为负值
        int32_t speed_param =
            (static_cast<int32_t>(rx_frame.data[2]) << 24) |
            (static_cast<int32_t>(rx_frame.data[3]) << 16) |
            (static_cast<int32_t>(rx_frame.data[4]) << 8)  |
            (static_cast<int32_t>(rx_frame.data[5]));

        /* ---------- 单位转换 ---------- */
        // 原始公式: rpm = param * 60 / 65536
        // 目标公式: rad/s = rpm * 2 * PI / 60
        // 合并简化: rad/s = param * (2 * PI) / 65536
        
        // 注意：speed_param 现在是有符号的，如果它是负数，计算结果自动为负
        const double SCALE_FACTOR = (2.0 * M_PI) / 65536.0;
        double rad_speed = static_cast<double>(speed_param) * SCALE_FACTOR;

        return rad_speed;
    }

    /* =========================================================
    * 辅助函数：获取电机额定扭矩
    * 避免代码重复，统一管理配置
    * ========================================================= */
    static float getMotorRatedTorque(uint8_t motor_id) {
        switch (motor_id) {
            case 2: case 3: return 5.4f;
            case 4: case 5: return 5.4f;
            case 6: case 7: return 2.4f; 
            default:
                std::cerr << "[MotorCAN] Unknown motor ID: " << (int)motor_id << "\n";
                return 0.0f;
        }
    }

    /* =========================================================
    * 函数 1: 读取原始千分比整数
    *
    * 返回:
    *   int32_t : 原始千分比值 (例如 500 代表 50%)
    *             如果读取失败，返回 INT32_MIN 作为错误标志
    * ========================================================= */
    int32_t getTorqueRatioRaw(uint8_t motor_id)
    {
        if (!openSocket())
            return INT32_MIN;

        /* ---------- 1. 发送指令 ---------- */
        uint8_t tx_data[8] = {
            0x03, 0x05,
            0x00, 0x00, 0x00, 0x00,
            0x00, 0x00
        };

        if (!sendFrame(motor_id, tx_data, 8))
            return INT32_MIN;

        /* ---------- 2. 接收反馈 ---------- */
        struct can_frame rx_frame {};
        if (!receiveFrame(rx_frame, motor_id))
            return INT32_MIN;

        /* ---------- 3. 校验响应 ---------- */
        if (rx_frame.can_dlc < 6 ||
            rx_frame.data[0] != 0x04 ||
            rx_frame.data[1] != 0x05) {
            return INT32_MIN;
        }

        /* ---------- 4. 解析 int32 (大端 + 有符号) ---------- */
        int32_t raw_val =
            (static_cast<int32_t>(rx_frame.data[2]) << 24) |
            (static_cast<int32_t>(rx_frame.data[3]) << 16) |
            (static_cast<int32_t>(rx_frame.data[4]) << 8)  |
            (static_cast<int32_t>(rx_frame.data[5]));

        return raw_val;
    }

    /* =========================================================
    * 函数 2: 读取实际扭矩 (N·m)
    *
    * 逻辑:
    *   1. 调用 getTorqueRatioRaw 获取原始值。
    *   2. 查询额定扭矩。
    *   3. 换算并返回。
    *
    * 返回:
    *   double : 实际扭矩 (N·m)
    *            读取失败返回 NAN
    * ========================================================= */
    double readTorque(uint8_t motor_id)
    {
        // 1. 获取原始千分比
        int32_t raw_ratio = getTorqueRatioRaw(motor_id);

        // 检查是否失败 (INT32_MIN 是错误标志)
        if (raw_ratio == INT32_MIN) {
            return std::nan("");
        }

        // 2. 获取额定扭矩
        float rated_nm = getMotorRatedTorque(motor_id);
        if (rated_nm <= 0.0f) {
            return std::nan("");
        }

        // 3. 换算公式
        double actual_nm = (static_cast<double>(raw_ratio) / 1000.0) * rated_nm;

        return actual_nm;
    }


    /* =========================================================
    * 设置速度模式并给定速度值（单位：rad/s）
    * 弧度/秒，一圈 = 6.283rad
    * 峰值转速 ： 4.0（rad/s）
    * ========================================================= */
    bool setSpeedRad(uint8_t motor_id, double rad_per_sec)
    {
        if (!openSocket())
            return false;

        /* ---------- Step 1: 切换到速度模式 ---------- */
        uint8_t mode_cmd[8] = {
            0x01, 0x0F,
            0x00, 0x00, 0x00, 0x03,
            0x00, 0x00
        };

        if (!sendFrame(motor_id, mode_cmd, 8))
            return false;

        struct can_frame rx_frame {};
        if (!receiveFrame(rx_frame, motor_id) ||
            rx_frame.can_dlc < 3 ||
            rx_frame.data[0] != 0x02 ||
            rx_frame.data[1] != 0x0F ||
            rx_frame.data[2] != 0x01)
            return false;

        /* ---------- Step 2: rad/s → 协议速度单位 ---------- */
        constexpr double TWO_PI = 6.283185307179586;
        int32_t speed_value = static_cast<int32_t>(
            rad_per_sec * 65536.0 / TWO_PI
        );

        uint8_t speed_cmd[8] = {
            0x01, 0x09,
            static_cast<uint8_t>((speed_value >> 24) & 0xFF),
            static_cast<uint8_t>((speed_value >> 16) & 0xFF),
            static_cast<uint8_t>((speed_value >> 8)  & 0xFF),
            static_cast<uint8_t>( speed_value        & 0xFF),
            0x00, 0x00
        };

        if (!sendFrame(motor_id, speed_cmd, 8))
            return false;

        if (!receiveFrame(rx_frame, motor_id) ||
            rx_frame.can_dlc < 3 ||
            rx_frame.data[0] != 0x02 ||
            rx_frame.data[1] != 0x09 ||
            rx_frame.data[2] != 0x01)
            return false;

        return true;
    }
 
    /* =========================================================
    * 轮廓位置控制（Profile Position Mode）- 已更新力矩限制逻辑
    *
    * 执行顺序：
    * 1. motor_id : 2~13
    * 2. 目标角度（rad）
    * 3. 目标速度（rad/s）峰值转速 4.0rad/s
    * 4. 目标力矩（Nm）额定2.4/5.4
    * 5. 设置目标加速度（rad/s²）
    * 6. 设置目标减速度（rad/s²）
    * ========================================================= */
    bool setProfilePosition(
        uint8_t motor_id,
        double target_rad,          // 目标位置 (rad)
        double speed_rad_s,         // 目标速度 (rad/s)
        float target_torque_nm,     // 【修改】目标力矩限制 (N·m)，例如 3.3f
        double accel_rad_s2,        // 目标加速度 (rad/s²)
        double decel_rad_s2)        // 目标减速度 (rad/s²)
    {
        if (!openSocket())
            return false;

        struct can_frame rx {};

        /* --- 辅助宏：安全的大端序转换 (32-bit Signed) --- */
        #define TO_BIG_ENDIAN_BYTES(val, arr, offset) \
            do { \
                uint32_t uval = static_cast<uint32_t>(val); \
                arr[offset]   = (uval >> 24) & 0xFF; \
                arr[offset+1] = (uval >> 16) & 0xFF; \
                arr[offset+2] = (uval >> 8)  & 0xFF; \
                arr[offset+3] = (uval)       & 0xFF; \
            } while(0)

        /* --- 辅助函数：获取额定扭矩 (避免代码重复) --- */
        auto getRatedTorque = [](uint8_t id) -> float {
            switch (id) {
                case 2: case 3: return 5.4f;
                case 4: case 5: return 5.4f;
                case 6: case 7: return 2.4f; 
                default:          return 0.0f;
            }
        };

        /* 1. 工作模式：Profile Position (0x04) */
        // 注意：原代码写的是 0x01，但注释说是 Profile Position。
        // 通常 CiA 402 中 Profile Pos 是 0x01，但你的 setTorqueMode 里用的是 0x04。
        // 这里保持你原代码的 0x01，如果实际设备需要 0x04 请自行调整。
        uint8_t mode_cmd[8] = {0x01, 0x0F, 0, 0, 0, 0x01, 0, 0}; 
        if (!sendFrame(motor_id, mode_cmd, 8)) return false;
        if (!receiveFrame(rx, motor_id)) return false;
        if (rx.data[0]!=0x02 || rx.data[1]!=0x0F || rx.data[2]!=0x01)
            return false;

        /* 2. 目标速度：rad/s → 寄存器值 (假设系数 1092 unit/rpm) */
        double speed_rpm = speed_rad_s * (30.0 / M_PI);
        int32_t speed_value = static_cast<int32_t>(std::round(speed_rpm * 1092.0));
        
        uint8_t speed_cmd[8] = {0x01, 0x09, 0, 0, 0, 0, 0, 0};
        TO_BIG_ENDIAN_BYTES(speed_value, speed_cmd, 2);

        if (!sendFrame(motor_id, speed_cmd, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0]!=0x02 || rx.data[1]!=0x09 || rx.data[2]!=0x01)
            return false;

        /* ============================================
        * 3. 【核心修改】力矩限制：N·m → 千分比 → 寄存器值
        * ============================================ */
        
        // A. 获取额定扭矩
        float rated_torque_nm = getRatedTorque(motor_id);
        if (rated_torque_nm <= 0.0f) {
            std::cerr << "[MotorCAN] Invalid motor ID for torque calculation in Profile Pos\n";
            return false;
        }

        // B. 计算千分比 (Target / Rated * 1000)
        float ratio = target_torque_nm / rated_torque_nm;
        int32_t torque_param = static_cast<int32_t>(std::round(ratio * 1000.0f));

        // C. 构建指令 (0x01 0x08)
        uint8_t torque_cmd[8] = {0x01, 0x08, 0, 0, 0, 0, 0, 0};
        TO_BIG_ENDIAN_BYTES(torque_param, torque_cmd, 2);

        if (!sendFrame(motor_id, torque_cmd, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0]!=0x02 || rx.data[1]!=0x08 || rx.data[2]!=0x01)
            return false;

        /* 4. 加速度：rad/s² → 寄存器值 */
        double accel_rpm_s = accel_rad_s2 * (30.0 / M_PI);
        int32_t accel_value = static_cast<int32_t>(std::round(accel_rpm_s * 1092.0));
        
        uint8_t accel_cmd[8] = {0x01, 0x0B, 0, 0, 0, 0, 0, 0};
        TO_BIG_ENDIAN_BYTES(accel_value, accel_cmd, 2);

        if (!sendFrame(motor_id, accel_cmd, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0]!=0x02 || rx.data[1]!=0x0B || rx.data[2]!=0x01)
            return false;

        /* 5. 减速度：rad/s² → 寄存器值 */
        double decel_rpm_s = decel_rad_s2 * (30.0 / M_PI);
        int32_t decel_value = static_cast<int32_t>(std::round(decel_rpm_s * 1092.0));
        
        uint8_t decel_cmd[8] = {0x01, 0x0C, 0, 0, 0, 0, 0, 0};
        TO_BIG_ENDIAN_BYTES(decel_value, decel_cmd, 2);

        if (!sendFrame(motor_id, decel_cmd, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0]!=0x02 || rx.data[1]!=0x0C || rx.data[2]!=0x01)
            return false;

        /* 6. 目标位置：rad → 脉冲数 (假设 1圈=65536) */
        double target_deg = target_rad * (180.0 / M_PI);
        int32_t pos_pulse = static_cast<int32_t>(std::round(target_deg / 360.0 * 65536.0));
        
        uint8_t pos_cmd[8] = {0x01, 0x0A, 0, 0, 0, 0, 0, 0};
        TO_BIG_ENDIAN_BYTES(pos_pulse, pos_cmd, 2);

        if (!sendFrame(motor_id, pos_cmd, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0]!=0x02 || rx.data[1]!=0x0A || rx.data[2]!=0x01)
            return false;

        return true;
    }

    /* =========================================================
    * 统一力矩控制函数 (支持多型号电机)
    * 
    * 逻辑:
    *   1. 根据 motor_id 查找对应的额定扭矩 (Rated Torque)。
    *   2. 计算千分比: value = (target_nm / rated_nm) * 1000。
    *   3. 发送 CAN 指令。
    *
    * 电机配置表:
    *   ID 8, 9  : 6.6 N·m
    *   ID 10, 11: 4.4 N·m
    *   ID 12, 13: 1.0 N·m
    * ========================================================= */

    bool setTorqueMode(uint8_t motor_id, float target_torque_nm)
    {
        if (!openSocket())
            return false;

        struct can_frame rx {};
        /* ---------- 1. 根据 ID 获取额定扭矩 ---------- */
        float rated_torque_nm = 0.0f;
        switch (motor_id) {
            case 2:
            case 3:
                rated_torque_nm = 5.4f;
                break;
            case 4:
            case 5:
                rated_torque_nm = 5.4f;
                break;
            case 6:
            case 7:
                rated_torque_nm = 2.4f;
                break;
            default:
                // 如果传入了未定义的电机ID，打印错误或直接返回失败
                // fprintf(stderr, "Error: Unknown motor ID %d\n", motor_id);
                return false;
        }
        // 安全检查：防止除以零
        if (rated_torque_nm <= 0.0f) {
            return false;
        }
        /* ---------- 2. 核心计算：N·m -> 千分比 ---------- */
        // 公式：(目标 / 额定) * 1000
        float ratio = target_torque_nm / rated_torque_nm;
        // 转换为 32位有符号整数 (四舍五入)
        int32_t torque_param = static_cast<int32_t>(std::round(ratio * 1000.0f));
        /* ---------- 3. Step 1: 设置模式 (0x0F) ---------- */
        uint8_t mode_cmd[8] = {
            0x01, 0x0F,
            0x00, 0x00, 0x00, 0x04,
            0x00, 0x00
        };
        if (!sendFrame(motor_id, mode_cmd, 8)) return false;
        if (!receiveFrame(rx, motor_id)) return false;
        // 校验回复 (02 0F 01 表示成功)
        if (rx.can_id != motor_id || 
            rx.data[0] != 0x02 || 
            rx.data[1] != 0x0F || 
            rx.data[2] != 0x01) {
            return false;
        }
        /* ---------- 4. Step 2: 发送力矩值 (0x08) ---------- */
        uint8_t torque_cmd[8] = {0};
        torque_cmd[0] = 0x01; 
        torque_cmd[1] = 0x08; 
        // --- 大端序转换 (32-bit Signed) ---
        uint32_t uval = static_cast<uint32_t>(torque_param);
        torque_cmd[2] = (uval >> 24) & 0xFF;
        torque_cmd[3] = (uval >> 16) & 0xFF;
        torque_cmd[4] = (uval >> 8)  & 0xFF;
        torque_cmd[5] = (uval)       & 0xFF;
        torque_cmd[6] = 0x00;
        torque_cmd[7] = 0x00;
        if (!sendFrame(motor_id, torque_cmd, 8)) return false;
        if (!receiveFrame(rx, motor_id)) return false;
        // 校验回复 (02 08 01 表示成功)
        if (rx.can_id != motor_id || 
            rx.data[0] != 0x02 || 
            rx.data[1] != 0x08 || 
            rx.data[2] != 0x01) {
            return false;
        }

        return true;
    }


    /* =========================================================
    * 设置当前位置为零点
    * ========================================================= */
    bool zeroCalibrate(uint8_t motor_id)
    {
        if (!openSocket())
            return false;

        struct can_frame rx {};

        /* =====================================================
        * 一、清除偏移值
        * ===================================================== */

        /* Step 1-1: 01 3B 00 00 00 00 */
        uint8_t clear1[8] = {0x01, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrame(motor_id, clear1, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x02 ||
            rx.data[1] != 0x3B ||
            rx.data[2] != 0x01)
            return false;

        /* Step 1-2: 03 3B 00 00 00 00 */
        uint8_t clear2[8] = {0x03, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrame(motor_id, clear2, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x04 ||
            rx.data[1] != 0x3B)
            return false;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /* =====================================================
        * 二、偏移至（核心逻辑：计算零点偏移）
        * ===================================================== */

        /* Step 2-1: 再次准备写偏移 */
        if (!sendFrame(motor_id, clear1, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x02 ||
            rx.data[1] != 0x3B ||
            rx.data[2] != 0x01)
            return false;

        /* Step 2-2: 读取当前位置 03 07 */
        uint8_t read_pos[8] = {0x03, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        if (!sendFrame(motor_id, read_pos, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x04 ||
            rx.data[1] != 0x07)
            return false;

        /*
        * 解析返回的 32 位位置值（多圈 + 单圈）
        */
        uint32_t raw_pos =
            (uint32_t(rx.data[2]) << 24) |
            (uint32_t(rx.data[3]) << 16) |
            (uint32_t(rx.data[4]) << 8)  |
            uint32_t(rx.data[5]);

        /*
        * 关键点：
        * - 驱动器零点校准只关心“单圈相位”
        * - 360° = 65536 脉冲 = 16 bit
        */
        uint16_t single_turn_pos = raw_pos & 0xFFFF;

        /*
        * 核心公式：
        * (当前位置 + 偏移) mod 65536 = 0
        */
        uint16_t offset_16 = (65536 - single_turn_pos) & 0xFFFF;

        /* Step 2-3: 写入偏移值 01 3B */
        uint8_t write_offset[8] = {
            0x01, 0x3B,
            0x00, 0x00,
            uint8_t(offset_16 >> 8),
            uint8_t(offset_16),
            0x00, 0x00
        };

        if (!sendFrame(motor_id, write_offset, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x02 ||
            rx.data[1] != 0x3B ||
            rx.data[2] != 0x01)
            return false;

        /* Step 2-4: 读回确认 03 3B */
        if (!sendFrame(motor_id, clear2, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x04 ||
            rx.data[1] != 0x3B)
            return false;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        /* =====================================================
        * 三、保存参数至控制器（掉电不丢）
        * ===================================================== */

        uint8_t save_cmd[8] = {0x01, 0x4D, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
        if (!sendFrame(motor_id, save_cmd, 8) ||
            !receiveFrame(rx, motor_id) ||
            rx.data[0] != 0x02 ||
            rx.data[1] != 0x4B ||
            rx.data[2] != 0x01)
            return false;

        return true;
    }

    
    /* =========================================================
    * 多电机轮廓位置控制（Profile Position Mode）- 两阶段同步优化版
    *
    * 优化策略：
    * 1. 第一阶段：批量配置运动参数（模式、速度、力矩、加减速），此时不下发目标位置，电机保持静止。
    * 2. 第二阶段：批量下发目标位置，触发所有电机几乎同时开始运动。
    *
    * 执行顺序：
    * Phase 1 (对所有电机):
    *   1. 设置工作模式
    *   2. 设置目标速度
    *   3. 设置目标力矩限制
    *   4. 设置目标加速度
    *   5. 设置目标减速度
    *   (跳过位置设置)
    *
    * Phase 2 (对所有电机):
    *   6. 设置目标位置 (触发运动)
    *
    * 注意：所有输入数组的大小必须一致。
    * ========================================================= */
    bool setProfilePositionMulti(
        const std::vector<uint8_t>& motor_ids,      // 电机ID列表
        const std::vector<double>& target_rads,     // 目标位置列表 (rad)
        const std::vector<double>& speed_rad_s_list,// 目标速度列表 (rad/s)
        const std::vector<float>& target_torques_nm,// 目标力矩限制列表 (N·m)
        const std::vector<double>& accel_rad_s2_list,// 目标加速度列表 (rad/s²)
        const std::vector<double>& decel_rad_s2_list)// 目标减速度列表 (rad/s²)
    {
        // 1. 基础校验
        size_t count = motor_ids.size();
        if (count == 0) {
            std::cerr << "[MotorCAN] Error: Motor ID list is empty.\n";
            return false;
        }
        if (target_rads.size() != count || speed_rad_s_list.size() != count ||
            target_torques_nm.size() != count || accel_rad_s2_list.size() != count ||
            decel_rad_s2_list.size() != count) {
            std::cerr << "[MotorCAN] Error: Input vector sizes mismatch.\n";
            return false;
        }

        if (!openSocket())
            return false;

        struct can_frame rx {};

        /* --- 辅助宏：安全的大端序转换 (32-bit Signed) --- */
        #define TO_BIG_ENDIAN_BYTES(val, arr, offset) \
            do { \
                uint32_t uval = static_cast<uint32_t>(val); \
                arr[offset]   = (uval >> 24) & 0xFF; \
                arr[offset+1] = (uval >> 16) & 0xFF; \
                arr[offset+2] = (uval >> 8)  & 0xFF; \
                arr[offset+3] = (uval)       & 0xFF; \
            } while(0)

        /* --- 辅助函数：获取额定扭矩 --- */
        auto getRatedTorque = [](uint8_t id) -> float {
            switch (id) {
                case 2: case 3: return 5.4f;
                case 4: case 5: return 5.4f;
                case 6: case 7: return 2.4f; 
                default:          return 0.0f;
            }
        };

        // ==========================================
        // PHASE 1: 配置运动参数 (不触发运动)
        // ==========================================
        // 此阶段所有电机接收完参数后，处于"就绪但未动"状态
        for (size_t i = 0; i < count; ++i) {
            uint8_t motor_id = motor_ids[i];
            
            // --- 步骤 1: 工作模式 Profile Position (0x01) ---
            uint8_t mode_cmd[8] = {0x01, 0x0F, 0, 0, 0, 0x01, 0, 0}; 
            if (!sendFrame(motor_id, mode_cmd, 8)) {
                std::cerr << "[MotorCAN] Phase 1 Failed to send mode cmd to motor " << (int)motor_id << "\n";
                return false;
            }
            if (!receiveFrame(rx, motor_id) || rx.data[0]!=0x02 || rx.data[1]!=0x0F || rx.data[2]!=0x01) {
                std::cerr << "[MotorCAN] Phase 1 Mode cmd response error on motor " << (int)motor_id << "\n";
                return false;
            }

            // --- 步骤 2: 目标速度 ---
            double speed_rpm = speed_rad_s_list[i] * (30.0 / M_PI);
            int32_t speed_value = static_cast<int32_t>(std::round(speed_rpm * 1092.0));
            
            uint8_t speed_cmd[8] = {0x01, 0x09, 0, 0, 0, 0, 0, 0};
            TO_BIG_ENDIAN_BYTES(speed_value, speed_cmd, 2);

            if (!sendFrame(motor_id, speed_cmd, 8) ||
                !receiveFrame(rx, motor_id) ||
                rx.data[0]!=0x02 || rx.data[1]!=0x09 || rx.data[2]!=0x01) {
                std::cerr << "[MotorCAN] Phase 1 Speed cmd error on motor " << (int)motor_id << "\n";
                return false;
            }

            // --- 步骤 3: 力矩限制 ---
            float rated_torque_nm = getRatedTorque(motor_id);
            if (rated_torque_nm <= 0.0f) {
                std::cerr << "[MotorCAN] Invalid motor ID " << (int)motor_id << " for torque calculation.\n";
                return false;
            }

            float ratio = target_torques_nm[i] / rated_torque_nm;
            int32_t torque_param = static_cast<int32_t>(std::round(ratio * 1000.0f));

            uint8_t torque_cmd[8] = {0x01, 0x08, 0, 0, 0, 0, 0, 0};
            TO_BIG_ENDIAN_BYTES(torque_param, torque_cmd, 2);

            if (!sendFrame(motor_id, torque_cmd, 8) ||
                !receiveFrame(rx, motor_id) ||
                rx.data[0]!=0x02 || rx.data[1]!=0x08 || rx.data[2]!=0x01) {
                std::cerr << "[MotorCAN] Phase 1 Torque cmd error on motor " << (int)motor_id << "\n";
                return false;
            }

            // --- 步骤 4: 加速度 ---
            double accel_rpm_s = accel_rad_s2_list[i] * (30.0 / M_PI);
            int32_t accel_value = static_cast<int32_t>(std::round(accel_rpm_s * 1092.0));
            
            uint8_t accel_cmd[8] = {0x01, 0x0B, 0, 0, 0, 0, 0, 0};
            TO_BIG_ENDIAN_BYTES(accel_value, accel_cmd, 2);

            if (!sendFrame(motor_id, accel_cmd, 8) ||
                !receiveFrame(rx, motor_id) ||
                rx.data[0]!=0x02 || rx.data[1]!=0x0B || rx.data[2]!=0x01) {
                std::cerr << "[MotorCAN] Phase 1 Accel cmd error on motor " << (int)motor_id << "\n";
                return false;
            }

            // --- 步骤 5: 减速度 ---
            double decel_rpm_s = decel_rad_s2_list[i] * (30.0 / M_PI);
            int32_t decel_value = static_cast<int32_t>(std::round(decel_rpm_s * 1092.0));
            
            uint8_t decel_cmd[8] = {0x01, 0x0C, 0, 0, 0, 0, 0, 0};
            TO_BIG_ENDIAN_BYTES(decel_value, decel_cmd, 2);

            if (!sendFrame(motor_id, decel_cmd, 8) ||
                !receiveFrame(rx, motor_id) ||
                rx.data[0]!=0x02 || rx.data[1]!=0x0C || rx.data[2]!=0x01) {
                std::cerr << "[MotorCAN] Phase 1 Decel cmd error on motor " << (int)motor_id << "\n";
                return false;
            }
            
            // 【注意】此处故意跳过步骤6 (目标位置)，等待Phase 2统一触发
        }

        // ==========================================
        // PHASE 2: 统一下发目标位置 (触发运动)
        // ==========================================
        // 此阶段尽可能快地将所有电机的目标位置下发
        // 时间差仅取决于最后几个电机的通信延迟
        for (size_t i = 0; i < count; ++i) {
            uint8_t motor_id = motor_ids[i];

            // --- 步骤 6: 目标位置 (核心触发点) ---
            double target_deg = target_rads[i] * (180.0 / M_PI);
            int32_t pos_pulse = static_cast<int32_t>(std::round(target_deg / 360.0 * 65536.0));
            
            uint8_t pos_cmd[8] = {0x01, 0x0A, 0, 0, 0, 0, 0, 0};
            TO_BIG_ENDIAN_BYTES(pos_pulse, pos_cmd, 2);

            if (!sendFrame(motor_id, pos_cmd, 8) ||
                !receiveFrame(rx, motor_id) ||
                rx.data[0]!=0x02 || rx.data[1]!=0x0A || rx.data[2]!=0x01) {
                std::cerr << "[MotorCAN] Phase 2 Pos cmd error on motor " << (int)motor_id << "\n";
                // 注意：如果这里失败，前面的电机可能已经动了，后面的没动。
                // 根据业务需求，这里可以选择立即停止所有电机或返回false
                return false;
            }
        }

        return true;
    }

private:
    std::string can_name_;
    int socket_fd_;
};

#endif // MOTOR_CAN_H