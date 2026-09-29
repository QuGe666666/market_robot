
/*
 * giantcrab_joint_node.cpp
 *
 * 说明：
 * 1. 本文件实现一个尽量简单、但比“临时验证脚本”更稳的 ROS2 单轴关节节点。
 * 2. 底层通信仍然基于创芯 / ZLG 兼容的 ControlCANFD SDK。
 * 3. 整体设计遵循你已经验证过的最小可用流程：
 *      dlopen -> OpenDevice -> SetBaud -> InitCAN -> StartCAN -> SDO读写
 * 4. 这里故意不调用 ZCAN_SetResistanceEnable，避免你前面遇到的段错误点。
 *
 * 使用前提：
 * - 系统可以找到 controlcanfd.h / config.h
 * - 运行时可以找到 libcontrolcanfd.so（环境变量 CONTROLCANFD_SO 或包 lib 目录）
 */

#include <ament_index_cpp/get_package_prefix.hpp>


#ifndef CANFD_BRS
#define CANFD_BRS 0x01
#endif
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "controlcanfd.h"
#include "config.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "giantcrab_joint_driver/srv/get_angle.hpp"
#include "giantcrab_joint_driver/srv/get_status.hpp"
#include "giantcrab_joint_driver/srv/set_angle.hpp"
#include "giantcrab_joint_driver/srv/set_limits.hpp"
#include "giantcrab_joint_driver/srv/set_speed.hpp"

using namespace std::chrono_literals;

#ifndef TYPE_CANFD
#define TYPE_CANFD 1
#endif

namespace giantcrab_joint_driver
{

/* ============================ 工具函数区 ============================ */

static std::string bytes_to_hex(const uint8_t *data, size_t len)
{
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');
  for (size_t i = 0; i < len; ++i) {
    oss << std::setw(2) << static_cast<unsigned>(data[i]);
    if (i + 1 < len) {
      oss << " ";
    }
  }
  return oss.str();
}

static int32_t le_to_i32(const uint8_t *p)
{
  return static_cast<int32_t>(
    (static_cast<uint32_t>(p[0])) |
    (static_cast<uint32_t>(p[1]) << 8) |
    (static_cast<uint32_t>(p[2]) << 16) |
    (static_cast<uint32_t>(p[3]) << 24));
}

static uint16_t le_to_u16(const uint8_t *p)
{
  return static_cast<uint16_t>(
    (static_cast<uint16_t>(p[0])) |
    (static_cast<uint16_t>(p[1]) << 8));
}

static void u16_to_le(uint16_t value, uint8_t *p)
{
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static void u32_to_le(uint32_t value, uint8_t *p)
{
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

static uint32_t i32_to_u32_bitwise(int32_t value)
{
  uint32_t out = 0;
  std::memcpy(&out, &value, sizeof(out));
  return out;
}

/* ============================ 动态库函数指针定义 ============================ */

using pZCAN_OpenDevice = DEVICE_HANDLE (*)(UINT, UINT, UINT);
using pZCAN_CloseDevice = UINT (*)(DEVICE_HANDLE);
using pZCAN_InitCAN = CHANNEL_HANDLE (*)(DEVICE_HANDLE, UINT, ZCAN_CHANNEL_INIT_CONFIG *);
using pZCAN_StartCAN = UINT (*)(CHANNEL_HANDLE);
using pZCAN_ResetCAN = UINT (*)(CHANNEL_HANDLE);
using pZCAN_ClearBuffer = UINT (*)(CHANNEL_HANDLE);
using pZCAN_GetReceiveNum = UINT (*)(CHANNEL_HANDLE, BYTE);
using pZCAN_TransmitFD = UINT (*)(CHANNEL_HANDLE, ZCAN_TransmitFD_Data *, UINT);
using pZCAN_ReceiveFD = UINT (*)(CHANNEL_HANDLE, ZCAN_ReceiveFD_Data *, UINT, int);
using pZCAN_SetAbitBaud = UINT (*)(DEVICE_HANDLE, UINT, UINT);
using pZCAN_SetDbitBaud = UINT (*)(DEVICE_HANDLE, UINT, UINT);
using pZCAN_SetCANFDStandard = UINT (*)(DEVICE_HANDLE, UINT, UINT);

/*
 * SdkApi:
 * 只负责：
 * - dlopen 动态加载 so
 * - dlsym 获取所需函数
 *
 * 这样做的好处：
 * - 可以沿用你当前已经跑通的“动态加载 so”方式
 * - 避免程序在启动时因为链接不到 so 直接无法运行
 */
class SdkApi
{
public:
  ~SdkApi()
  {
    unload();
  }

  bool load(const std::string &so_path, std::string &err)
  {
    unload();

    handle_ = dlopen(so_path.c_str(), RTLD_LAZY);
    if (!handle_) {
      err = std::string("dlopen 失败: ") + dlerror();
      return false;
    }

    open_device = reinterpret_cast<pZCAN_OpenDevice>(dlsym(handle_, "ZCAN_OpenDevice"));
    close_device = reinterpret_cast<pZCAN_CloseDevice>(dlsym(handle_, "ZCAN_CloseDevice"));
    init_can = reinterpret_cast<pZCAN_InitCAN>(dlsym(handle_, "ZCAN_InitCAN"));
    start_can = reinterpret_cast<pZCAN_StartCAN>(dlsym(handle_, "ZCAN_StartCAN"));
    reset_can = reinterpret_cast<pZCAN_ResetCAN>(dlsym(handle_, "ZCAN_ResetCAN"));
    clear_buffer = reinterpret_cast<pZCAN_ClearBuffer>(dlsym(handle_, "ZCAN_ClearBuffer"));
    get_receive_num = reinterpret_cast<pZCAN_GetReceiveNum>(dlsym(handle_, "ZCAN_GetReceiveNum"));
    transmit_fd = reinterpret_cast<pZCAN_TransmitFD>(dlsym(handle_, "ZCAN_TransmitFD"));
    receive_fd = reinterpret_cast<pZCAN_ReceiveFD>(dlsym(handle_, "ZCAN_ReceiveFD"));
    set_abit_baud = reinterpret_cast<pZCAN_SetAbitBaud>(dlsym(handle_, "ZCAN_SetAbitBaud"));
    set_dbit_baud = reinterpret_cast<pZCAN_SetDbitBaud>(dlsym(handle_, "ZCAN_SetDbitBaud"));
    set_canfd_standard = reinterpret_cast<pZCAN_SetCANFDStandard>(dlsym(handle_, "ZCAN_SetCANFDStandard"));

    if (!open_device || !close_device || !init_can || !start_can || !reset_can ||
      !clear_buffer || !get_receive_num || !transmit_fd || !receive_fd ||
      !set_abit_baud || !set_dbit_baud || !set_canfd_standard)
    {
      err = "dlsym 失败：所需 ControlCANFD 符号不完整";
      unload();
      return false;
    }

    return true;
  }

  void unload()
  {
    if (handle_) {
      dlclose(handle_);
      handle_ = nullptr;
    }
  }

  pZCAN_OpenDevice open_device = nullptr;
  pZCAN_CloseDevice close_device = nullptr;
  pZCAN_InitCAN init_can = nullptr;
  pZCAN_StartCAN start_can = nullptr;
  pZCAN_ResetCAN reset_can = nullptr;
  pZCAN_ClearBuffer clear_buffer = nullptr;
  pZCAN_GetReceiveNum get_receive_num = nullptr;
  pZCAN_TransmitFD transmit_fd = nullptr;
  pZCAN_ReceiveFD receive_fd = nullptr;
  pZCAN_SetAbitBaud set_abit_baud = nullptr;
  pZCAN_SetDbitBaud set_dbit_baud = nullptr;
  pZCAN_SetCANFDStandard set_canfd_standard = nullptr;

private:
  void *handle_ = nullptr;
};

/*
 * DriverParams:
 * 集中管理节点参数，避免到处散落。
 */
struct DriverParams
{
  uint32_t device_type = 41;
  uint32_t device_index = 0;
  uint32_t channel_index = 0;
  uint32_t node_id = 1;
  uint32_t arb_bitrate = 1000000;
  uint32_t data_bitrate = 5000000;
  bool enable_brs = true;
  bool canfd_iso = true;
  std::string sdk_so_path;

  std::string joint_name = "pitch_joint";
  double min_angle_deg = 0.0;
  double max_angle_deg = 50.0;
  double speed_rpm = 0.5;
  double accel_rpm_per_s = 1000.0;
  double decel_rpm_per_s = 1000.0;
  double counts_per_turn = 65536.0;
  bool auto_enable_before_move = true;
  bool wait_boot_ready = true;
  int boot_ready_timeout_ms = 3500;
  int settle_after_enable_ms = 100;
  bool enable_verbose_log = true;
  int publish_period_ms = 100;

  uint16_t od_controlword_index = 0x6040;
  uint16_t od_statusword_index = 0x6041;
  uint16_t od_fault_code_index = 0x603F;
  uint16_t od_mode_of_operation_index = 0x6060;
  uint16_t od_actual_position_index = 0x6064;
  uint16_t od_actual_velocity_index = 0x606C;
  uint16_t od_target_position_index = 0x607A;
  uint16_t od_profile_velocity_index = 0x6081;
  uint16_t od_profile_accel_index = 0x6083;
  uint16_t od_profile_decel_index = 0x6084;
  uint16_t od_set_zero_index = 0x2531;
};

/*
 * GiantCrabCanDriver:
 * 真正负责与巨蟹关节模组通信。
 *
 * 设计原则：
 * - 尽量简单，遵循你已验证的最短通信路径
 * - 但修掉老程序中最危险的坑：
 *   1. 加互斥锁，避免并发 service 把收发搞乱
 *   2. 每次事务严格匹配响应 ID / index / subindex
 *   3. 位置值按 32 位处理，不再只取 16 位
 *   4. 负数目标位置按标准补码写入
 */
class GiantCrabCanDriver
{
public:
  explicit GiantCrabCanDriver(rclcpp::Logger logger)
  : logger_(logger)
  {
  }

  bool init(const DriverParams &params, const std::string &package_name, std::string &msg)
  {
    params_ = params;
    set_limits(params_.min_angle_deg, params_.max_angle_deg, msg);

    std::string so_path = params.sdk_so_path;
    if (so_path.empty()) {
      const char *env_so = std::getenv("CONTROLCANFD_SO");
      if (env_so && std::strlen(env_so) > 0) {
        so_path = env_so;
      }
    }
    if (so_path.empty()) {
      try {
        const auto prefix = ament_index_cpp::get_package_prefix(package_name);
        so_path = (std::filesystem::path(prefix) / "lib" / "libcontrolcanfd.so").string();
      } catch (const std::exception &e) {
        msg = std::string("无法获取包安装前缀: ") + e.what();
        return false;
      }
    }

    std::string err;
    if (!sdk_.load(so_path, err)) {
      msg = "加载 libcontrolcanfd.so 失败: " + err + "，尝试路径: " + so_path;
      return false;
    }

    device_ = sdk_.open_device(params_.device_type, params_.device_index, 0);
    if (device_ == INVALID_DEVICE_HANDLE) {
      msg = "ZCAN_OpenDevice 失败";
      return false;
    }

    if (sdk_.set_abit_baud(device_, params_.channel_index, params_.arb_bitrate) != STATUS_OK) {
      msg = "ZCAN_SetAbitBaud 失败";
      return false;
    }
    if (sdk_.set_dbit_baud(device_, params_.channel_index, params_.data_bitrate) != STATUS_OK) {
      msg = "ZCAN_SetDbitBaud 失败";
      return false;
    }

    if (sdk_.set_canfd_standard(device_, params_.channel_index, params_.canfd_iso ? 0U : 1U) != STATUS_OK) {
      RCLCPP_WARN(logger_, "ZCAN_SetCANFDStandard 失败，继续尝试后续流程。");
    }

    ZCAN_CHANNEL_INIT_CONFIG cfg {};
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.can_type = 1;  // 1 = CANFD

    /*
     * 注意：
     * 不同厂商 / 不同头文件版本，这里的字段布局可能略有差异。
     * 你当前已经跑通的老程序使用的是：
     *   cfg.canfd.acc_code / acc_mask / filter / mode / brp
     * 所以这里沿用同一风格。
     */
    cfg.canfd.acc_code = 0;
    cfg.canfd.acc_mask = 0xFFFFFFFF;
    cfg.canfd.filter = 1;
    cfg.canfd.mode = 0;
    cfg.canfd.brp = 0;

    channel_ = sdk_.init_can(device_, params_.channel_index, &cfg);
    if (channel_ == INVALID_CHANNEL_HANDLE) {
      msg = "ZCAN_InitCAN 失败";
      return false;
    }

    if (sdk_.start_can(channel_) != STATUS_OK) {
      msg = "ZCAN_StartCAN 失败";
      return false;
    }

    sdk_.clear_buffer(channel_);

    if (params_.wait_boot_ready) {
      wait_boot_ready(params_.boot_ready_timeout_ms);
    }

    msg = "CANFD 通道已启动";
    return true;
  }

  void shutdown()
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (channel_ != INVALID_CHANNEL_HANDLE && sdk_.reset_can) {
      sdk_.reset_can(channel_);
      channel_ = INVALID_CHANNEL_HANDLE;
    }
    if (device_ != INVALID_DEVICE_HANDLE && sdk_.close_device) {
      sdk_.close_device(device_);
      device_ = INVALID_DEVICE_HANDLE;
    }
  }

  bool clear_fault(std::string &msg)
  {
    // 正确做法：向控制字 0x6040 写 0x0080
    if (!write_u16(params_.od_controlword_index, 0x00, 0x0080, msg)) {
      return false;
    }
    msg = "已发送清故障控制字 0x0080";
    return true;
  }

bool set_enable(bool enable, std::string &msg)
{
  if (enable) {
    if (!write_u16(params_.od_controlword_index, 0x00, 0x0006, msg)) {
      enabled_ = false;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!write_u16(params_.od_controlword_index, 0x00, 0x000F, msg)) {
      enabled_ = false;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(params_.settle_after_enable_ms));

    uint16_t sw = 0;
    std::string sw_msg;
    if (get_status_word(sw, sw_msg)) {
      // 常见 CiA402: 0x0027 表示 operation enabled
      enabled_ = ((sw & 0x006F) == 0x0027) || ((sw & 0x0027) == 0x0027);
      if (!enabled_) {
        msg = "控制字已发送，但状态字未确认进入使能态，status_word=0x" + [&]() {
          std::ostringstream oss;
          oss << std::hex << std::uppercase << sw;
          return oss.str();
        }();
        return false;
      }
      msg = "关节已使能";
      return true;
    }

    enabled_ = false;
    msg = "控制字已发送，但读取状态字失败: " + sw_msg;
    return false;
  }

  if (!write_u16(params_.od_controlword_index, 0x00, 0x0000, msg)) {
    return false;
  }

  enabled_ = false;
  msg = "关节已失能";
  return true;
}

  bool set_zero(std::string &msg)
  {
    // 设零位前先失能，更符合设备协议说明中的推荐动作。
    std::string tmp;
    (void)set_enable(false, tmp);

    if (!write_u32(params_.od_set_zero_index, 0x00, 1U, msg)) {
      return false;
    }
    msg = "已发送设零位指令";
    return true;
  }

  bool set_speed(double speed_rpm, std::string &msg)
  {
    if (speed_rpm <= 0.0) {
      msg = "速度必须大于 0";
      return false;
    }
    speed_rpm_ = speed_rpm;

    if (!write_u32(params_.od_profile_velocity_index, 0x00, static_cast<uint32_t>(std::llround(speed_rpm_)), msg)) {
      return false;
    }
    msg = "已更新轮廓速度";
    return true;
  }

  bool set_limits(double min_angle_deg, double max_angle_deg, std::string &msg)
  {
    const double motor_min_angle_deg = from_display_angle(max_angle_deg);
    const double motor_max_angle_deg = from_display_angle(min_angle_deg);

    if (motor_min_angle_deg >= motor_max_angle_deg) {
      msg = "下限必须小于上限";
      return false;
    }
    min_angle_deg_ = motor_min_angle_deg;
    max_angle_deg_ = motor_max_angle_deg;
    msg = "软件限位已更新";
    return true;
  }

  bool get_angle(double &angle_deg, std::string &msg)
  {
    int32_t raw = 0;
    if (!read_i32(params_.od_actual_position_index, 0x00, raw, msg)) {
      return false;
    }
    angle_deg = to_display_angle(raw_to_angle(raw));
    msg = "读取角度成功";
    return true;
  }

  bool get_velocity(double &velocity_rpm, std::string &msg)
  {
    int32_t raw = 0;
    if (!read_i32(params_.od_actual_velocity_index, 0x00, raw, msg)) {
      return false;
    }
    velocity_rpm = static_cast<double>(raw);
    msg = "读取速度成功";
    return true;
  }

  bool get_status_word(uint16_t &status_word, std::string &msg)
  {
    uint16_t raw = 0;
    if (!read_u16(params_.od_statusword_index, 0x00, raw, msg)) {
      return false;
    }
    status_word = raw;
    msg = "读取状态字成功";
    return true;
  }

  bool get_fault_code(uint16_t &fault_code, std::string &msg)
  {
    uint16_t raw = 0;
    if (!read_u16(params_.od_fault_code_index, 0x00, raw, msg)) {
      return false;
    }
    fault_code = raw;
    msg = "读取故障码成功";
    return true;
  }

  bool set_angle(double angle_deg, std::string &msg)
  {
    const double motor_angle_deg = from_display_angle(angle_deg);
    const double clamped = std::min(std::max(motor_angle_deg, min_angle_deg_), max_angle_deg_);

    if (params_.auto_enable_before_move && !enabled_) {
      std::string enable_msg;
      if (!set_enable(true, enable_msg)) {
        msg = "自动使能失败: " + enable_msg;
        return false;
      }
    }

    // 1) 进入轮廓位置模式
    if (!write_u8(params_.od_mode_of_operation_index, 0x00, 0x01, msg)) {
      return false;
    }

    // 2) 写轮廓速度/加速度/减速度
    if (!write_u32(params_.od_profile_velocity_index, 0x00,
      static_cast<uint32_t>(std::llround(speed_rpm_)), msg))
    {
      return false;
    }
    if (!write_u32(params_.od_profile_accel_index, 0x00,
      static_cast<uint32_t>(std::llround(accel_rpm_per_s_)), msg))
    {
      return false;
    }
    if (!write_u32(params_.od_profile_decel_index, 0x00,
      static_cast<uint32_t>(std::llround(decel_rpm_per_s_)), msg))
    {
      return false;
    }

    // 3) 写目标位置
    const int32_t raw_target = angle_to_raw(clamped);
    if (!write_i32(params_.od_target_position_index, 0x00, raw_target, msg)) {
      return false;
    }

    // 4) 写控制字 0x004F 触发绝对运动
    if (!write_u16(params_.od_controlword_index, 0x00, 0x004F, msg)) {
      return false;
    }

    msg = "已发送目标角度";
    return true;
  }

  double min_limit() const {return min_angle_deg_;}
  double max_limit() const {return max_angle_deg_;}
  double min_display_limit() const
  {
    return std::min(to_display_angle(min_angle_deg_), to_display_angle(max_angle_deg_));
  }
  double max_display_limit() const
  {
    return std::max(to_display_angle(min_angle_deg_), to_display_angle(max_angle_deg_));
  }
  double speed_rpm() const {return speed_rpm_;}
  bool enabled() const {return enabled_;}

private:
  bool wait_boot_ready(int timeout_ms)
  {
    auto start = std::chrono::steady_clock::now();

    while (true) {
      ZCAN_ReceiveFD_Data rx[256] {};
      const UINT count = sdk_.receive_fd(channel_, rx, 256, 20);

      for (UINT i = 0; i < count; ++i) {
        const auto &f = rx[i].frame;
        const uint32_t id = GET_ID(f.can_id);
        // if (params_.enable_verbose_log) {
        //   RCLCPP_INFO(logger_, "[BOOT-RX] id=0x%X len=%u data=[%s]",
        //     id, static_cast<unsigned>(f.len), bytes_to_hex(f.data, f.len).c_str());
        // }
        if (id == (0x700U + params_.node_id) && f.len >= 1 && f.data[0] == 0x00) {
          RCLCPP_INFO(logger_, "检测到 0x700 + node_id 上电反馈，驱动器 App 已就绪。");
          return true;
        }
      }

      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
      if (elapsed >= timeout_ms) {
        RCLCPP_WARN(logger_, "在 %d ms 内未检测到上电反馈，继续后续流程。", timeout_ms);
        return false;
      }
    }
  }

  /*
   * transact_sdo:
   * 一次完整 SDO 事务：
   * - 先清空缓冲区
   * - 发送 1 帧 8 字节 CANFD 标准帧
   * - 等待 0x580 + node_id 的响应
   * - 严格匹配 index / subindex
   */
  bool transact_sdo(
    const uint8_t req[8],
    uint8_t resp[8],
    uint16_t expect_index,
    uint8_t expect_subindex,
    std::string &msg,
    int timeout_ms = 500)
  {
    std::lock_guard<std::mutex> lock(io_mutex_);

    sdk_.clear_buffer(channel_);

    ZCAN_TransmitFD_Data tx {};
    tx.transmit_type = 0;
    
    tx.frame.len = 8;
    
    tx.frame.can_id = MAKE_CAN_ID((0x600U + params_.node_id), 0, 0, 0);
    tx.frame.flags = params_.enable_brs ? CANFD_BRS : 0;
    std::memcpy(tx.frame.data, req, 8);

    const UINT sent = sdk_.transmit_fd(channel_, &tx, 1);
    if (sent != 1) {
      msg = "ZCAN_TransmitFD 发送失败";
      return false;
    }

    // if (params_.enable_verbose_log) {
    //   RCLCPP_INFO(logger_, "[TX] id=0x%X len=%u data=[%s]",
    //     GET_ID(tx.frame.can_id),
    //     static_cast<unsigned>(tx.frame.len),
    //     bytes_to_hex(tx.frame.data, tx.frame.len).c_str());
    // }

    auto start = std::chrono::steady_clock::now();
    while (true) {
      ZCAN_ReceiveFD_Data rx[256] {};
      const UINT count = sdk_.receive_fd(channel_, rx, 256, 20);

      for (UINT i = 0; i < count; ++i) {
        const auto &f = rx[i].frame;
        const uint32_t id = GET_ID(f.can_id);

        // if (params_.enable_verbose_log) {
        //   RCLCPP_INFO(logger_, "[RX] id=0x%X len=%u data=[%s]",
        //     id, static_cast<unsigned>(f.len), bytes_to_hex(f.data, f.len).c_str());
        // }

        if (id != (0x580U + params_.node_id)) {
          continue;
        }
        if (f.len < 8) {
          continue;
        }
        if (f.data[1] != static_cast<uint8_t>(expect_index & 0xFF) ||
          f.data[2] != static_cast<uint8_t>((expect_index >> 8) & 0xFF) ||
          f.data[3] != expect_subindex)
        {
          // 不是本次事务对应的响应，跳过
          continue;
        }

        std::memcpy(resp, f.data, 8);
        return true;
      }

      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
      if (elapsed >= timeout_ms) {
        msg = "等待 SDO 响应超时";
        return false;
      }
    }
  }

  bool write_u8(uint16_t index, uint8_t subindex, uint8_t value, std::string &msg)
  {
    uint8_t req[8] = {0x2F, 0, 0, subindex, value, 0, 0, 0};
    req[1] = static_cast<uint8_t>(index & 0xFF);
    req[2] = static_cast<uint8_t>((index >> 8) & 0xFF);

    uint8_t resp[8] {};
    if (!transact_sdo(req, resp, index, subindex, msg)) {
      return false;
    }
    return true;
  }

  bool write_u16(uint16_t index, uint8_t subindex, uint16_t value, std::string &msg)
  {
    uint8_t req[8] = {0x2B, 0, 0, subindex, 0, 0, 0, 0};
    req[1] = static_cast<uint8_t>(index & 0xFF);
    req[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    u16_to_le(value, &req[4]);

    uint8_t resp[8] {};
    if (!transact_sdo(req, resp, index, subindex, msg)) {
      return false;
    }
    return true;
  }

  bool write_u32(uint16_t index, uint8_t subindex, uint32_t value, std::string &msg)
  {
    uint8_t req[8] = {0x23, 0, 0, subindex, 0, 0, 0, 0};
    req[1] = static_cast<uint8_t>(index & 0xFF);
    req[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    u32_to_le(value, &req[4]);

    uint8_t resp[8] {};
    if (!transact_sdo(req, resp, index, subindex, msg)) {
      return false;
    }
    return true;
  }

  bool write_i32(uint16_t index, uint8_t subindex, int32_t value, std::string &msg)
  {
    return write_u32(index, subindex, i32_to_u32_bitwise(value), msg);
  }

  bool read_i32(uint16_t index, uint8_t subindex, int32_t &value, std::string &msg)
  {
    uint8_t req[8] = {0x40, 0, 0, subindex, 0, 0, 0, 0};
    req[1] = static_cast<uint8_t>(index & 0xFF);
    req[2] = static_cast<uint8_t>((index >> 8) & 0xFF);

    uint8_t resp[8] {};
    if (!transact_sdo(req, resp, index, subindex, msg)) {
      return false;
    }

    value = le_to_i32(&resp[4]);
    return true;
  }

  bool read_u16(uint16_t index, uint8_t subindex, uint16_t &value, std::string &msg)
  {
    uint8_t req[8] = {0x40, 0, 0, subindex, 0, 0, 0, 0};
    req[1] = static_cast<uint8_t>(index & 0xFF);
    req[2] = static_cast<uint8_t>((index >> 8) & 0xFF);

    uint8_t resp[8] {};
    if (!transact_sdo(req, resp, index, subindex, msg)) {
      return false;
    }

    value = le_to_u16(&resp[4]);
    return true;
  }

  int32_t angle_to_raw(double angle_deg) const
  {
    const double raw = angle_deg * params_.counts_per_turn / 360.0;
    return static_cast<int32_t>(std::llround(raw));
  }

  double raw_to_angle(int32_t raw) const
  {
    return static_cast<double>(raw) * 360.0 / params_.counts_per_turn;
  }

  double to_display_angle(double angle_deg) const
  {
    // Only flip the reported sign. Motor-space motion stays unchanged.
    return -angle_deg;
  }

  double from_display_angle(double angle_deg) const
  {
    return -angle_deg;
  }

  rclcpp::Logger logger_;
  DriverParams params_;
  SdkApi sdk_;

  DEVICE_HANDLE device_ = INVALID_DEVICE_HANDLE;
  CHANNEL_HANDLE channel_ = INVALID_CHANNEL_HANDLE;
  std::mutex io_mutex_;

  double min_angle_deg_ = -50.0;
  double max_angle_deg_ = 0.0;
  double speed_rpm_ = 0.5;
  double accel_rpm_per_s_ = 1000.0;
  double decel_rpm_per_s_ = 1000.0;
  bool enabled_ = false;
};

/*
 * GiantCrabJointNode:
 * ROS2 节点层。
 *
 * 职责非常直接：
 * - 暴露 services / topics
 * - 周期发布 JointState / 状态字 / 故障码
 * - 底层所有真实动作都委托给 GiantCrabCanDriver
 */
class GiantCrabJointNode : public rclcpp::Node
{
public:
  GiantCrabJointNode()
  : Node("giantcrab_joint_node"), driver_(this->get_logger())
  {
    load_params();

    std::string init_msg;
    if (!driver_.init(params_, "giantcrab_joint_driver", init_msg)) {
      RCLCPP_FATAL(this->get_logger(), "驱动初始化失败：%s", init_msg.c_str());
      throw std::runtime_error(init_msg);
    }
    RCLCPP_INFO(this->get_logger(), "%s", init_msg.c_str());

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint/joint_state", 10);
    status_word_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/joint/status_word", 10);
    fault_code_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/joint/fault_code", 10);

    angle_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/joint/command_angle", 10,
      std::bind(&GiantCrabJointNode::on_angle_command, this, std::placeholders::_1));

    get_angle_srv_ = this->create_service<srv::GetAngle>(
      "/joint/get_angle",
      std::bind(&GiantCrabJointNode::on_get_angle, this, std::placeholders::_1, std::placeholders::_2));

    set_angle_srv_ = this->create_service<srv::SetAngle>(
      "/joint/set_angle",
      std::bind(&GiantCrabJointNode::on_set_angle, this, std::placeholders::_1, std::placeholders::_2));

    set_limits_srv_ = this->create_service<srv::SetLimits>(
      "/joint/set_limits",
      std::bind(&GiantCrabJointNode::on_set_limits, this, std::placeholders::_1, std::placeholders::_2));

    set_speed_srv_ = this->create_service<srv::SetSpeed>(
      "/joint/set_speed",
      std::bind(&GiantCrabJointNode::on_set_speed, this, std::placeholders::_1, std::placeholders::_2));

    get_status_srv_ = this->create_service<srv::GetStatus>(
      "/joint/get_status",
      std::bind(&GiantCrabJointNode::on_get_status, this, std::placeholders::_1, std::placeholders::_2));

    clear_fault_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/joint/clear_fault",
      std::bind(&GiantCrabJointNode::on_clear_fault, this, std::placeholders::_1, std::placeholders::_2));

    set_enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "/joint/set_enable",
      std::bind(&GiantCrabJointNode::on_set_enable, this, std::placeholders::_1, std::placeholders::_2));

    set_zero_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/joint/set_zero",
      std::bind(&GiantCrabJointNode::on_set_zero, this, std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(params_.publish_period_ms),
      std::bind(&GiantCrabJointNode::publish_state, this));
  }

  ~GiantCrabJointNode() override
  {
    driver_.shutdown();
  }

private:
void load_params()
{
  params_.device_type = static_cast<uint32_t>(
    this->declare_parameter<int>("device_type", 41));
  params_.device_index = static_cast<uint32_t>(
    this->declare_parameter<int>("device_index", 0));
  params_.channel_index = static_cast<uint32_t>(
    this->declare_parameter<int>("channel_index", 0));
  params_.node_id = static_cast<uint32_t>(
    this->declare_parameter<int>("node_id", 1));
  params_.arb_bitrate = static_cast<uint32_t>(
    this->declare_parameter<int>("arb_bitrate", 1000000));
  params_.data_bitrate = static_cast<uint32_t>(
    this->declare_parameter<int>("data_bitrate", 5000000));

  params_.enable_brs = this->declare_parameter<bool>("enable_brs", true);
  params_.canfd_iso = this->declare_parameter<bool>("canfd_iso", true);
  params_.sdk_so_path = this->declare_parameter<std::string>("sdk_so_path", "");

  params_.joint_name = this->declare_parameter<std::string>("joint_name", "pitch_joint");
  params_.min_angle_deg = this->declare_parameter<double>("min_angle_deg", 0.0);
  params_.max_angle_deg = this->declare_parameter<double>("max_angle_deg", 50.0);
  params_.speed_rpm = this->declare_parameter<double>("speed_rpm", 0.5);
  params_.accel_rpm_per_s = this->declare_parameter<double>("accel_rpm_per_s", 1000.0);
  params_.decel_rpm_per_s = this->declare_parameter<double>("decel_rpm_per_s", 1000.0);
  params_.counts_per_turn = this->declare_parameter<double>("counts_per_turn", 65536.0);

  // 想禁用“设置角度前自动使能”，这里就保持参数读取，
  // 但你要在 yaml 里配成 false，并重启节点
  params_.auto_enable_before_move = this->declare_parameter<bool>("auto_enable_before_move", true);

  params_.wait_boot_ready = this->declare_parameter<bool>("wait_boot_ready", true);
  params_.boot_ready_timeout_ms = this->declare_parameter<int>("boot_ready_timeout_ms", 3500);
  params_.settle_after_enable_ms = this->declare_parameter<int>("settle_after_enable_ms", 100);
  params_.enable_verbose_log = this->declare_parameter<bool>("enable_verbose_log", true);
  params_.publish_period_ms = this->declare_parameter<int>("publish_period_ms", 100);

  params_.od_controlword_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_controlword_index", 0x6040));
  params_.od_statusword_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_statusword_index", 0x6041));
  params_.od_fault_code_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_fault_code_index", 0x603F));
  params_.od_mode_of_operation_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_mode_of_operation_index", 0x6060));
  params_.od_actual_position_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_actual_position_index", 0x6064));
  params_.od_actual_velocity_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_actual_velocity_index", 0x606C));
  params_.od_target_position_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_target_position_index", 0x607A));
  params_.od_profile_velocity_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_profile_velocity_index", 0x6081));
  params_.od_profile_accel_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_profile_accel_index", 0x6083));
  params_.od_profile_decel_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_profile_decel_index", 0x6084));
  params_.od_set_zero_index = static_cast<uint16_t>(
    this->declare_parameter<int>("od_set_zero_index", 0x2531));
}

  void on_angle_command(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::string info;
    if (!driver_.set_angle(msg->data, info)) {
      RCLCPP_ERROR(this->get_logger(), "话题下发角度失败：%s", info.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "话题下发角度成功：%.3f deg", msg->data);
  }

  void on_get_angle(
    const std::shared_ptr<srv::GetAngle::Request>,
    std::shared_ptr<srv::GetAngle::Response> res)
  {
    std::string msg;
    double angle = 0.0;
    res->success = driver_.get_angle(angle, msg);
    res->angle_deg = angle;
    res->message = msg;
  }

  void on_set_angle(
    const std::shared_ptr<srv::SetAngle::Request> req,
    std::shared_ptr<srv::SetAngle::Response> res)
  {
    std::string msg;
    res->success = driver_.set_angle(req->angle_deg, msg);
    res->message = msg;
  }

  void on_set_limits(
    const std::shared_ptr<srv::SetLimits::Request> req,
    std::shared_ptr<srv::SetLimits::Response> res)
  {
    std::string msg;
    res->success = driver_.set_limits(req->min_angle_deg, req->max_angle_deg, msg);
    res->message = msg;
  }

  void on_set_speed(
    const std::shared_ptr<srv::SetSpeed::Request> req,
    std::shared_ptr<srv::SetSpeed::Response> res)
  {
    std::string msg;
    res->success = driver_.set_speed(req->speed_rpm, msg);
    res->message = msg;
  }

  void on_get_status(
    const std::shared_ptr<srv::GetStatus::Request>,
    std::shared_ptr<srv::GetStatus::Response> res)
  {
    std::string msg;
    double angle = 0.0;
    double vel = 0.0;
    uint16_t sw = 0;
    uint16_t fault = 0;

    bool ok = true;
    ok &= driver_.get_angle(angle, msg);
    ok &= driver_.get_velocity(vel, msg);
    ok &= driver_.get_status_word(sw, msg);
    ok &= driver_.get_fault_code(fault, msg);

    res->success = ok;
    res->angle_deg = angle;
    res->velocity_rpm = vel;
    res->status_word = sw;
    res->fault_code = fault;
    res->min_angle_deg = driver_.min_display_limit();
    res->max_angle_deg = driver_.max_display_limit();
    res->speed_rpm = driver_.speed_rpm();
    res->enabled = driver_.enabled();
    res->message = ok ? "读取状态成功" : msg;
  }

  void on_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::string msg;
    res->success = driver_.clear_fault(msg);
    res->message = msg;
  }

  void on_set_enable(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    std::string msg;
    res->success = driver_.set_enable(req->data, msg);
    res->message = msg;
  }

  void on_set_zero(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::string msg;
    res->success = driver_.set_zero(msg);
    res->message = msg;
  }

  void publish_state()
  {
    std::string msg;
    double angle_deg = 0.0;
    double velocity_rpm = 0.0;
    uint16_t status_word = 0;
    uint16_t fault_code = 0;

    if (!driver_.get_angle(angle_deg, msg)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "读取角度失败：%s", msg.c_str());
      return;
    }
    (void)driver_.get_velocity(velocity_rpm, msg);
    (void)driver_.get_status_word(status_word, msg);
    (void)driver_.get_fault_code(fault_code, msg);

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = {params_.joint_name};
    js.position = {angle_deg * M_PI / 180.0};
    js.velocity = {velocity_rpm};
    joint_state_pub_->publish(js);

    std_msgs::msg::UInt16 sw_msg;
    sw_msg.data = status_word;
    status_word_pub_->publish(sw_msg);

    std_msgs::msg::UInt16 fault_msg;
    fault_msg.data = fault_code;
    fault_code_pub_->publish(fault_msg);
  }

  DriverParams params_;
  GiantCrabCanDriver driver_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_word_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr fault_code_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_cmd_sub_;

  rclcpp::Service<srv::GetAngle>::SharedPtr get_angle_srv_;
  rclcpp::Service<srv::SetAngle>::SharedPtr set_angle_srv_;
  rclcpp::Service<srv::SetLimits>::SharedPtr set_limits_srv_;
  rclcpp::Service<srv::SetSpeed>::SharedPtr set_speed_srv_;
  rclcpp::Service<srv::GetStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_zero_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace giantcrab_joint_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<giantcrab_joint_driver::GiantCrabJointNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "giantcrab_joint_node 异常退出: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
