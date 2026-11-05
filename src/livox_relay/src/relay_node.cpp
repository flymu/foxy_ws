// relay_node.cpp — livox_relay: CustomMsg → PointCloud2 本地中继（D2 修复核心，2026-11-05）
//
// 为什么存在（11-05 戳龄病解剖定案）：
//   rclpy 反序列化 600KB CustomMsg（~2 万点 ctypes 对象）要 ~190ms/帧 → 所有
//   python 订阅者被钉死在 ~5.3Hz，且从 depth 历史里读旧帧 → "读侧假滞后"~1.2s
//   （veto 曾是受害者之一）。PointCloud2 的 data 是纯字节序列（rclpy 一次
//   memcpy 进 array.array），python 端 np.frombuffer 零拷贝直取——天花板消失。
//
// 为什么放 Orin 本地：中继与驱动同机，DDS 走 SHM/回环，不占 WiFi 一字节。
//   （D1：/livox/lidar 的 WiFi 订阅者只剩 point_lio 一个。）
//
// 输出格式（python 端 np.frombuffer(m.data, np.float32).reshape(-1, 4)）：
//   每点 16B：x, y, z, intensity（float32×4）；intensity = livox reflectivity(0~255)
//   保留原 header（stamp=驱动回调时刻、frame_id）——戳龄仪表语义不变。
//
// 自带 10s 统计日志（收帧率+戳龄）——同时是 D1 的仪表：WiFi 负载棘轮（D1）
// 复发时，这里第一个掉率/涨戳龄（本地 C++ 读侧没有 rclpy 天花板，掉率只怪上游）。

#include <chrono>
#include <cstdint>
#include <memory>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

using livox_ros_driver2::msg::CustomMsg;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class RelayNode : public rclcpp::Node
{
public:
  RelayNode()
  : Node("livox_relay"),
    last_log_(std::chrono::steady_clock::now())
  {
    sub_ = create_subscription<CustomMsg>(
      "/livox/lidar", rclcpp::SensorDataQoS(),
      [this](const CustomMsg::SharedPtr msg) { on_cloud(msg); });
    pub_ = create_publisher<PointCloud2>("/livox/points2", rclcpp::SensorDataQoS());
    RCLCPP_INFO(get_logger(),
      "livox_relay 就绪: /livox/lidar (CustomMsg) -> /livox/points2 (PointCloud2, 16B/点)");
  }

private:
  void on_cloud(const CustomMsg::SharedPtr msg)
  {
    const size_t n = msg->points.size();
    auto out = std::make_shared<PointCloud2>();
    out->header = msg->header;              // 原样保留 stamp + frame_id
    out->height = 1;
    out->width = static_cast<uint32_t>(n);
    out->is_bigendian = false;
    out->is_dense = true;
    out->point_step = 16;
    out->row_step = static_cast<uint32_t>(16 * n);
    out->fields.resize(4);
    static const char * kName[4] = {"x", "y", "z", "intensity"};
    for (uint32_t i = 0; i < 4; ++i) {
      PointField & f = out->fields[i];
      f.name = kName[i];
      f.offset = i * 4;
      f.datatype = PointField::FLOAT32;
      f.count = 1;
    }
    out->data.resize(out->row_step);
    float * dst = reinterpret_cast<float *>(out->data.data());
    const auto & pts = msg->points;
    for (size_t i = 0; i < n; ++i) {
      const auto & p = pts[i];
      float * q = dst + i * 4;
      q[0] = p.x;
      q[1] = p.y;
      q[2] = p.z;
      q[3] = static_cast<float>(p.reflectivity);
    }
    // 本机 rclcpp 的 publish 重载集(unique_ptr/const&/serialized/Loaned)
    // 没有 shared_ptr 版本——走 const MessageT&，内部一次 320KB 拷贝，可忽略
    pub_->publish(*out);

    // ---- 10s 统计：D1 仪表（本地 C++ 读侧的收帧率 + 戳龄）----
    ++n_frames_;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_log_ >= std::chrono::seconds(10)) {
      const double dt = std::chrono::duration<double>(now - last_log_).count();
      const double age = (this->now() - rclcpp::Time(msg->header.stamp)).seconds();
      RCLCPP_INFO(get_logger(), "中继: %.2f Hz (10s窗) 上帧 %zu 点 戳龄 %.3fs",
        n_frames_ / dt, n, age);
      n_frames_ = 0;
      last_log_ = now;
    }
  }

  rclcpp::Subscription<CustomMsg>::SharedPtr sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
  uint64_t n_frames_ = 0;
  std::chrono::steady_clock::time_point last_log_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayNode>());
  rclcpp::shutdown();
  return 0;
}
