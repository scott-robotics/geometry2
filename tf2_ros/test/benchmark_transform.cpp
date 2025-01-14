#include <benchmark/benchmark.h>

#include "tf2_ros/buffer.h"


auto generate_transform()
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "foo";
    transform.header.stamp.sec = 0;
    transform.header.stamp.nanosec = 0;
    transform.child_frame_id = "bar";
    transform.transform.translation.x = 42.0;
    transform.transform.translation.y = -3.14;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.591922;
    transform.transform.rotation.y = 0.210684;
    transform.transform.rotation.z = -0.762476;
    transform.transform.rotation.w = 0.154502;
    return transform;
}

static void benchmark_set_transform(benchmark::State & state)
{
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    buffer.setUsingDedicatedThread(false);

    auto transform = generate_transform();

    for (auto _ : state) {
        transform.header.stamp.nanosec++;
        buffer.setTransform(transform, "benchmark");
    }
}

static void benchmark_get_transform(benchmark::State & state)
{
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    buffer.setUsingDedicatedThread(false);

    auto transform = generate_transform();
    buffer.setTransform(transform, "benchmark");

    for (auto _ : state) {
        buffer.lookupTransform("foo", "bar", rclcpp::Time(0));
    }
}

static void benchmark_resolve_transform(benchmark::State & state)
{
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    tf2_ros::Buffer buffer(clock);
    buffer.setUsingDedicatedThread(false);

    auto transform = generate_transform();
    buffer.setTransform(transform, "benchmark");

    transform.header.frame_id = "bar";
    transform.child_frame_id = "lum";
    buffer.setTransform(transform, "benchmark");

    transform.header.frame_id = "lum";
    transform.child_frame_id = "ber";
    buffer.setTransform(transform, "benchmark");

    transform.header.frame_id = "ber";
    transform.child_frame_id = "tin";
    buffer.setTransform(transform, "benchmark");

    for (auto _ : state) {
        buffer.lookupTransform("foo", "tin", rclcpp::Time(0));
    }
}

BENCHMARK(benchmark_set_transform);
BENCHMARK(benchmark_get_transform);
BENCHMARK(benchmark_resolve_transform);
