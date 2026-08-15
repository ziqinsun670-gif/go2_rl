#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace {

constexpr std::array<float, 12> kDefaultPosition = {
    0.0F, 0.8F, -1.5F, 0.0F, 0.8F, -1.5F,
    0.0F, 0.8F, -1.5F, 0.0F, 0.8F, -1.5F,
};
constexpr std::array<float, 12> kActionScale = {
    0.125F, 0.25F, 0.25F, 0.125F, 0.25F, 0.25F,
    0.125F, 0.25F, 0.25F, 0.125F, 0.25F, 0.25F,
};

double Percentile(std::vector<double> samples, double quantile) {
    std::sort(samples.begin(), samples.end());
    const auto index = std::min(
        samples.size() - 1,
        static_cast<std::size_t>(std::ceil(quantile * samples.size()) - 1));
    return samples[index];
}

}  // namespace

int main(int argc, char** argv) try {
    const auto model_path = argc > 1
        ? std::filesystem::path(argv[1])
        : std::filesystem::path("../models/go2_robot_lab/policy.onnx");
    const int iterations = argc > 2 ? std::stoi(argv[2]) : 10000;
    if (!std::filesystem::is_regular_file(model_path) || iterations < 1) {
        throw std::runtime_error("usage: go2_policy_benchmark MODEL.onnx [ITERATIONS]");
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "go2_policy");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env, model_path.c_str(), options);

    const auto input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape != std::vector<int64_t>({1, 45}) || output_shape != std::vector<int64_t>({1, 12})) {
        throw std::runtime_error("unexpected policy input or output shape");
    }

    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name = session.GetInputNameAllocated(0, allocator);
    auto output_name = session.GetOutputNameAllocated(0, allocator);
    const char* input_names[] = {input_name.get()};
    const char* output_names[] = {output_name.get()};

    std::array<float, 45> observation{};
    observation[5] = -1.0F;  // upright projected gravity
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    auto run = [&]() {
        auto tensor = Ort::Value::CreateTensor<float>(
            memory, observation.data(), observation.size(), input_shape.data(), input_shape.size());
        return session.Run(
            Ort::RunOptions{nullptr}, input_names, &tensor, 1, output_names, 1);
    };

    for (int i = 0; i < 200; ++i) run();
    std::vector<double> latency_ms;
    latency_ms.reserve(iterations);
    for (int i = 0; i < iterations; ++i) {
        const auto start = std::chrono::steady_clock::now();
        run();
        const auto elapsed = std::chrono::steady_clock::now() - start;
        latency_ms.push_back(std::chrono::duration<double, std::milli>(elapsed).count());
    }

    auto output = run();
    const float* actions = output.front().GetTensorData<float>();
    std::array<float, 12> targets{};
    for (std::size_t i = 0; i < targets.size(); ++i) {
        if (!std::isfinite(actions[i])) throw std::runtime_error("policy returned a non-finite action");
        targets[i] = kDefaultPosition[i] + actions[i] * kActionScale[i];
    }

    const double mean = std::accumulate(latency_ms.begin(), latency_ms.end(), 0.0) / latency_ms.size();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "model=" << model_path << '\n';
    std::cout << "input=[1,45] output=[1,12] iterations=" << iterations << '\n';
    std::cout << "mean_ms=" << mean
              << " p95_ms=" << Percentile(latency_ms, 0.95)
              << " max_ms=" << *std::max_element(latency_ms.begin(), latency_ms.end()) << '\n';
    std::cout << "joint_targets=";
    for (float target : targets) std::cout << target << ' ';
    std::cout << '\n';
    return 0;
} catch (const std::exception& error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
}
