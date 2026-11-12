// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "onnxruntime_cxx_api.h"
#include <deque>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stdexcept>

namespace isaaclab
{

class Algorithms
{
public:
    virtual std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) = 0;
    virtual void reset() {}

    std::vector<float> get_action()
    {
        std::lock_guard<std::mutex> lock(act_mtx_);
        return action;
    }
    
    std::vector<float> action;
protected:
    std::mutex act_mtx_;
};

class OrtRunner : public Algorithms
{
public:
    OrtRunner(std::string model_path)
    {
        // Init Model
        env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "onnx_model");
        session_options.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);

        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);

        for (size_t i = 0; i < session->GetInputCount(); ++i) {
            Ort::TypeInfo input_type = session->GetInputTypeInfo(i);
            input_shapes.push_back(input_type.GetTensorTypeAndShapeInfo().GetShape());
            auto input_name = session->GetInputNameAllocated(i, allocator);
            input_names.push_back(input_name.release());
        }

        for (const auto& shape : input_shapes) {
            size_t size = 1;
            for (const auto& dim : shape) {
                size *= dim;
            }
            input_sizes.push_back(size);
        }

        // Get output shape
        Ort::TypeInfo output_type = session->GetOutputTypeInfo(0);
        output_shape = output_type.GetTensorTypeAndShapeInfo().GetShape();
        auto output_name = session->GetOutputNameAllocated(0, allocator);
        output_names.push_back(output_name.release());

        action.resize(output_shape[1]);
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs)
    {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // make sure all input names are in obs
        for (const auto& name : input_names) {
            if (obs.find(name) == obs.end()) {
                throw std::runtime_error("Input name " + std::string(name) + " not found in observations.");
            }
        }

        // Create input tensors
        std::vector<Ort::Value> input_tensors;
        for(int i(0); i<input_names.size(); ++i)
        {
            const std::string name_str(input_names[i]);
            auto& input_data = obs.at(name_str);
            if (input_data.size() != input_sizes[i]) {
                throw std::runtime_error(
                    "ONNX input '" + name_str + "' expects " + std::to_string(input_sizes[i]) +
                    " floats, but observation manager produced " + std::to_string(input_data.size()));
            }
            auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_data.data(), input_sizes[i], input_shapes[i].data(), input_shapes[i].size());
            input_tensors.push_back(std::move(input_tensor));
        }

        // Run the model
        auto output_tensor = session->Run(Ort::RunOptions{nullptr}, input_names.data(), input_tensors.data(), input_tensors.size(), output_names.data(), 1);

        // Copy output data
        auto floatarr = output_tensor.front().GetTensorMutableData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), floatarr, output_shape[1] * sizeof(float));
        return action;
    }

private:
    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;

    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<int64_t> input_sizes;
    std::vector<int64_t> output_shape;
};

// HIM policies export the actor and history encoder as separate ONNX models.
// The actor input is [current_proprioception, encoder(history)], not merely
// the current observation.  History follows the legacy deployment layout:
// current frame first, followed by older frames.
class HimOrtRunner : public Algorithms
{
public:
    HimOrtRunner(const std::string& policy_path, const std::string& encoder_path, size_t history_length)
    : history_length_(history_length), env_(ORT_LOGGING_LEVEL_WARNING, "him_onnx_model")
    {
        if (history_length_ == 0) {
            throw std::runtime_error("HIM history_length must be greater than zero");
        }
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_EXTENDED);
        policy_session_ = std::make_unique<Ort::Session>(env_, policy_path.c_str(), session_options_);
        encoder_session_ = std::make_unique<Ort::Session>(env_, encoder_path.c_str(), session_options_);

        if (policy_session_->GetInputCount() != 1 || policy_session_->GetOutputCount() != 1 ||
            encoder_session_->GetInputCount() != 1 || encoder_session_->GetOutputCount() != 1) {
            throw std::runtime_error("HIM deployment supports exactly one input and one output for each ONNX model");
        }

        policy_input_shape_ = input_shape(*policy_session_);
        encoder_input_shape_ = input_shape(*encoder_session_);
        policy_output_shape_ = output_shape(*policy_session_);
        encoder_output_shape_ = output_shape(*encoder_session_);
        policy_input_size_ = tensor_size(policy_input_shape_);
        encoder_input_size_ = tensor_size(encoder_input_shape_);
        policy_output_size_ = tensor_size(policy_output_shape_);
        encoder_output_size_ = tensor_size(encoder_output_shape_);

        if (encoder_input_size_ % history_length_ != 0) {
            throw std::runtime_error("encoder input dimension is not divisible by history_length");
        }
        obs_dim_ = encoder_input_size_ / history_length_;
        if (policy_input_size_ != obs_dim_ + encoder_output_size_) {
            throw std::runtime_error(
                "HIM model dimensions are inconsistent: policy expects " + std::to_string(policy_input_size_) +
                ", but current obs + encoder output is " + std::to_string(obs_dim_ + encoder_output_size_));
        }

        policy_input_name_ = policy_session_->GetInputNameAllocated(0, allocator_).release();
        policy_output_name_ = policy_session_->GetOutputNameAllocated(0, allocator_).release();
        encoder_input_name_ = encoder_session_->GetInputNameAllocated(0, allocator_).release();
        encoder_output_name_ = encoder_session_->GetOutputNameAllocated(0, allocator_).release();
        action.resize(policy_output_size_);
    }

    void reset() override
    {
        history_.clear();
    }

    std::vector<float> act(std::unordered_map<std::string, std::vector<float>> obs) override
    {
        const auto it = obs.find("obs");
        if (it == obs.end()) {
            throw std::runtime_error("HIM policy requires an observation group named 'obs'");
        }
        const auto& current_obs = it->second;
        if (current_obs.size() != obs_dim_) {
            throw std::runtime_error(
                "HIM encoder expects " + std::to_string(obs_dim_) + " current observation floats, but got " +
                std::to_string(current_obs.size()));
        }

        if (history_.empty()) {
            // Match the legacy deployment: current observation followed by
            // zero-filled previous frames at the start of an episode.
            history_.push_back(current_obs);
            for (size_t i = 1; i < history_length_; ++i) {
                history_.emplace_back(obs_dim_, 0.0f);
            }
        } else {
            history_.push_front(current_obs);
            if (history_.size() > history_length_) history_.pop_back();
        }

        std::vector<float> obs_history;
        obs_history.reserve(encoder_input_size_);
        for (const auto& frame : history_) {
            obs_history.insert(obs_history.end(), frame.begin(), frame.end());
        }

        const auto memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
        auto encoder_input = Ort::Value::CreateTensor<float>(
            memory_info, obs_history.data(), obs_history.size(),
            encoder_input_shape_.data(), encoder_input_shape_.size());
        const char* encoder_input_names[] = {encoder_input_name_};
        const char* encoder_output_names[] = {encoder_output_name_};
        auto encoder_result = encoder_session_->Run(
            Ort::RunOptions{nullptr}, encoder_input_names, &encoder_input, 1,
            encoder_output_names, 1);
        const float* latent = encoder_result.front().GetTensorData<float>();

        std::vector<float> policy_obs;
        policy_obs.reserve(policy_input_size_);
        policy_obs.insert(policy_obs.end(), current_obs.begin(), current_obs.end());
        policy_obs.insert(policy_obs.end(), latent, latent + encoder_output_size_);

        auto policy_input = Ort::Value::CreateTensor<float>(
            memory_info, policy_obs.data(), policy_obs.size(),
            policy_input_shape_.data(), policy_input_shape_.size());
        const char* policy_input_names[] = {policy_input_name_};
        const char* policy_output_names[] = {policy_output_name_};
        auto policy_result = policy_session_->Run(
            Ort::RunOptions{nullptr}, policy_input_names, &policy_input, 1,
            policy_output_names, 1);

        const float* output = policy_result.front().GetTensorData<float>();
        std::lock_guard<std::mutex> lock(act_mtx_);
        std::memcpy(action.data(), output, policy_output_size_ * sizeof(float));
        return action;
    }

private:
    static std::vector<int64_t> input_shape(Ort::Session& session)
    {
        return session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    }

    static std::vector<int64_t> output_shape(Ort::Session& session)
    {
        return session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    }

    static size_t tensor_size(const std::vector<int64_t>& shape)
    {
        size_t size = 1;
        for (const auto dim : shape) {
            if (dim <= 0) throw std::runtime_error("dynamic ONNX tensor shapes are not supported");
            size *= static_cast<size_t>(dim);
        }
        return size;
    }

    size_t history_length_;
    size_t obs_dim_ = 0;
    size_t policy_input_size_ = 0;
    size_t encoder_input_size_ = 0;
    size_t policy_output_size_ = 0;
    size_t encoder_output_size_ = 0;
    std::deque<std::vector<float>> history_;

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> policy_session_;
    std::unique_ptr<Ort::Session> encoder_session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    const char* policy_input_name_ = nullptr;
    const char* policy_output_name_ = nullptr;
    const char* encoder_input_name_ = nullptr;
    const char* encoder_output_name_ = nullptr;
    std::vector<int64_t> policy_input_shape_;
    std::vector<int64_t> encoder_input_shape_;
    std::vector<int64_t> policy_output_shape_;
    std::vector<int64_t> encoder_output_shape_;
};
};
