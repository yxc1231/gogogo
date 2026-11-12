/**********************************************************************
 Copyright (c) 2026, Go1 FSM State Machine System. All rights reserved.
***********************************************************************/

#include "go1_fsm/model_loader.h"
#include <cstdio>

// 按编译选项条件包含
#ifdef HAS_ONNXRUNTIME
#include "onnxruntime_cxx_api.h"
#endif
#ifdef HAS_LIBTORCH
#include <torch/script.h>
#include <torch/torch.h>
#endif

namespace GO1_FSM {

// ============================================================
// ONNX Runtime 后端
// ============================================================
#ifdef HAS_ONNXRUNTIME
class ONNXModel : public ModelLoader {
public:
    explicit ONNXModel(const std::string& device) : _device(device) {}

    bool load(const std::string& path) override {
        try {
            _env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "Go1FSM");
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(1);
            _session = std::make_unique<Ort::Session>(*_env, path.c_str(), opts);
            
            // 获取输入输出名称
            Ort::AllocatorWithDefaultOptions alloc;
            size_t n_in = _session->GetInputCount();
            size_t n_out = _session->GetOutputCount();
            for (size_t i = 0; i < n_in; ++i)
                _in_names.push_back(_session->GetInputNameAllocated(i, alloc).get());
            for (size_t i = 0; i < n_out; ++i)
                _out_names.push_back(_session->GetOutputNameAllocated(i, alloc).get());
            
            printf("[Model] ONNX loaded: %s (%zu inputs, %zu outputs)\n",
                   path.c_str(), n_in, n_out);
            return true;
        } catch (const std::exception& e) {
            printf("[Model] ONNX load error: %s\n", e.what());
            return false;
        }
    }

    bool inference(const std::vector<float>& obs, std::vector<float>& act) override {
        if (!_session) return false;
        try {
            Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            std::array<int64_t, 2> shape = {1, static_cast<int64_t>(obs.size())};
            Ort::Value in = Ort::Value::CreateTensor<float>(mem,
                const_cast<float*>(obs.data()), obs.size(), shape.data(), shape.size());
            
            std::vector<const char*> in_n, out_n;
            for (auto& s : _in_names)  in_n.push_back(s.c_str());
            for (auto& s : _out_names) out_n.push_back(s.c_str());
            
            auto outputs = _session->Run(Ort::RunOptions{nullptr},
                in_n.data(), &in, 1, out_n.data(), out_n.size());
            
            float* data = outputs[0].GetTensorMutableData<float>();
            auto info = outputs[0].GetTensorTypeAndShapeInfo();
            size_t n = info.GetElementCount();
            act.resize(n);
            std::copy(data, data + n, act.begin());
            return true;
        } catch (const std::exception& e) {
            printf("[Model] ONNX inference error: %s\n", e.what());
            return false;
        }
    }

    bool isLoaded() const override { return _session != nullptr; }
    const char* formatName() const override { return "ONNX"; }

private:
    std::string _device;
    std::unique_ptr<Ort::Env> _env;
    std::unique_ptr<Ort::Session> _session;
    std::vector<std::string> _in_names, _out_names;
};
#endif  // HAS_ONNXRUNTIME

// ============================================================
// LibTorch 后端
// ============================================================
#ifdef HAS_LIBTORCH
class TorchModel : public ModelLoader {
public:
    explicit TorchModel(const std::string& device) : _device(device) {}

    bool load(const std::string& path) override {
        try {
            _module = torch::jit::load(path);
            if (_device == "cuda") _module.to(torch::kCUDA);
            _module.eval();
            printf("[Model] Torch loaded: %s\n", path.c_str());
            return true;
        } catch (const std::exception& e) {
            printf("[Model] Torch load error: %s\n", e.what());
            return false;
        }
    }

    bool inference(const std::vector<float>& obs, std::vector<float>& act) override {
        if (!_module) return false;
        try {
            auto opts = torch::TensorOptions().dtype(torch::kFloat32);
            torch::Tensor in = torch::from_blob(
                const_cast<float*>(obs.data()), {1, static_cast<long>(obs.size())}, opts);
            if (_device == "cuda") in = in.to(torch::kCUDA);
            
            torch::NoGradGuard no_grad;
            auto out = _module.forward({in}).toTensor().to(torch::kCPU);
            
            act.resize(out.numel());
            std::copy(out.data_ptr<float>(), out.data_ptr<float>() + out.numel(), act.begin());
            return true;
        } catch (const std::exception& e) {
            printf("[Model] Torch inference error: %s\n", e.what());
            return false;
        }
    }

    bool isLoaded() const override { return _module != nullptr; }
    const char* formatName() const override { return "LibTorch"; }

private:
    torch::jit::script::Module _module;
    std::string _device;
};
#endif  // HAS_LIBTORCH

// ============================================================
// 工厂
// ============================================================
std::unique_ptr<ModelLoader> ModelLoader::create(const std::string& format,
                                                  const std::string& device) {
#ifdef HAS_ONNXRUNTIME
    if (format == "onnx") return std::make_unique<ONNXModel>(device);
#endif
#ifdef HAS_LIBTORCH
    if (format == "pt")   return std::make_unique<TorchModel>(device);
#endif
    printf("[Model] No backend for '%s'. Build: cmake -DUSE_ONNX=ON/-DUSE_TORCH=ON\n", format.c_str());
    return nullptr;
}

}  // namespace GO1_FSM
