// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "tensorrt_common/tensorrt_common.hpp"

namespace fs = std::filesystem;

void generateEngineForModel(const std::string& onnx_path, const std::string& precision)
{
    std::cout << "Generating " << precision << " engine for: " << onnx_path << std::endl;
    
    try {
        tensorrt_common::BatchConfig batch_config = {1, 1, 1};
        size_t max_workspace_size = (1ULL << 33); // 8GB
        tensorrt_common::BuildConfig build_config;
        
        auto trt_common = std::make_unique<tensorrt_common::TrtCommon>(
            onnx_path, precision, nullptr, batch_config, max_workspace_size, build_config, std::vector<std::string>{});
        
        trt_common->setup();
        
        if (trt_common->isInitialized()) {
            std::cout << "Successfully generated engine for: " << onnx_path << std::endl;
        } else {
            std::cerr << "Failed to generate engine for: " << onnx_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error generating engine for " << onnx_path << ": " << e.what() << std::endl;
    }
}

int main(int argc, char* argv[])
{
    std::string models_dir = "models/";
    std::vector<std::string> precisions = {"fp16"}; // "fp32"
    
    // Check if models directory argument is provided
    if (argc > 1) {
        models_dir = argv[1];
    }
    
    if (!fs::exists(models_dir)) {
        std::cerr << "Models directory does not exist: " << models_dir << std::endl;
        return 1;
    }
    
    std::cout << "Searching for ONNX models in: " << models_dir << std::endl;
    
    // Find all ONNX files in the models directory
    std::vector<std::string> onnx_files;
    try {
        for (const auto& entry : fs::directory_iterator(models_dir)) {
            if (entry.is_regular_file() && entry.path().extension() == ".onnx") {
                onnx_files.push_back(entry.path().string());
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error reading directory: " << e.what() << std::endl;
        return 1;
    }
    
    if (onnx_files.empty()) {
        std::cout << "No ONNX files found in: " << models_dir << std::endl;
        return 0;
    }
    
    std::cout << "Found " << onnx_files.size() << " ONNX files:" << std::endl;
    for (const auto& file : onnx_files) {
        std::cout << "  " << file << std::endl;
    }
    
    // Generate engines for each model and precision
    for (const auto& onnx_file : onnx_files) {
        for (const auto& precision : precisions) {
            generateEngineForModel(onnx_file, precision);
        }
        std::cout << std::endl;
    }
    
    std::cout << "Engine generation completed!" << std::endl;
    return 0;
}
