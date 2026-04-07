/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "humanoid_interface/common/ModelSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <array>
#include <cstdio>
#include <memory>
#include <ctime>
#include <fstream>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace humanoid {

namespace {
// 内部函数：计算URDF文件的MD5哈希值
std::string calculateUrdfHash(const std::string& urdfFile) {
    std::string command = "md5sum " + urdfFile;
    std::array<char, 128> buffer;
    std::string result;
    
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return "default";
    }
    
    if (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result = std::string(buffer.data()).substr(0, 8); // 只取前8位
    } else {
        result = "default";
    }
    pclose(pipe);
    return result;
}
}  // namespace

ModelSettings loadModelSettings(const std::string& filename, const std::string& urdfFile, const std::string& fieldName, bool verbose) {
  ModelSettings modelSettings;
  std::cout << "Loading Model Settings from file: " << filename << std::endl;
  loadData::loadStdVector<std::string>(filename, fieldName+".contactNames3DoF", modelSettings.contactNames3DoF, verbose);

  loadData::loadStdVector<std::string>(filename, fieldName+".contactNames6DoF", modelSettings.contactNames6DoF, verbose);

  loadData::loadStdVector<std::string>(filename, fieldName+".jointNames", modelSettings.jointNames, verbose);
  loadData::loadStdVector<std::string>(filename, fieldName+".jointNamesReal", modelSettings.jointNamesReal, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Legged Robot Model Settings:";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadPtreeValue(pt, modelSettings.positionErrorGain, fieldName + ".positionErrorGain", verbose);
  loadData::loadPtreeValue(pt, modelSettings.positionErrorGainZStepControl, fieldName + ".positionErrorGainZStepControl", verbose);
  loadData::loadPtreeValue(pt, modelSettings.positionErrorGain_xy, fieldName + ".positionErrorGain_xy", verbose);
  loadData::loadPtreeValue(pt, modelSettings.velocityErrorGain_xy, fieldName + ".velocityErrorGain_xy", verbose);
  loadData::loadPtreeValue(pt, modelSettings.positionErrorGain_zero, fieldName + ".positionErrorGain_zero", verbose);
  loadData::loadPtreeValue(pt, modelSettings.positionErrorGainXYStepControl, fieldName + ".positionErrorGainXYStepControl", verbose);
  loadData::loadPtreeValue(pt, modelSettings.velocityErrorGainXYStepControl, fieldName + ".velocityErrorGainXYStepControl", verbose);

  loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime, fieldName + ".phaseTransitionStanceTime", verbose);
  loadData::loadPtreeValue(pt, modelSettings.verboseCppAd, fieldName + ".verboseCppAd", verbose);
  loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd", verbose);
  loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);

  loadData::loadPtreeValue(pt, modelSettings.info.baseFrame, fieldName + ".baseFrame", verbose);
  loadData::loadStdVector<std::string>(filename, fieldName + ".eeFrame", modelSettings.info.eeFrame, verbose);
  loadData::loadPtreeValue(pt, modelSettings.mpcArmsDof, fieldName + ".mpcArmsDof", verbose);
  loadData::loadPtreeValue(pt, modelSettings.mpcLegsDof, fieldName + ".mpcLegsDof", verbose);
  loadData::loadPtreeValue(pt, modelSettings.mpcWaistDof, fieldName + ".mpcWaistDof", verbose);
  loadData::loadPtreeValue(pt, modelSettings.modelDof, fieldName + ".modelDof", verbose);
  
  int centroidal_model_type = 0;
  loadData::loadPtreeValue(pt, centroidal_model_type, "centroidalModelType", verbose);

  // 计算URDF哈希值
  std::string urdfHash;
  if (urdfFile == "default"){
    urdfHash = "default";
  }else{
    urdfHash = calculateUrdfHash(urdfFile);
  }

  std::string cppad_use_timestamp_dir = modelSettings.modelFolderCppAd;
  
  // 修改modelFolderCppAd路径，加入URDF哈希值
  modelSettings.modelFolderCppAd += "/md5_" + urdfHash + "/" + std::to_string(centroidal_model_type) + 
                                   "/" + std::to_string(modelSettings.mpcArmsDof + modelSettings.mpcLegsDof + modelSettings.mpcWaistDof) + "dof" ;
  // 创建目录
  if (urdfFile != "default")
  {
    // 获取当前时间戳
    std::time_t now = std::time(nullptr);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    cppad_use_timestamp_dir += "/md5_" + urdfHash;
    // 先删除所有已存在的时间戳文件夹
    std::string cleanCommand = "rm -rf " + cppad_use_timestamp_dir + "/build_timestamp_*";
    system(cleanCommand.c_str());
    
    // 创建带时间戳的目录
    std::string timestampDir = cppad_use_timestamp_dir + "/build_timestamp_" + std::string(timestamp);
    std::string command = "mkdir -p " + timestampDir;
    if (system(command.c_str()) != 0) {
      std::cerr << "[ModelSettings] Warning: Failed to create directory: " << timestampDir << std::endl;
    }
  }
  
  std::cout << "[ModelSettings]:Cppad Model folder: " << modelSettings.modelFolderCppAd << std::endl;

  // Adjust jointNames size if it is not equal to mpcArmsDof + mpcLegsDof + mpcWaistDof
  if (modelSettings.jointNames.size() != modelSettings.mpcArmsDof + modelSettings.mpcLegsDof + modelSettings.mpcWaistDof)
  {
    std::cerr << "[ModelSettings::loadModelSettings] Warning: jointNames size is not equal to mpcArmsDof + mpcLegsDof + mpcWaistDof, scaling with mpcArmsDof" << std::endl;
    std::vector<std::string> adjustedJointNames;
    if (modelSettings.jointNames.size() < modelSettings.mpcArmsDof + modelSettings.mpcLegsDof + modelSettings.mpcWaistDof)
    {
      std::cerr << "[ModelSettings::loadModelSettings] Error: jointNames size is less than mpcArmsDof + mpcLegsDof + mpcWaistDof, which is not allowed" << std::endl;
      exit(1);
    }
    for (int i = 0; i < modelSettings.mpcLegsDof; i++)
    {
      adjustedJointNames.push_back(modelSettings.jointNames[i]);
    }

    for (int i = 0; i < modelSettings.mpcWaistDof; i++)
    {
      adjustedJointNames.push_back(modelSettings.jointNames[modelSettings.mpcLegsDof + i]);
    }

    int singleArmDof = modelSettings.mpcArmsDof / 2;
    int singleArmDofOriginal = (modelSettings.jointNames.size() - modelSettings.mpcLegsDof - modelSettings.mpcWaistDof) / 2;
    for (int i = 0; i < 2; i++)
    {
      for (int j = 0; j < singleArmDof; j++)
      {
        adjustedJointNames.push_back(modelSettings.jointNames[modelSettings.mpcLegsDof + modelSettings.mpcWaistDof + i*singleArmDofOriginal + j]);
      }
    }
    modelSettings.jointNames = adjustedJointNames;
  }

  for (auto& jointName : modelSettings.jointNamesReal)
  {
    if (std::find(modelSettings.jointNames.begin(), modelSettings.jointNames.end(), jointName) == modelSettings.jointNames.end())
    {
      modelSettings.simplifiedJointNames.push_back(jointName);
    }
  }

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
  return modelSettings;
}

}  // namespace humanoid
}  // namespace ocs2
