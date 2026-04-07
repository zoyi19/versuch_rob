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

#include "humanoid_interface/gait/GaitSchedule.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2
{
  namespace humanoid
  {
    std::ostream& operator<<(std::ostream& os, const FootPoseSchedule& schedule)
    {
        os << "Event Times: ";
        for (const auto& time : schedule.eventTimes)
        {
          os << time << " ";
        }
        os << "\n";

        os << "Foot Indices: ";
        for (const auto& index : schedule.footIndices)
        {
          os << static_cast<int>(index) << " ";
        }
        os << "\n";

        os << "Foot Pose Sequence:\n";
        for (const auto& pose : schedule.footPoseSequence)
        {
          os << "(" << pose.x() << ", " << pose.y() << ", " << pose.z() << ", " << pose.w() << ", " << pose(4) << ", " << pose(5) << ") ";
        }
        os << "\n";

        os << "Torso Pose Sequence:\n";
        for (const auto& pose : schedule.torsoPoseSequence)
        {
          os << "(" << pose.x() << ", " << pose.y() << ", " << pose.z() << ", " << pose.w() << ", " << pose(4) << ", " << pose(5) << ") ";
        }
        os << "\n";

        return os;
    }

    std::map<FootIdx, ModeNumber> footIdxToNum = {
      {FootIdx::Left, ModeNumber::FS},
      {FootIdx::Right, ModeNumber::SF},
      {FootIdx::Stance, ModeNumber::SS},
      {FootIdx::ST, ModeNumber::ST},
      {FootIdx::TS, ModeNumber::TS},
      {FootIdx::SH, ModeNumber::SH},
      {FootIdx::HS, ModeNumber::HS},
      {FootIdx::TH, ModeNumber::TH},
      {FootIdx::HT, ModeNumber::HT},

    };
    std::map<ModeNumber, FootIdx> numToFootIdx = {
      {ModeNumber::FS, FootIdx::Left},
      {ModeNumber::SF, FootIdx::Right},
      {ModeNumber::SS, FootIdx::Stance},
      {ModeNumber::ST, FootIdx::ST},
      {ModeNumber::TS, FootIdx::TS},
      {ModeNumber::SH, FootIdx::SH},
      {ModeNumber::HS, FootIdx::HS},
      {ModeNumber::TH, FootIdx::TH},
      {ModeNumber::HT, FootIdx::HT},
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    GaitSchedule::GaitSchedule(const std::string &referenceFile, const std::string &gaitFile, scalar_t phaseTransitionStanceTime)
        : phaseTransitionStanceTime_(phaseTransitionStanceTime),
          modeSchedule_(loadModeSchedule(referenceFile, "initialModeSchedule", false)),
          modeSequenceTemplate_(loadModeSequenceTemplate(referenceFile, "defaultModeSequenceTemplate", false)),
          defaultModeSequenceTemplate_(loadModeSequenceTemplate(referenceFile, "defaultModeSequenceTemplate", false)),
          gaitManager_(20)
    {
      std::cout << "gaitFile:" << gaitFile << std::endl;
      currentModeSchedule_ = modeSchedule_;
      prev_left_mode_ = modeNumber2String(modeSchedule_.modeSequence[1])[0];
      prev_right_mode_ = modeNumber2String(modeSchedule_.modeSequence[1])[1];
      loadData::loadStdVector(gaitFile, "list", gaitList_, false);
      loadData::loadCppDataType(referenceFile, "default_stance_duration_", default_stance_duration_);

      gaitMap_.clear();
      for (const auto &gaitName : gaitList_)
      {
        std::cout << "loading gait: " << gaitName << std::endl;
        gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, true)});
      }
      modeSequenceTemplate_ = gaitMap_.at("stance");
      gaitTimeName_ = {0.0, getModeScheduleName(gaitMap_,modeSequenceTemplate_)};
      gaitManager_.add(gaitTimeName_);

    }

    scalar_t GaitSchedule::getModeEndTime(scalar_t &modeTime)
    {
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &eventTimes = modeSchedule_.eventTimes;
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), modeTime) - eventTimes.begin();

      return eventTimes[index];
    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate &modeSequenceTemplate, scalar_t startTime, scalar_t finalTime)
    {
      modeSequenceTemplate_ = modeSequenceTemplate;
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;

      // gaitTimeName_ = {startTime, getModeScheduleName(modeSequenceTemplate)};
      // find the index on which the new gait should be added
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();
      auto mode_end_time = eventTimes[index + 1];
      // delete the old logic from the index
      if (index < eventTimes.size())
      {
        eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
        modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
      }

      // add an intermediate stance phase
      scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;
      if (!modeSequence.empty() && modeSequence.back() == ModeNumber::SS)
      {
        phaseTransitionStanceTime = 0.0;
      }

      if (phaseTransitionStanceTime > 0.0)
      {
        std::cout << "phase Transition to Stance delay to " << mode_end_time << std::endl;
        eventTimes.push_back(mode_end_time);
        modeSequence.push_back(ModeNumber::SS);
        startTime = mode_end_time;
      }

      // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
      tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSchedule::scaleModeSequenceTemplate(scalar_t scale, scalar_t startTime, scalar_t finalTime)
    {
      assert(scale > 0.0);
      auto new_template = modeSequenceTemplate_;
      
      std::for_each(new_template.switchingTimes.begin(), new_template.switchingTimes.end(), [&](scalar_t &time)
                    { time *= scale; });
      modifyWalkModeSequenceTemplate(new_template, startTime, finalTime,gaitTimeName_.second);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSchedule::modifyWalkModeSequenceTemplate(const ModeSequenceTemplate &newModeSequenceTemplate, scalar_t startTime, scalar_t timeHorizon,  std::string gaitName)
    {

      if (newModeSequenceTemplate.modeSequence.size() == 0)
      {
        std::cerr << "[GaitSchedule] Invalid mode sequence template for modifying walk mode sequence template." << std::endl;
        return;
      }
      bool verbose = false;
      // gaitTimeName_ = {startTime, getModeScheduleName(newModeSequenceTemplate)};
      // gaitTimeName_.first = startTime; // 名字暂时由单独方法设置
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &enableFootSequence = modeSchedule_.enableFootSequence;
      auto &enableFullBodySequence = modeSchedule_.enableFullBodySequence;
      auto &torsoPoseSequence = modeSchedule_.torsoPoseSequence;
      auto &footPoseSequence = modeSchedule_.footPoseSequence;
      if (verbose)
      {
        std::cout << "eventTimes size: " << eventTimes.size();
        std::cout << " modeSequence size: " << modeSequence.size() << std::endl;
        for (int i = 0; i < modeSequence.size(); ++i)
        {
          std::cout << i << " time: " << ((i < modeSequence.size() - 1) ? std::to_string(eventTimes[i]) : " (last)") << " mode: " << std::to_string(static_cast<int>(modeSequence[i])) << std::endl;
        }
      }
      // 找到新步态应该被添加的索引
      size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();
      index++; // 加一个mode，防止修改当前使用中的mode, 容易导致ddp unstable或者复制冲突
      std::cout << "start index: " << index << " startTime:" << startTime << " timeHorizon:" << timeHorizon << std::endl;
      // 从startIndex开始寻找完全匹配的序列
      auto findMatchingSequence = [&](const size_t start, const ModeSequenceTemplate &mode_template) -> std::optional<size_t>
      {
        if (modeSequence.size() < start + mode_template.modeSequence.size())
        {
          return std::nullopt;
        }
        for (size_t i = start; i <= modeSequence.size() - mode_template.modeSequence.size(); ++i)
        {
          if (std::equal(mode_template.modeSequence.begin(), mode_template.modeSequence.end(),
                         modeSequence.begin() + i))
          {
            return i - 1;
          }
        }
        return std::nullopt;
      };
      size_t new_index = index;
      bool is_switch_to_stance = newModeSequenceTemplate == gaitMap_.at("stance");
      if (is_switch_to_stance) // 切换到站立状态
      {
        // auto it = std::find(modeSequence.begin() + index + 1, modeSequence.end() - 1, ModeNumber::SS); // modeSequence.end()-1一定是ss
        // if (it != modeSequence.end() - 1)
        // {
        //   // 当前gait存在 "SS" 模式，将从该mode之后的所有mode替换为 "SS",时间不变
        //   std::cout << "found stance mode, switch to SS mode now at " << *it << std::endl;
        //   std::fill(it + 1, modeSequence.end(), ModeNumber::SS);
        // }
        // else
        // {
        //   // 不存在 "SS" 模式，查找模板中
        //   auto old_template_match = findMatchingSequence(index, modeSequenceTemplate_); // 查找旧的模板匹配的起点
        //   if (!old_template_match)
        //   {
        //     // 旧模板也没有匹配到，一般不出现，直接插入SS
        //     std::cout << "no match old_template_match in modesequence, insert SS mode at " << index << std::endl;
            std::fill(modeSequence.begin() + index, modeSequence.end(), ModeNumber::SS);
            std::fill(enableFootSequence.begin() + index, enableFootSequence.end(), false);
            std::fill(enableFullBodySequence.begin() + index, enableFullBodySequence.end(), false);
            std::fill(torsoPoseSequence.begin() + index, torsoPoseSequence.end(), torsoPoseSequence.at(index));
            std::fill(footPoseSequence.begin() + index, footPoseSequence.end(), footPoseSequence.at(index));
            gaitTimeName_.first = eventTimes[index];
            
        //   }
        //   else
        //   {
        //     std::cout << "old_template_match: " << *old_template_match << std::endl;
        //     size_t replace_start_index = *old_template_match;
        //     std::fill(modeSequence.begin() + replace_start_index + 1, modeSequence.end(), ModeNumber::SS);
        //   }
        // }
        // }
        new_index = eventTimes.size() - 1;
      }
      else
      {
        auto matchIndex = findMatchingSequence(index, newModeSequenceTemplate); // 查找新模板匹配的起点
        if (!matchIndex)
        {
          // std::cerr << "[GaitSchedule] do not find a matchIndex sequence before the specified start time." << std::endl;
          // index--; // 使用当前index前一个作为替换的起点，即会马上替换成新的mode sequence
          auto old_template_match = findMatchingSequence(index, modeSequenceTemplate_); // 查找旧的模板匹配的起点,切换到新的模板状态时触发
          if (!old_template_match)
          {
            // std::cerr << "[GaitSchedule] Could not find a matchIndex sequence before the specified start time." << std::endl;
            return;
          }
          else
          {
            // std::cout << "old_template_match: " << *old_template_match << std::endl;
            matchIndex = old_template_match;
          }
        }
        new_index = *matchIndex;
      }
      std::cout << "new_index: " << new_index << std::endl;
      index--; // 回退到匹配的起点
      if (new_index < modeSequence.size() - 1 && new_index >= index)
      {
        index = new_index;
      }
      else
      {
        std::cout << "[GaitSchedule]match index " << new_index << " is not in the range, hold the last index." << std::endl;
      }

      modeSequenceTemplate_ = newModeSequenceTemplate;
      if (verbose)
        std::cout << "split index: " << index << std::endl;
      // 从索引处删除旧的逻辑
      if (index < eventTimes.size())
      {
        eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
        modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
      }
      if (!is_switch_to_stance)
        gaitTimeName_.first = eventTimes[index];
      gaitTimeName_.second = gaitName;
      gaitManager_.add(gaitTimeName_);
      std::cout << "[GaitSchedule]gaitTimeName: " << gaitTimeName_.first << " " << gaitTimeName_.second << std::endl;
      // 从 startTime 到 timeHorizon 平铺新的模式序列模板,
      tileModeSequenceTemplate(eventTimes[index], startTime + std::max(timeHorizon * 2, modeSequenceTemplate_.switchingTimes.back()));
      if (verbose)
      {
        for (int i = 0; i < modeSequence.size(); ++i)
        {
          std::cout << i << " time: " << ((i < modeSequence.size() - 1) ? std::to_string(eventTimes[i]) : " (last)") << " mode: " << std::to_string(static_cast<int>(modeSequence[i])) << std::endl;
        }
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime, bool clean_up)
    {
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &footPoseSequence = modeSchedule_.footPoseSequence;
      auto &torsoPoseSequence = modeSchedule_.torsoPoseSequence;
      auto &enableFootSequence = modeSchedule_.enableFootSequence;
      auto &isLastCommandSequence = modeSchedule_.isLastCommandSequence;
      auto &additionalFootPoseSequence = modeSchedule_.additionalFootPoseSequence;
      auto &enableFullBodySequence = modeSchedule_.enableFullBodySequence;
      auto &fullBodyStateSequence = modeSchedule_.fullBodyStateSequence;
      auto &timeTrajectorySequence = modeSchedule_.timeTrajectorySequence;
      auto &swingHeightSequence = modeSchedule_.swingHeightSequence;
      // modeSchedule_.with_pos_sequence = false;
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();

      if (index > 0 && clean_up)
      {
        // delete the old logic from index and set the default start phase to stance
        eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1); // keep the one before the last to make it stance
        modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);
        footPoseSequence.erase(footPoseSequence.begin(), footPoseSequence.begin() + index - 1);
        torsoPoseSequence.erase(torsoPoseSequence.begin(), torsoPoseSequence.begin() + index - 1);
        enableFootSequence.erase(enableFootSequence.begin(), enableFootSequence.begin() + index - 1);
        isLastCommandSequence.erase(isLastCommandSequence.begin(), isLastCommandSequence.begin() + index - 1);
        additionalFootPoseSequence.erase(additionalFootPoseSequence.begin(), additionalFootPoseSequence.begin() + index - 1);        
        enableFullBodySequence.erase(enableFullBodySequence.begin(), enableFullBodySequence.begin() + index - 1);
        swingHeightSequence.erase(swingHeightSequence.begin(), swingHeightSequence.begin() + index - 1);
        fullBodyStateSequence.erase(fullBodyStateSequence.begin(), fullBodyStateSequence.begin() + index - 1);
        timeTrajectorySequence.erase(timeTrajectorySequence.begin(), timeTrajectorySequence.begin() + index - 1);
        // set the default initial phase
        modeSequence.front() = ModeNumber::SS;
      }

      // Start tiling at time
      const auto tilingStartTime = eventTimes.empty() ? upperBoundTime : eventTimes.back();

      // delete the last default stance phase
      eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
      modeSequence.erase(modeSequence.end() - 1, modeSequence.end());
      footPoseSequence.erase(footPoseSequence.end() - 1, footPoseSequence.end());
      torsoPoseSequence.erase(torsoPoseSequence.end() - 1, torsoPoseSequence.end());
      enableFootSequence.erase(enableFootSequence.end() - 1, enableFootSequence.end());
      isLastCommandSequence.erase(isLastCommandSequence.end() - 1, isLastCommandSequence.end());
      additionalFootPoseSequence.erase(additionalFootPoseSequence.end() - 1, additionalFootPoseSequence.end());      
      enableFullBodySequence.erase(enableFullBodySequence.end() - 1, enableFullBodySequence.end());
      swingHeightSequence.erase(swingHeightSequence.end() - 1, swingHeightSequence.end());
      fullBodyStateSequence.erase(fullBodyStateSequence.end() - 1, fullBodyStateSequence.end());
      timeTrajectorySequence.erase(timeTrajectorySequence.end() - 1, timeTrajectorySequence.end());
      // tile the template logic
      tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
      // countSteps(1);
      ModeSchedule sub_mode_schedule;
      
      double new_upper_time =  (!modeSchedule_.existValidFootPose())? upperBoundTime:upperBoundTime+2;

      if (!modeSchedule_.getSubSchedule(lowerBoundTime, new_upper_time, sub_mode_schedule)) {
        std::cerr << "[GaitSchedule] Failed to get sub schedule for time range [" << lowerBoundTime << ", " << upperBoundTime << "]" << std::endl;
        currentModeSchedule_ = modeSchedule_;
        return modeSchedule_;
      }
      
      
     // 删除最后一个默认的 stance 相位
      if (!sub_mode_schedule.modeSequence.empty()) {
        sub_mode_schedule.modeSequence.pop_back();
        sub_mode_schedule.footPoseSequence.pop_back();
        sub_mode_schedule.torsoPoseSequence.pop_back();
        sub_mode_schedule.enableFootSequence.pop_back();
        sub_mode_schedule.additionalFootPoseSequence.pop_back();
        sub_mode_schedule.isLastCommandSequence.pop_back();
        sub_mode_schedule.enableFullBodySequence.pop_back();
        sub_mode_schedule.fullBodyStateSequence.pop_back();
        sub_mode_schedule.timeTrajectorySequence.pop_back();
        sub_mode_schedule.swingHeightSequence.pop_back();
      }

      // 添加新的默认 stance 相位
      sub_mode_schedule.modeSequence.push_back(ModeNumber::SS);
      sub_mode_schedule.footPoseSequence.push_back(0*vector6_t::Ones());
      sub_mode_schedule.torsoPoseSequence.push_back(0*vector6_t::Ones());
      sub_mode_schedule.enableFootSequence.push_back(false);
      sub_mode_schedule.additionalFootPoseSequence.push_back(std::vector<vector6_t>());
      sub_mode_schedule.isLastCommandSequence.push_back(false);
      sub_mode_schedule.enableFullBodySequence.push_back(false);
      sub_mode_schedule.fullBodyStateSequence.push_back(vector_array_t());
      sub_mode_schedule.timeTrajectorySequence.push_back(scalar_array_t(0));
      sub_mode_schedule.swingHeightSequence.push_back(0.06);
      currentModeSchedule_ = sub_mode_schedule;
      return currentModeSchedule_;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSchedule::countSteps(scalar_t currentTime)
    {      
      const auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;

      // find the index on which the current time is located
      size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), currentTime) - eventTimes.begin();
      char current_mode_left = modeNumber2String(modeSchedule_.modeSequence[index])[0];
      char current_mode_right = modeNumber2String(modeSchedule_.modeSequence[index])[1];
      // std::cout << "left:  " << prev_left_mode_ << ", " << current_mode_left << std::endl;
      // std::cout << "right: " << prev_right_mode_ << ", " << current_mode_right << std::endl;
      auto countSingleSteps = [&](char prev_mode, char current_mode)
      {
        if (prev_mode == 'F' && current_mode != 'F')
          ++total_steps_;
        // std::cout << "total_steps: " << total_steps_<< std::endl;

      };
      countSingleSteps(prev_left_mode_, current_mode_left);
      countSingleSteps(prev_right_mode_, current_mode_right);
      prev_left_mode_ = current_mode_left;
      prev_right_mode_ = current_mode_right;
      // trot步态由于是SF->FS，因此需要-1
      auto tmp_idx = (gaitTimeName_.second == "trot")?(stop_step_idx_-1):stop_step_idx_;
      if(total_steps_ == tmp_idx){
        for(int i=index+1; i<modeSequence.size(); ++i)
          modeSequence[i] = ModeNumber::SS;   
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)
    {
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &footPoseSequence = modeSchedule_.footPoseSequence;
      auto &torsoPoseSequence = modeSchedule_.torsoPoseSequence;
      auto &enableFootSequence = modeSchedule_.enableFootSequence;
      auto &isLastCommandSequence = modeSchedule_.isLastCommandSequence;
      auto &additionalFootPoseSequence = modeSchedule_.additionalFootPoseSequence;      
      auto &enableFullBodySequence = modeSchedule_.enableFullBodySequence;
      auto &fullBodyStateSequence = modeSchedule_.fullBodyStateSequence;
      auto &timeTrajectorySequence = modeSchedule_.timeTrajectorySequence;
      auto &swingHeightSequence = modeSchedule_.swingHeightSequence;

      const auto &templateTimes = modeSequenceTemplate_.switchingTimes;
      const auto &templateModeSequence = modeSequenceTemplate_.modeSequence;
      const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();

      // If no template subsystem is defined, the last subsystem should continue for ever
      if (numTemplateSubsystems == 0)
      {
        return;
      }

      if (!eventTimes.empty() && startTime <= eventTimes.back())
      {
        throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
      }

      // add a initial time
      eventTimes.push_back(startTime);

      // concatenate from index
      while (eventTimes.back() <= finalTime)
      {
        for (size_t i = 0; i < templateModeSequence.size(); i++)
        {
          modeSequence.push_back(templateModeSequence[i]);
          footPoseSequence.push_back(0*vector6_t::Ones());
          additionalFootPoseSequence.push_back(std::vector<vector6_t>());
          torsoPoseSequence.push_back(0*vector6_t::Ones());
          enableFootSequence.push_back(false);
          isLastCommandSequence.push_back(false);
          enableFullBodySequence.push_back(false);
          fullBodyStateSequence.push_back(vector_array_t());
          timeTrajectorySequence.push_back(scalar_array_t(0));
          swingHeightSequence.push_back(0.06);
          scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
          eventTimes.push_back(eventTimes.back() + deltaTime);
        } // end of i loop
      } // end of while loop

      // default final phase
      modeSequence.push_back(ModeNumber::SS);
      footPoseSequence.push_back(0*vector6_t::Ones());
      additionalFootPoseSequence.push_back(std::vector<vector6_t>());
      torsoPoseSequence.push_back(0*vector6_t::Ones());
      enableFootSequence.push_back(false);
      isLastCommandSequence.push_back(false);
      enableFullBodySequence.push_back(false);
      fullBodyStateSequence.push_back(vector_array_t());
      timeTrajectorySequence.push_back(scalar_array_t(0));
      swingHeightSequence.push_back(0.06);
    }

    ModeSchedule GaitSchedule::modifyModeFullBodySchedules(scalar_t currentTime, const Eigen::Vector4d &currentTorsoPose, 
                                                         const FullBodySchedule &fullBodySchedule, 
                                                         const feet_array_t<vector3_t> &foot_pos,
                                                         scalar_t startInsertTime)
    {
      const double end_time = modeSchedule_.eventTimes.back();
      const auto &templateTimes = modeSequenceTemplate_.switchingTimes;
      const double delta_time = templateTimes[1] - templateTimes[0];
      tileModeSequenceTemplate(end_time+0.01, end_time + fullBodySchedule.eventTimes.back() + delta_time * fullBodySchedule.eventTimes.size());
      std::cout << "[GaitSchedule] tile eventime: from " << end_time+0.01 << " to " << end_time + fullBodySchedule.eventTimes.back() + delta_time << std::endl;
      
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &fullBodyStateSequence = modeSchedule_.fullBodyStateSequence;
      auto &timeTrajectorySequence = modeSchedule_.timeTrajectorySequence;

      // 找到当前时间所在的索引
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), currentTime) - eventTimes.begin();

      // 如果指定了起始时间且比当前index时间还早，直接返回原序列
      if (startInsertTime > 0 && startInsertTime <= eventTimes[index+1]) {
        std::cout << "[GaitSchedule] ignore fullbody schedule!!!" << std::endl;
        std::cout << "startInsertTime" << startInsertTime<<std::endl;
        std::cout << "currentTime" << currentTime<<std::endl;
        std::cout << "eventTimes[index+1]" << eventTimes[index+1]<<std::endl;
        return getModeSchedule(currentTime-1, currentTime+2);
      }

      auto &enableFootSequence = modeSchedule_.enableFootSequence;
      auto &enableFullBodySequence = modeSchedule_.enableFullBodySequence;
      auto &footPoseSequence = modeSchedule_.footPoseSequence;
      auto &torsoPoseSequence = modeSchedule_.torsoPoseSequence;
      auto &swingHeightSequence = modeSchedule_.swingHeightSequence;
      auto &additionalFootPoseSequence = modeSchedule_.additionalFootPoseSequence; // cppcheck-suppress variableScope

      // 获取足部和躯干姿态信息
      const auto& foot_event_times = fullBodySchedule.eventTimes;
      const auto& foot_indices = fullBodySchedule.footIndices;
      std::cout << "[GaitSchedule] foot_event_times: " << foot_event_times.size() << std::endl;
      std::cout << "[GaitSchedule] eventTimes.size():"<<eventTimes.size()<<std::endl;
      std::cout << "[GaitSchedule] enableFullBodySequence.size(): " << enableFullBodySequence.size() << std::endl;
      std::cout << "[GaitSchedule] fullBodyStateSequence.size:"<<fullBodyStateSequence.size()<<std::endl;
      std::cout << "[GaitSchedule] fullBodySchedule.targetTrajectories.size:"<<fullBodySchedule.targetTrajectories.size()<<std::endl;

      int j = 0;
      double last_foot_event_time = 0.0;
      size_t i = index + 1;  // 从index后一个开始
      double mode_start_time;

      // 根据是否指定startInsertTime来决定起始时间
      if (startInsertTime > 0) {
        i = index;
        // 使用指定的起始时间
        mode_start_time = startInsertTime;
        
        // 计算需要插入的SS相数量
        double time_to_fill = startInsertTime - eventTimes[i-1];
        const double max_ss_interval = 0.3;  // 最大时间间隔
        int num_ss_phases = std::ceil(time_to_fill / max_ss_interval);
        double ss_interval = time_to_fill / num_ss_phases;  // 实际的时间间隔

        // 插入SS相
        double current_time = eventTimes[i-1];
        while (i < eventTimes.size() && current_time < startInsertTime) {
          current_time += ss_interval;
          eventTimes[i] = current_time;
          modeSequence[i] = ModeNumber::SS;
          enableFullBodySequence[i] = false;
          enableFootSequence[i] = false;
          footPoseSequence[i] = 0*vector6_t::Ones();
          torsoPoseSequence[i] = 0*vector6_t::Ones();
          additionalFootPoseSequence[i] = std::vector<vector6_t>();
          fullBodyStateSequence[i] = vector_array_t();
          timeTrajectorySequence[i] = scalar_array_t(0);
          swingHeightSequence[i] = 0.06;
          i++;
        }
      } else {
        // 使用原有逻辑查找起始时间
        mode_start_time = eventTimes[i - 1];
      }

      std::cout << "[GaitSchedule] mode_start_time: " << mode_start_time << std::endl;

      // 插入新的mode序列
      for (; i < eventTimes.size() && j < foot_event_times.size(); j++)
      {
        std::cout << "[" << j << "]: " << foot_event_times[j] << std::endl;
        auto foot_idx = foot_indices[j];
        double step_delta_time = foot_event_times[j] - last_foot_event_time;
        const auto ts = eventTimes[i - 1];
        eventTimes[i] = ts + step_delta_time;
        std::cout << "eventTimes[i]: " << eventTimes[i] << std::endl;
        modeSequence[i] = (foot_idx == FootIdx::Left) ? ModeNumber::FS : ((foot_idx == FootIdx::Right) ? ModeNumber::SF : ModeNumber::SS);
        std::cout << "modeSequence[" << i << "]: " << modeSequence[i] << std::endl;
        enableFullBodySequence[i] = true;
        last_foot_event_time = foot_event_times[j];
        fullBodyStateSequence[i] = fullBodySchedule.targetTrajectories[j].stateTrajectory;
        timeTrajectorySequence[i] = fullBodySchedule.targetTrajectories[j].timeTrajectory;
        for (int k = 0; k < timeTrajectorySequence[i].size(); k++)
        {
          timeTrajectorySequence[i][k] = timeTrajectorySequence[i][k] + mode_start_time;
        }
        i++;
      }

      std::cout << "[GaitSchedule] mode end time: " << eventTimes.back() << std::endl;
      // 刷新后续时间
      for (; i < eventTimes.size(); ++i)
      {
        eventTimes[i] = eventTimes[i - 1] + delta_time;
      }
      std::cout << "[GaitSchedule] eventTimes: " << eventTimes.size() << std::endl;
      // // modeSequence.push_back(ModeNumber::SS);
      // std::cout << "After modify mode schedule: \n";
      // for (int i = 0; i < modeSequence.size(); ++i)
      // {
      //   std::cout << "[" << eventTimes[i] << ":";
      //   std::cout << modeNumber2String(modeSequence[i]) << "] " << enableFootSequence[i] << " | ";
      // }
      // std::cout << std::endl;
      // std::cout << "footPoseSequence: \n";
      // for (int i = 0; i < footPoseSequence.size(); ++i)
      // {
      //   std::cout << footPoseSequence[i].transpose() << std::endl;
      // }
      // std::cout << "swingHeightSequence: \n";
      // for (int i = 0; i < swingHeightSequence.size(); ++i)
      // {
      //   std::cout << swingHeightSequence[i] << std::endl;
      // }
      // std::cout << "\ntorsoPoseSequence: \n";
      // for (int i = 0; i < torsoPoseSequence.size(); ++i)
      // {
      //   std::cout << torsoPoseSequence[i].transpose() << std::endl;
      // }
      // std::cout << "enableFootSequence: \n";
      // for (int i = 0; i < enableFootSequence.size(); ++i)
      // {
      //   std::cout << enableFootSequence[i] << std::endl;
      // }
      return getModeSchedule(currentTime-1, currentTime+2);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ModeSchedule GaitSchedule::modifyModePoseSchedules(scalar_t currentTime, const Eigen::Vector4d& currentTorsoPose, 
                                                      const FootPoseSchedule& footPoseSchedule, 
                                                      const feet_array_t<vector3_t> &foot_pos,
                                                      scalar_t startInsertTime, const TargetTrajectories &targetTrajectories, double terrain_height)
    {
      const auto &templateTimes = modeSequenceTemplate_.switchingTimes;
      const double delta_time = templateTimes[1] - templateTimes[0];
      {
        erase_index = -1;
        const size_t end_index = std::lower_bound(modeSchedule_.eventTimes.begin(), modeSchedule_.eventTimes.end(), currentTime) - modeSchedule_.eventTimes.begin();
        for(size_t i = end_index; i < modeSchedule_.eventTimes.size(); ++i)
        {
          if((modeSchedule_.modeSequence[i] == ModeNumber::SS && std::abs(modeSchedule_.eventTimes[i] - currentTime)>0.3))
          {
            erase_index = i;
            modeSchedule_.eventTimes.erase(modeSchedule_.eventTimes.begin() + erase_index+1, modeSchedule_.eventTimes.end());
            modeSchedule_.modeSequence.erase(modeSchedule_.modeSequence.begin() + erase_index+1, modeSchedule_.modeSequence.end());
            modeSchedule_.footPoseSequence.erase(modeSchedule_.footPoseSequence.begin() + erase_index+1, modeSchedule_.footPoseSequence.end());
            modeSchedule_.torsoPoseSequence.erase(modeSchedule_.torsoPoseSequence.begin() + erase_index+1, modeSchedule_.torsoPoseSequence.end());
            modeSchedule_.enableFootSequence.erase(modeSchedule_.enableFootSequence.begin() + erase_index+1, modeSchedule_.enableFootSequence.end());
            modeSchedule_.isLastCommandSequence.erase(modeSchedule_.isLastCommandSequence.begin() + erase_index+1, modeSchedule_.isLastCommandSequence.end());
            modeSchedule_.enableFullBodySequence.erase(modeSchedule_.enableFullBodySequence.begin() + erase_index+1, modeSchedule_.enableFullBodySequence.end());
            modeSchedule_.fullBodyStateSequence.erase(modeSchedule_.fullBodyStateSequence.begin() + erase_index+1, modeSchedule_.fullBodyStateSequence.end());
            modeSchedule_.timeTrajectorySequence.erase(modeSchedule_.timeTrajectorySequence.begin() + erase_index+1, modeSchedule_.timeTrajectorySequence.end());
            modeSchedule_.swingHeightSequence.erase(modeSchedule_.swingHeightSequence.begin() + erase_index+1, modeSchedule_.swingHeightSequence.end());
            modeSchedule_.additionalFootPoseSequence.erase(modeSchedule_.additionalFootPoseSequence.begin() + erase_index+1, modeSchedule_.additionalFootPoseSequence.end());

            for(int p=0;p<modeSchedule_.isLastCommandSequence.size(); ++p)
            {
                modeSchedule_.isLastCommandSequence[p] = true;
            }

            modeSchedule_.modeSequence.push_back(ModeNumber::SS);
            modeSchedule_.footPoseSequence.push_back(0*vector6_t::Ones());
            modeSchedule_.torsoPoseSequence.push_back(0*vector6_t::Ones());
            modeSchedule_.enableFootSequence.push_back(false);
            modeSchedule_.isLastCommandSequence.push_back(false);
            modeSchedule_.enableFullBodySequence.push_back(false);
            modeSchedule_.fullBodyStateSequence.push_back(vector_array_t());
            modeSchedule_.timeTrajectorySequence.push_back(scalar_array_t(0));
            modeSchedule_.swingHeightSequence.push_back(0.06);
            modeSchedule_.additionalFootPoseSequence.push_back(std::vector<vector6_t>());
            break;
          }
        }
        if (erase_index == -1)
          return modeSchedule_;
          
        tileModeSequenceTemplate(modeSchedule_.eventTimes.back()+0.01, modeSchedule_.eventTimes.back() + delta_time*footPoseSchedule.eventTimes.size()*2);
      }
      
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &enableFootSequence = modeSchedule_.enableFootSequence;
      auto &isLastCommandSequence = modeSchedule_.isLastCommandSequence;
      auto &footPoseSequence = modeSchedule_.footPoseSequence;
      auto &torsoPoseSequence = modeSchedule_.torsoPoseSequence;
      auto &swingHeightSequence = modeSchedule_.swingHeightSequence;
      auto &additionalFootPoseSequence = modeSchedule_.additionalFootPoseSequence;

      // 找到当前时间所在的索引
      size_t index = erase_index+1;

      // 如果指定了起始时间且比当前index时间还早，直接返回原序列
      if (startInsertTime > 0 && startInsertTime <= eventTimes[index+1]) {
        std::cout << "[GaitSchedule] ignore footpose schedule!!!" << std::endl;
        std::cout << "[GaitSchedule] startInsertTime: " << startInsertTime << std::endl;
        std::cout << "[GaitSchedule] currentTime: " << currentTime << std::endl;
        std::cout << "[GaitSchedule] eventTimes[index+1]: " << eventTimes[index+1] << std::endl;
        return getModeSchedule(currentTime-1, currentTime+2);
      }
    const auto &additional_foot_pose = footPoseSchedule.additionalFootPoseSequence;

      std::cout << "Before modify mode schedule: " << currentTime << "\n";
      for (int i = 0; i < modeSequence.size(); ++i)
      {
        std::cout << "[" << eventTimes[i] << ":";
        std::cout << modeNumber2String(modeSequence[i]) << "] ";
      }
      std::cout << std::endl;
    Eigen::Vector4d currentTorsoPose_modify;
    if (targetTrajectories.empty())// 没有传递targetTrajectories，需要和旧的实现兼容
    {
      currentTorsoPose_modify = currentTorsoPose;
    }
    else
    {
      currentTorsoPose_modify = Eigen::Vector4d(targetTrajectories.getDesiredState(eventTimes[index]).segment<4>(6));
    }
    const Eigen::Matrix<scalar_t, 3, 1> currentZyx = {currentTorsoPose_modify(3), 0, 0};
    // const Eigen::Matrix<scalar_t, 3, 1> currentZyx = {currentTorsoPose(3), 0, 0};
    const auto Rot = getRotationMatrixFromZyxEulerAngles(currentZyx);
    auto get_foot_pose_world = [&currentTorsoPose_modify, &Rot, &terrain_height](const vector6_t& foot_pose)
    {
      vector6_t foot_pose_world;
      foot_pose_world.head<2>() = currentTorsoPose_modify.head<2>() + (Rot*foot_pose.head<3>()).head<2>();
      foot_pose_world(2) = foot_pose(2) + terrain_height;
      foot_pose_world(3) = currentTorsoPose_modify(3) + foot_pose(3);
      foot_pose_world(4) = foot_pose(4); // pitch
      foot_pose_world(5) = foot_pose(5); // roll
      return foot_pose_world;
    };
    // auto get_foot_pose_world = [&currentTorsoPose, &Rot](const Eigen::Vector4d& foot_pose)
    // {
    //   Eigen::Vector4d foot_pose_world;
    //   foot_pose_world.head<2>() = currentTorsoPose.head<2>() + (Rot*foot_pose.head<3>()).head<2>();
    //   foot_pose_world(2) = foot_pose(2);
    //   foot_pose_world(3) = currentTorsoPose(3) + foot_pose(3);
    //   return foot_pose_world;
    // };
    int j = 0;
    double last_foot_event_time = 0.0;
    size_t i = index + 1;
    // time check[important!!!]
    double insert_point_delta_time = eventTimes[i] - currentTime;
    while(insert_point_delta_time < 0.8)
    {
      ++i;
      insert_point_delta_time = eventTimes[i] - currentTime;
    }
    std::cout << "insert_point_delta_time: " << insert_point_delta_time << std::endl;

    // 如果指定了起始时间，插入SS相直到起始时间
    if (startInsertTime > 0) {
      i = index;
      // 计算需要插入的SS相数量
      double time_to_fill = startInsertTime - eventTimes[i-1];
      const double max_ss_interval = 0.3;  // 最大时间间隔
      int num_ss_phases = std::ceil(time_to_fill / max_ss_interval);
      double ss_interval = time_to_fill / num_ss_phases;  // 实际的时间间隔

      // 插入SS相
      double current_time = eventTimes[i-1];
      while (i < eventTimes.size() && current_time < startInsertTime) {
        current_time += ss_interval;
        eventTimes[i] = current_time;
        modeSequence[i] = ModeNumber::SS;
        enableFootSequence[i] = false;
        footPoseSequence[i] = 0*vector6_t::Ones();
        torsoPoseSequence[i] = 0*vector6_t::Ones();
        swingHeightSequence[i] = 0.06;
        i++;
      }
    }

    // 获取足部和躯干姿态信息
    const auto& foot_event_times = footPoseSchedule.eventTimes;
    const auto& foot_indices = footPoseSchedule.footIndices;
    const auto& foot_pose = footPoseSchedule.footPoseSequence;
    const auto& torso_pose = footPoseSchedule.torsoPoseSequence;
    const auto& swing_height = footPoseSchedule.swingHeightSequence;
    bool update_swing_height = true;
    if (swing_height.size() == 0 || swing_height.size() != foot_indices.size())
    {
      update_swing_height = false;
      std::cout << "[GaitSchedule] swing_height size is not defined, default to 0.06m" << std::endl;
    }
    
    for (; i < eventTimes.size() && j < foot_event_times.size(); j++) {
      // SS作为初始状态
      auto foot_idx = foot_indices[j];
      // double last_time = eventTimes[i-1];
      double step_delta_time = foot_event_times[j] - last_foot_event_time;
      const auto ts = eventTimes[i-1];
      eventTimes[i] = ts + step_delta_time;
      modeSequence[i] = footIdxToNum[foot_idx];
      std::cout << "i: " << i << " modeSequence[i]: " << modeSequence[i] << std::endl;
      last_foot_event_time = foot_event_times[j];
      enableFootSequence[i] = true;
      swingHeightSequence[i] = (update_swing_height)? swing_height[j] : 0.06;

      if (foot_idx == FootIdx::Left || foot_idx == FootIdx::Right){// 当前是腾空相，落点位置无效，躯干位置有效，如果是连续腾空，增加落点判断
        torsoPoseSequence[i] = torso_pose[j];
        bool next_foot_idx_is_stance = j < foot_event_times.size() -1 && foot_indices[j+1] != FootIdx::Left && foot_indices[j+1] != FootIdx::Right;// 下一个指定mode**也**是SS
        bool last_foot_idx_is_not_stance = j == foot_event_times.size() -1 && foot_indices[j] != FootIdx::Stance;// 最后一个指定mode不是SS
        bool has_duplicate_swing_leg = !next_foot_idx_is_stance;// 存在重复的腾空相
        // bool has_duplicate_swing_leg =  j == foot_event_times.size() -1 && (foot_indices[j] != FootIdx::Stance && foot_indices[j-1] != FootIdx::Stance);
        std::cout << "has_duplicate_swing_leg: " << has_duplicate_swing_leg << std::endl;
        std::cout << "next_foot_idx_is_stance: " << next_foot_idx_is_stance << std::endl;
        std::cout << "last_foot_idx_is_not_stance: " << last_foot_idx_is_not_stance << std::endl;
        bool enable_auto_insert_stance = !numerics::almost_eq(default_stance_duration_, 0.0);
        // has_duplicate_swing_leg = has_duplicate_swing_leg && enable_auto_insert_stance;
        std::cout << "enable_auto_insert_stance: " << enable_auto_insert_stance << std::endl;
        // std::cout << "has_duplicate_swing_leg: " << has_duplicate_swing_leg << std::endl;
        if (!enable_auto_insert_stance){
          footPoseSequence[i] = get_foot_pose_world(foot_pose[j]);
          if (additional_foot_pose.size() > 0)
          {
            additionalFootPoseSequence[i].clear();
            std::cout << "s1 additional_foot_pose["<<j<<"].size(): " << additional_foot_pose[j].size() << std::endl;
            for (int k = 0; k < additional_foot_pose[j].size(); ++k)
            {
              vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[j][k]);
              std::cout << "s1 adding foot_pose_world:"<<foot_pose_world.transpose()<<std::endl;
              additionalFootPoseSequence[i].push_back(foot_pose_world);
            }
          }
        }
        
        if (!next_foot_idx_is_stance || last_foot_idx_is_not_stance)// 下一个没有指定stance || 存在重复的腾空相 || 最后一个指定mode不是SS，插入一个落点位置
        {
          i++;
          enableFootSequence[i-1] = false;
          enableFootSequence[i] = true;
          footPoseSequence[i] = get_foot_pose_world(foot_pose[j]);
          swingHeightSequence[i] = (update_swing_height)? swing_height[j] : 0.06;
          if (additional_foot_pose.size() > 0)
          {
            additionalFootPoseSequence[i].clear();
            std::cout << "s2 additional_foot_pose["<<j<<"].size(): " << additional_foot_pose[j].size() << std::endl;
            for (int k = 0; k < additional_foot_pose[j].size(); ++k)
            {
              vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[j][k]);
              std::cout << "s2 adding foot_pose_world:"<<foot_pose_world.transpose()<<std::endl;
              additionalFootPoseSequence[i].push_back(foot_pose_world);
            }
          }
          torsoPoseSequence[i] = torso_pose[j];
          modeSequence[i] = ModeNumber::SS;
          eventTimes[i] = ts + step_delta_time + ((has_duplicate_swing_leg)? default_stance_duration_ : 0.4);
          if (!has_duplicate_swing_leg)
          {
            std::cerr << "[GaitSchedule] foot indices has duplicate values." << std::to_string(static_cast<int>(foot_indices[j])) << std::endl;

            std::cerr << "[GaitSchedule] insert a stance phase at time: " << eventTimes[i] << std::endl;
          }
          else
          {
            std::cout << "[GaitSchedule] insert a default stance phase at time: " << eventTimes[i] << std::endl;
          }
          i++;
        }
        else {//没有插入SS相
          footPoseSequence[i] = get_foot_pose_world(foot_pose[j]);
          swingHeightSequence[i] = (update_swing_height)? swing_height[j] : 0.06;
          if (additional_foot_pose.size() > 0 && j > 0)
          {
            additionalFootPoseSequence[i].clear();
            std::cout << "ss additional_foot_pose["<<j<<"].size(): " << additional_foot_pose[j].size() << std::endl;
            for (int k = 0; k < additional_foot_pose[j].size(); ++k)
            {
              vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[j][k]);
              std::cout << "ss adding foot_pose_world:"<<foot_pose_world.transpose()<<std::endl;
              additionalFootPoseSequence[i].push_back(foot_pose_world);
            }
          }
          torsoPoseSequence[i] = torso_pose[j];
          i++;
        }

      } 
      else {//FootIdx::Stance，ST和SH等站立情况下，直接插入SS相的落足位置和躯干位置
        auto select_index = (foot_idx == FootIdx::Stance)? j-1 : j;// 站立的时候取前一个落足位置，其他情况取当前落足位置
        footPoseSequence[i] = get_foot_pose_world(foot_pose[select_index]);
        swingHeightSequence[i] = (update_swing_height)? swing_height[j] : 0.06;
        if (additional_foot_pose.size() > 0 && j > 0)
        {
          additionalFootPoseSequence[i].clear();
          std::cout << "ss additional_foot_pose["<<select_index<<"].size(): " << additional_foot_pose[select_index].size() << std::endl;
          for (int k = 0; k < additional_foot_pose[select_index].size(); ++k)
          {
            vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[select_index][k]);
            std::cout << "ss adding foot_pose_world:"<<foot_pose_world.transpose()<<std::endl;
            additionalFootPoseSequence[i].push_back(foot_pose_world);
          }
        }
        torsoPoseSequence[i] = torso_pose[j];
        i++;
      }
    }
    // 刷新后续时间
    for(; i < eventTimes.size(); ++i)
    {
      eventTimes[i] = eventTimes[i-1] + delta_time;
    }
    // modeSequence.push_back(ModeNumber::SS);
    std::cout << "After modify mode schedule: \n";

    for (int i = 0; i < footPoseSequence.size() && i < eventTimes.size(); ++i) {
      std::cout << "t["<<i<<"]:" << eventTimes[i] << " mode: " << modeNumber2String(modeSequence[i]) << "\nfootPose: " << footPoseSequence[i].transpose() 
      << " torsoPose: " << torsoPoseSequence[i].transpose() << " enable: " << enableFootSequence[i] << "  islast: "<<isLastCommandSequence[i]<< std::endl;
    }
    
    return modeSchedule_;
    }

    ModeSchedule GaitSchedule::modifyModeWorldPoseSchedules(scalar_t currentTime, const Eigen::Vector4d &currentTorsoPose,
                                                            const FootPoseSchedule &footPoseSchedule,
                                                            const feet_array_t<vector3_t> &foot_pos,
                                                            scalar_t startInsertTime, 
                                                            const TargetTrajectories &targetTrajectories,
                                                            scalar_t insert_time)
    {
      bool is_continuous_mode_equal = false;
      const auto &templateTimes = modeSequenceTemplate_.switchingTimes;
      const double delta_time = templateTimes[1] - templateTimes[0];

      erase_index = -1;
      const size_t end_index = std::lower_bound(modeSchedule_.eventTimes.begin(), modeSchedule_.eventTimes.end(), currentTime) - modeSchedule_.eventTimes.begin();
     
      size_t insert_index = -1;
      if (insert_time > 0){ // 如果指定了插入时间，计算插入索引
        insert_index = std::lower_bound(modeSchedule_.eventTimes.begin(), modeSchedule_.eventTimes.end(), insert_time) - modeSchedule_.eventTimes.begin();
      }
     
      bool modeSchedule_all_stance = true;  // cppcheck-suppress variableScope

      for (size_t i = 0; i < modeSchedule_.modeSequence.size(); ++i)
      {
        if (modeSchedule_.modeSequence[i] != ModeNumber::SS)
        {
          modeSchedule_all_stance = false;
          break;
        }
      }
     
      // fix cppcheck warning
      bool modeSchedule_all_stance_after = true;
      size_t all_stance_after_index = end_index;
      if (insert_index != -1)
        all_stance_after_index = insert_index;

      for (size_t i = all_stance_after_index; i < modeSchedule_.modeSequence.size(); ++i)
      {
        if (modeSchedule_.modeSequence[i] != ModeNumber::SS)
        {
          modeSchedule_all_stance_after = false;
          break;
        }
      }

      for (size_t i = end_index; i < modeSchedule_.eventTimes.size(); ++i)
      {
        if ((insert_index != -1) || (modeSchedule_.modeSequence[i] == ModeNumber::SS && std::abs(modeSchedule_.eventTimes[i] - currentTime) > 0.3))
        {
          if (insert_index != -1) // 指定了插入时间
          {
            if (modeSchedule_.modeSequence[insert_index] == ModeNumber::SS && ((modeSchedule_.footPoseSequence[insert_index][0] != 0 && modeSchedule_.footPoseSequence[insert_index][2] != 0) || modeSchedule_all_stance_after))
            {
              erase_index = insert_index;
              std::cout << "erase_index = insert_index" << std::endl;
            }
            else if (modeSchedule_.modeSequence[insert_index - 1] == ModeNumber::SS && modeSchedule_.footPoseSequence[insert_index-1][0]!=0 && modeSchedule_.footPoseSequence[insert_index-1][2]!=0) // 由于insert_time是double类型可能存在精度误差，insert_index可能是SS的下一个
            {  
              erase_index = insert_index - 1;
              std::cout<<"erase_index = insert_index - 1"<<std::endl;
            } 
            else // 插入时间点不合法
              return modeSchedule_;
          }
          else // 没有指定插入时间
          {
            erase_index = i;
          }

          if ((insert_index != -1) && ((modeSchedule_.modeSequence[erase_index - 1] == ModeNumber::SF && footPoseSchedule.footIndices[0] == FootIdx::Right) || (modeSchedule_.modeSequence[erase_index - 1] == ModeNumber::FS && footPoseSchedule.footIndices[0] == FootIdx::Left)))
          {
            is_continuous_mode_equal = true; // 连续上楼梯时新插入摆动相与前一摆动相是同一只脚
          }

          modeSchedule_.eventTimes.erase(modeSchedule_.eventTimes.begin() + erase_index + 1, modeSchedule_.eventTimes.end());
          modeSchedule_.modeSequence.erase(modeSchedule_.modeSequence.begin() + erase_index + 1, modeSchedule_.modeSequence.end());
          modeSchedule_.footPoseSequence.erase(modeSchedule_.footPoseSequence.begin() + erase_index + 1, modeSchedule_.footPoseSequence.end());
          modeSchedule_.torsoPoseSequence.erase(modeSchedule_.torsoPoseSequence.begin() + erase_index + 1, modeSchedule_.torsoPoseSequence.end());
          modeSchedule_.enableFootSequence.erase(modeSchedule_.enableFootSequence.begin() + erase_index + 1, modeSchedule_.enableFootSequence.end());
          modeSchedule_.isLastCommandSequence.erase(modeSchedule_.isLastCommandSequence.begin() + erase_index + 1, modeSchedule_.isLastCommandSequence.end());
          modeSchedule_.enableFullBodySequence.erase(modeSchedule_.enableFullBodySequence.begin() + erase_index + 1, modeSchedule_.enableFullBodySequence.end());
          modeSchedule_.fullBodyStateSequence.erase(modeSchedule_.fullBodyStateSequence.begin() + erase_index + 1, modeSchedule_.fullBodyStateSequence.end());
          modeSchedule_.timeTrajectorySequence.erase(modeSchedule_.timeTrajectorySequence.begin() + erase_index + 1, modeSchedule_.timeTrajectorySequence.end());
          modeSchedule_.swingHeightSequence.erase(modeSchedule_.swingHeightSequence.begin() + erase_index + 1, modeSchedule_.swingHeightSequence.end());
          modeSchedule_.additionalFootPoseSequence.erase(modeSchedule_.additionalFootPoseSequence.begin() + erase_index + 1, modeSchedule_.additionalFootPoseSequence.end());

          for (int p = 0; p < modeSchedule_.isLastCommandSequence.size(); ++p)
          {
            modeSchedule_.isLastCommandSequence[p] = true;
          }

          modeSchedule_.modeSequence.push_back(ModeNumber::SS);
          modeSchedule_.footPoseSequence.push_back(0 * vector6_t::Ones());
          modeSchedule_.torsoPoseSequence.push_back(0 * vector6_t::Ones());
          modeSchedule_.enableFootSequence.push_back(false);
          modeSchedule_.isLastCommandSequence.push_back(false);
          modeSchedule_.enableFullBodySequence.push_back(false);
          modeSchedule_.fullBodyStateSequence.push_back(vector_array_t());
          modeSchedule_.timeTrajectorySequence.push_back(scalar_array_t(0));
          modeSchedule_.swingHeightSequence.push_back(0.06);
          modeSchedule_.additionalFootPoseSequence.push_back(std::vector<vector6_t>());
          break;
        }
      }

      if (erase_index == -1)
        return modeSchedule_;

      tileModeSequenceTemplate(modeSchedule_.eventTimes.back() + 0.01, modeSchedule_.eventTimes.back() + delta_time * footPoseSchedule.eventTimes.size() * 2);

      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      auto &enableFootSequence = modeSchedule_.enableFootSequence;
      auto &isLastCommandSequence = modeSchedule_.isLastCommandSequence;
      auto &footPoseSequence = modeSchedule_.footPoseSequence;
      auto &torsoPoseSequence = modeSchedule_.torsoPoseSequence;
      auto &swingHeightSequence = modeSchedule_.swingHeightSequence;
      auto &additionalFootPoseSequence = modeSchedule_.additionalFootPoseSequence;

      // 找到当前时间所在的索引
      size_t index = erase_index + 1;

      // 如果指定了起始时间且比当前index时间还早，直接返回原序列
      if (startInsertTime > 0 && startInsertTime <= eventTimes[index + 1])
      {
        std::cout << "[GaitSchedule] ignore footpose schedule!!!" << std::endl;
        std::cout << "[GaitSchedule] startInsertTime: " << startInsertTime << std::endl;
        std::cout << "[GaitSchedule] currentTime: " << currentTime << std::endl;
        std::cout << "[GaitSchedule] eventTimes[index+1]: " << eventTimes[index + 1] << std::endl;
        return getModeSchedule(currentTime - 1, currentTime + 2);
      }
      const auto &additional_foot_pose = footPoseSchedule.additionalFootPoseSequence;

      std::cout << "Before modify mode schedule: " << currentTime << "\n";
      for (int i = 0; i < modeSequence.size(); ++i)
      {
        std::cout << "[" << eventTimes[i] << ":";
        std::cout << modeNumber2String(modeSequence[i]) << "] ";
      }
      std::cout << std::endl;

      auto currentTorsoPose_modify = Eigen::Vector4d(targetTrajectories.getDesiredState(eventTimes[index]).segment<4>(6));
      const Eigen::Matrix<scalar_t, 3, 1> currentZyx = {currentTorsoPose_modify(3), 0, 0};
      // const Eigen::Matrix<scalar_t, 3, 1> currentZyx = {currentTorsoPose(3), 0, 0};
      const auto Rot = getRotationMatrixFromZyxEulerAngles(currentZyx);
      auto lastFootHeight = footPoseSequence[erase_index][2];
      auto get_foot_pose_world = [&is_continuous_mode_equal, &currentTorsoPose_modify, &Rot, &lastFootHeight](const vector6_t &foot_pose)
      {
        vector6_t foot_pose_world;
        // foot_pose_world.head<2>() = currentTorsoPose_modify.head<2>() + (Rot*foot_pose.head<3>()).head<2>();
        // foot_pose_world(2) = lastFootHeight + foot_pose(2);
        vector3_t modify_y_foot_pose = foot_pose.head<3>();
        if (is_continuous_mode_equal)
          modify_y_foot_pose(1) = -modify_y_foot_pose(1);
        foot_pose_world.head<2>() = modify_y_foot_pose.head<2>();
        // foot_pose_world.head<2>() = (Rot * modify_y_foot_pose).head<2>();
        foot_pose_world(2) = foot_pose(2);
        foot_pose_world(3) = foot_pose(3);
        foot_pose_world(4) = foot_pose(4); // pitch
        foot_pose_world(5) = foot_pose(5); // roll
        return foot_pose_world;
      };
      // auto get_foot_pose_world = [&currentTorsoPose, &Rot](const Eigen::Vector4d& foot_pose)
      // {
      //   Eigen::Vector4d foot_pose_world;
      //   foot_pose_world.head<2>() = currentTorsoPose.head<2>() + (Rot*foot_pose.head<3>()).head<2>();
      //   foot_pose_world(2) = foot_pose(2);
      //   foot_pose_world(3) = currentTorsoPose(3) + foot_pose(3);
      //   return foot_pose_world;
      // };
      int j = 0;
      double last_foot_event_time = 0.0;
      size_t i = index + 1;
      // time check[important!!!]
      double insert_point_delta_time = eventTimes[i] - currentTime;
      // while (insert_point_delta_time < 0.8)
      // {
      //   ++i;
      //   insert_point_delta_time = eventTimes[i] - currentTime;
      //   std::cout<<"qwewqewqeewqeqweqweqweweeqweweqew"<<std::endl;
      // }
      std::cout << "insert_point_delta_time: " << insert_point_delta_time << std::endl;

      // 如果指定了起始时间，插入SS相直到起始时间
      if (startInsertTime > 0)
      {
        i = index;
        // 计算需要插入的SS相数量
        double time_to_fill = startInsertTime - eventTimes[i - 1];
        const double max_ss_interval = 0.3; // 最大时间间隔
        int num_ss_phases = std::ceil(time_to_fill / max_ss_interval);
        double ss_interval = time_to_fill / num_ss_phases; // 实际的时间间隔

        // 插入SS相
        double current_time = eventTimes[i - 1];
        while (i < eventTimes.size() && current_time < startInsertTime)
        {
          current_time += ss_interval;
          eventTimes[i] = current_time;
          modeSequence[i] = ModeNumber::SS;
          enableFootSequence[i] = false;
          footPoseSequence[i] = 0 * vector6_t::Ones();
          torsoPoseSequence[i] = 0 * vector6_t::Ones();
          swingHeightSequence[i] = 0.06;
          i++;
        }
      }

      // 获取足部和躯干姿态信息
      const auto &foot_event_times = footPoseSchedule.eventTimes;
      const auto &foot_indices = footPoseSchedule.footIndices;
      const auto &foot_pose = footPoseSchedule.footPoseSequence;
      const auto &torso_pose = footPoseSchedule.torsoPoseSequence;
      const auto &swing_height = footPoseSchedule.swingHeightSequence;
      bool update_swing_height = true;
      if (swing_height.size() == 0 || swing_height.size() != foot_indices.size())
      {
        update_swing_height = false;
        std::cout << "[GaitSchedule] swing_height size is not defined, default to 0.06m" << std::endl;
      }

      for (; i < eventTimes.size() && j < foot_event_times.size(); j++)
      {
        // SS作为初始状态
        auto foot_idx = foot_indices[j];
        if (is_continuous_mode_equal)
        {
          if (foot_idx == FootIdx::Left)
          {
            foot_idx = FootIdx::Right;
          }
          else if (foot_idx == FootIdx::Right)
          {
            foot_idx = FootIdx::Left;
          }
        }
        // double last_time = eventTimes[i-1];
        double step_delta_time = foot_event_times[j] - last_foot_event_time;
        const auto ts = eventTimes[i - 1];
        eventTimes[i] = ts + step_delta_time;
        modeSequence[i] = (foot_idx == FootIdx::Left) ? ModeNumber::FS : ((foot_idx == FootIdx::Right) ? ModeNumber::SF : ModeNumber::SS);
        std::cout << "i: " << i << " modeSequence[i]: " << modeSequence[i] << std::endl;
        last_foot_event_time = foot_event_times[j];
        enableFootSequence[i] = true;
        swingHeightSequence[i] = (update_swing_height) ? swing_height[j] : 0.06;

        if (foot_idx != FootIdx::Stance)
        {
          torsoPoseSequence[i] = torso_pose[j];
          bool next_foot_idx_is_stance = j < foot_event_times.size() - 1 && foot_indices[j + 1] == FootIdx::Stance;  // 下一个指定mode**也**是SS
          bool last_foot_idx_is_not_stance = j == foot_event_times.size() - 1 && foot_indices[j] != FootIdx::Stance; // 最后一个指定mode不是SS
          bool has_duplicate_swing_leg = !next_foot_idx_is_stance;                                                   // 存在重复的腾空相
          // bool has_duplicate_swing_leg =  j == foot_event_times.size() -1 && (foot_indices[j] != FootIdx::Stance && foot_indices[j-1] != FootIdx::Stance);
          std::cout << "has_duplicate_swing_leg: " << has_duplicate_swing_leg << std::endl;
          std::cout << "next_foot_idx_is_stance: " << next_foot_idx_is_stance << std::endl;
          std::cout << "last_foot_idx_is_not_stance: " << last_foot_idx_is_not_stance << std::endl;
          bool enable_auto_insert_stance = !numerics::almost_eq(default_stance_duration_, 0.0);
          // has_duplicate_swing_leg = has_duplicate_swing_leg && enable_auto_insert_stance;
          std::cout << "enable_auto_insert_stance: " << enable_auto_insert_stance << std::endl;
          // std::cout << "has_duplicate_swing_leg: " << has_duplicate_swing_leg << std::endl;
          if (!enable_auto_insert_stance)
          {
            footPoseSequence[i] = get_foot_pose_world(foot_pose[j]);
            if (additional_foot_pose.size() > 0)
            {
              additionalFootPoseSequence[i].clear();
              std::cout << "s1 additional_foot_pose[" << j << "].size(): " << additional_foot_pose[j].size() << std::endl;
              for (int k = 0; k < additional_foot_pose[j].size(); ++k)
              {
                vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[j][k]);
                std::cout << "s1 adding foot_pose_world:" << foot_pose_world.transpose() << std::endl;
                additionalFootPoseSequence[i].push_back(foot_pose_world);
              }
            }
          }
          i++;
          if (!next_foot_idx_is_stance || last_foot_idx_is_not_stance) // 下一个没有指定stance || 存在重复的腾空相 || 最后一个指定mode不是SS
          {
            enableFootSequence[i - 1] = false;
            enableFootSequence[i] = true;
            footPoseSequence[i] = get_foot_pose_world(foot_pose[j]);
            swingHeightSequence[i] = (update_swing_height) ? swing_height[j] : 0.06;
            if (additional_foot_pose.size() > 0)
            {
              additionalFootPoseSequence[i].clear();
              std::cout << "s2 additional_foot_pose[" << j << "].size(): " << additional_foot_pose[j].size() << std::endl;
              for (int k = 0; k < additional_foot_pose[j].size(); ++k)
              {
                vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[j][k]);
                std::cout << "s2 adding foot_pose_world:" << foot_pose_world.transpose() << std::endl;
                additionalFootPoseSequence[i].push_back(foot_pose_world);
              }
            }
            torsoPoseSequence[i] = torso_pose[j];
            modeSequence[i] = ModeNumber::SS;
            eventTimes[i] = ts + step_delta_time + ((has_duplicate_swing_leg) ? default_stance_duration_ : 0.4);
            if (!has_duplicate_swing_leg)
            {
              std::cerr << "[GaitSchedule] foot indices has duplicate values." << std::to_string(static_cast<int>(foot_indices[j])) << std::endl;

              std::cerr << "[GaitSchedule] insert a stance phase at time: " << eventTimes[i] << std::endl;
            }
            else
            {
              std::cout << "[GaitSchedule] insert a default stance phase at time: " << eventTimes[i] << std::endl;
            }
            i++;
          }
        }
        else
        {
          footPoseSequence[i] = get_foot_pose_world(foot_pose[j - 1]);
          swingHeightSequence[i] = (update_swing_height) ? swing_height[j] : 0.06;
          if (additional_foot_pose.size() > 0 && j > 0)
          {
            additionalFootPoseSequence[i].clear();
            std::cout << "ss additional_foot_pose[" << j - 1 << "].size(): " << additional_foot_pose[j - 1].size() << std::endl;
            for (int k = 0; k < additional_foot_pose[j - 1].size(); ++k)
            {
              vector6_t foot_pose_world = get_foot_pose_world(additional_foot_pose[j - 1][k]);
              std::cout << "ss adding foot_pose_world:" << foot_pose_world.transpose() << std::endl;
              additionalFootPoseSequence[i].push_back(foot_pose_world);
            }
          }
          torsoPoseSequence[i] = torso_pose[j];
          i++;
        }
      }
      // 刷新后续时间
      for (; i < eventTimes.size(); ++i)
      {
        eventTimes[i] = eventTimes[i - 1] + delta_time;
      }
      // modeSequence.push_back(ModeNumber::SS);
      std::cout << "After modify mode schedule: \n";

      for (int i = 0; i < footPoseSequence.size() && i < eventTimes.size(); ++i)
      {
        std::cout << "t[" << i << "]:" << eventTimes[i] << " mode: " << modeNumber2String(modeSequence[i]) << "\nfootPose: " << footPoseSequence[i].transpose()
                  << " torsoPose: " << torsoPoseSequence[i].transpose() << " enable: " << enableFootSequence[i] << "  islast: " << isLastCommandSequence[i] << std::endl;
      }

      return modeSchedule_;
    }
  } // namespace humanoid
} // namespace ocs2
