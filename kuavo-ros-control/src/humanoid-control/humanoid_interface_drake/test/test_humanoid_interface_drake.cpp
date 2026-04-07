#include "humanoid_interface_drake/humanoid_interface_drake.h"

using namespace HighlyDynamic;
int main(int argc, char **argv)
{

    const char* version_env = std::getenv("ROBOT_VERSION");
    if (!version_env) 
    {
        std::cerr << "ROBOT_VERSION env var not set" << std::endl;
        return -1;
    }

    RobotVersion version = RobotVersion::create(std::stoi(version_env));
    HumanoidInterfaceDrake &humanoid_interface_drake = HumanoidInterfaceDrake::getInstance(version, false, 1e-3);
    auto [plant, context] = humanoid_interface_drake.getPlantAndContext();
    auto kuavo_settings = humanoid_interface_drake.getKuavoSettings();
    auto eef_names = kuavo_settings.model_settings.end_frames_name;

    drake::math::RigidTransformd foot_in_torso = plant.GetFrameByName(eef_names[1]).CalcPose(context, plant.GetFrameByName(eef_names[0]));
    std::cout << "foot in torso: " << foot_in_torso.translation().transpose() << std::endl;

    std::cout << "mass: " << humanoid_interface_drake.getPlantMass() << std::endl;
    std::cout << "mass with arm: " << humanoid_interface_drake.getPlantWithArmMass() << std::endl;
    std::cout << "Test over!\n";
    return 0;
}
