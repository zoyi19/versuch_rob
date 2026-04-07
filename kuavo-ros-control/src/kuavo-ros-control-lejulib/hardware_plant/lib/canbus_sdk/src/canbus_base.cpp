#include "canbus_base.h"
#include "bm_canbus_wrapper.h"
#include "leju_canbus_wrapper.h"

namespace canbus_sdk {

std::unique_ptr<CanBusBase> CanBusBase::create(const std::string& canbus_name, CanBusModelType model_type, const CanBusBitrate& bitrate)
{
    // 根据型号类型创建对应的CAN总线实现
    switch (model_type) {
        case CanBusModelType::BUSMUST_A:
            return std::unique_ptr<CanBusBase>(new BMCanbusWrapper(canbus_name, model_type, bitrate));
        case CanBusModelType::BUSMUST_B:
            return std::unique_ptr<CanBusBase>(new BMCanbusWrapper(canbus_name, model_type, bitrate));
        case CanBusModelType::LEJU_CAN_A:
            return std::unique_ptr<CanBusBase>(new LejuCanbusWrapper(canbus_name, model_type, bitrate));
        case CanBusModelType::LEJU_CAN_B:
            return std::unique_ptr<CanBusBase>(new LejuCanbusWrapper(canbus_name, model_type, bitrate));
        default:
            // 未知型号，返回空指针
            return nullptr;
    }
}

const std::string& CanBusBase::getCanbusName() const
{
    return canbus_name_;
}

CanBusModelType CanBusBase::getModelType() const
{
    return model_type_;
}

} // namespace canbus_sdk