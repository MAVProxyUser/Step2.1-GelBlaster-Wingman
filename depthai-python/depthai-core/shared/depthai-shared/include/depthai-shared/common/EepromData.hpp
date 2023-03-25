#pragma once
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraInfo.hpp"
#include "depthai-shared/common/Extrinsics.hpp"
#include "depthai-shared/common/Point3f.hpp"
#include "depthai-shared/common/StereoRectification.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {
/**
 * EepromData structure
 *
 * Contains the Calibration and Board data stored on device
 */
struct EepromData {
    uint32_t version = 7;
    std::string productName, boardCustom, boardName, boardRev, boardConf, hardwareConf, batchName;
    uint64_t batchTime{0};
    uint32_t boardOptions{0};
    std::unordered_map<CameraBoardSocket, CameraInfo> cameraData;
    StereoRectification stereoRectificationData;
    Extrinsics imuExtrinsics;
    std::vector<uint8_t> miscellaneousData;
};

DEPTHAI_SERIALIZE_OPTIONAL_EXT(EepromData,
                               version,
                               boardCustom,
                               boardName,
                               boardRev,
                               boardConf,
                               hardwareConf,
                               productName,
                               batchName,
                               batchTime,
                               boardOptions,
                               cameraData,
                               stereoRectificationData,
                               imuExtrinsics,
                               miscellaneousData);

}  // namespace dai
