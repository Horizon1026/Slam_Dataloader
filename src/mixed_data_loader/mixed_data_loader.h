#ifndef _MIXED_DATA_LOADER_H_
#define _MIXED_DATA_LOADER_H_

#include "basic_type.h"
#include "camera_measurement.h"
#include "circular_buffer.h"
#include "datatype_image.h"
#include "imu_measurement.h"
#include "lidar_measurement.h"
#include "object_pool.h"

#include "mutex"

namespace dataloader {

using namespace slam_utility;
using namespace sensor_model;

/* Class MixedDataLoader Declaration. */
class MixedDataLoader final {

public:
    struct VisualInertialMeasurePackage {
        std::vector<ObjectPtr<ImuMeasurement>> imu_measures;
        std::vector<ObjectPtr<CameraMeasurement>> camera_measures;
    };
    struct Options {
        float kMaxToleranceTimeDifferenceBetweenMultiViewImagesInSeconds = 0.005f;
        uint32_t kMaxNumberOfMultiViewCameras = 1;
        bool kEnableImuMeasurementInterpolation = true;
        bool kEnableOneMoreImuMeasurement = false;
    };

public:
    MixedDataLoader() = default;
    ~MixedDataLoader();

    bool Initialize();
    void Clear();

    // Push measurements into dataloader.
    bool PushImuMeasurement(const Vec3 &accel, const Vec3 &gyro, const double &time_stamp_s);
    bool PushImageMeasurement(GrayImage &&image, const double &time_stamp_s, const uint32_t camera_id = 0);
    bool PushLidarMeasurement(ObjectPtr<LidarMeasurement> &lidar_measure);

    // Pop measurements from dataloader.
    bool PopVisualInertialMeasurePackage(VisualInertialMeasurePackage &measure);
    bool PopLidarMeasurement(ObjectPtr<LidarMeasurement> &lidar_measure);
    bool ValidateVisualInertialMeasurePackage(const VisualInertialMeasurePackage &measure);
    static void ReportPackedMeasurements(const VisualInertialMeasurePackage &measure);

    // Check if any buffer is full.
    bool IsImuBufferFull() const { return imu_buffer_.Full(); }
    bool IsImageBufferFull(int32_t camera_id) const { return multi_view_image_buffer_[camera_id].Full(); }
    bool IsLidarBufferFull() const { return lidar_buffer_.Full(); }

    // Reference for member variables.
    Options &options() { return options_; }
    // Const reference for member variables.
    const Options &options() const { return options_; }

private:
    Options options_;

    // ObjectPool must be declared before CircularBuffer to ensure correct destruction order
    ObjectPool<ImuMeasurement> imu_pool_;
    std::vector<ObjectPool<CameraMeasurement>> multi_view_image_pool_;
    ObjectPool<LidarMeasurement> lidar_pool_;

    CircularBuffer<ObjectPtr<ImuMeasurement>, 200> imu_buffer_;
    std::vector<CircularBuffer<ObjectPtr<CameraMeasurement>, 20>> multi_view_image_buffer_;
    CircularBuffer<ObjectPtr<LidarMeasurement>, 20> lidar_buffer_;

    std::unique_ptr<std::mutex> imu_mutex_;
    std::vector<std::unique_ptr<std::mutex>> multi_view_image_mutex_;
    std::unique_ptr<std::mutex> lidar_mutex_;
};

}  // namespace dataloader

#endif  // end of _MIXED_DATA_LOADER_H_
