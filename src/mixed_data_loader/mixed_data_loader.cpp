#include "mixed_data_loader.h"
#include "slam_log_reporter.h"
#include "slam_operations.h"

namespace dataloader {

MixedDataLoader::~MixedDataLoader() {
    // With correct declaration order, destruction happens in the right sequence:
    // 1. CircularBuffers are destroyed first, returning objects to pools
    // 2. ObjectPools are destroyed last, safely cleaning up all objects
    Clear();
}

bool MixedDataLoader::Initialize() {
    Clear();

    // Initialize image buffers and pools
    multi_view_image_buffer_.resize(options_.kMaxNumberOfMultiViewCameras);
    multi_view_image_pool_.resize(options_.kMaxNumberOfMultiViewCameras);
    multi_view_image_mutex_.resize(options_.kMaxNumberOfMultiViewCameras);

    // Initialize mutex pointers
    imu_mutex_ = std::make_unique<std::mutex>();
    lidar_mutex_ = std::make_unique<std::mutex>();

    for (auto &mutex: multi_view_image_mutex_) {
        mutex = std::make_unique<std::mutex>();
    }

    return true;
}

void MixedDataLoader::Clear() {
    imu_buffer_.Clear();
    multi_view_image_buffer_.clear();
    lidar_buffer_.Clear();
}

bool MixedDataLoader::PushImuMeasurement(const Vec3 &accel, const Vec3 &gyro, const double &time_stamp_s) {
    if (!imu_buffer_.Empty() && imu_buffer_.Back()->time_stamp_s > time_stamp_s) {
        ReportWarn("[MixedDataLoader] Imu measurement pushed has invalid timestamp. Latest in buffer is " << imu_buffer_.Back()->time_stamp_s
                                                                                                          << " s, but pushed is " << time_stamp_s << " s.");
        return false;
    }

    auto object_ptr = imu_pool_.Get();
    object_ptr->accel = accel;
    object_ptr->gyro = gyro;
    object_ptr->time_stamp_s = time_stamp_s;

    std::unique_lock<std::mutex> lck(*imu_mutex_);
    imu_buffer_.MovePushBack(object_ptr);

    return true;
}

bool MixedDataLoader::PushImageMeasurement(GrayImage &&image, const double &time_stamp_s, const uint32_t camera_id) {
    RETURN_FALSE_IF(camera_id >= options_.kMaxNumberOfMultiViewCameras);
    auto &image_buffer = multi_view_image_buffer_[camera_id];
    auto &image_pool = multi_view_image_pool_[camera_id];
    auto &image_mutex = multi_view_image_mutex_[camera_id];

    if (!image_buffer.Empty() && image_buffer.Back()->time_stamp_s > time_stamp_s) {
        ReportWarn("[MixedDataLoader] Image measurement pushed has invalid timestamp. Latest in buffer is " << image_buffer.Back()->time_stamp_s
                                                                                                            << " s, but pushed is " << time_stamp_s << " s.");
        return false;
    }

    auto object_ptr = image_pool.Get();
    object_ptr->time_stamp_s = time_stamp_s;
    object_ptr->image = std::move(image);

    std::unique_lock<std::mutex> lck(*image_mutex);
    image_buffer.MovePushBack(object_ptr);

    return true;
}

bool MixedDataLoader::PushLidarMeasurement(ObjectPtr<LidarMeasurement> &lidar_measure) {
    if (!lidar_buffer_.Empty() && lidar_buffer_.Back()->time_stamp_s > lidar_measure->time_stamp_s) {
        ReportWarn("[MixedDataLoader] Lidar measurement pushed has invalid timestamp. Latest in buffer is "
                   << lidar_buffer_.Back()->time_stamp_s << " s, but pushed is " << lidar_measure->time_stamp_s << " s.");
        return false;
    }

    std::unique_lock<std::mutex> lck(*lidar_mutex_);
    lidar_buffer_.MovePushBack(lidar_measure);
    return true;
}

bool MixedDataLoader::PopVisualInertialMeasurePackage(VisualInertialMeasurePackage &measure) {
    // Lock all imu mutex and image mutex at the same time.
    std::unique_lock<std::mutex> imu_lck(*imu_mutex_);
    std::vector<std::unique_lock<std::mutex>> image_lcks;
    for (auto &mutex: multi_view_image_mutex_) {
        image_lcks.emplace_back(*mutex);
    }

    // Failed to pop package if imu buffer is empty.
    if (imu_buffer_.Empty()) {
        return false;
    }

    // Image data cannot be older than imu data. Discard useless image data.
    const double oldest_imu_timestamp_s = imu_buffer_.Front()->time_stamp_s;
    for (auto &buffer: multi_view_image_buffer_) {
        while (!buffer.Empty()) {
            if (buffer.Front()->time_stamp_s < oldest_imu_timestamp_s) {
                buffer.PopFront();
            } else {
                break;
            }
        }
    }

    // Failed to pop package if any image buffer is empty.
    for (auto &buffer: multi_view_image_buffer_) {
        if (buffer.Empty()) {
            return false;
        }
    }

    // Reset measure.
    measure.imu_measures.clear();
    measure.camera_measures.clear();

    // Check timestamp of multi-view images and imu data.
    double min_image_timestamp_s = multi_view_image_buffer_.front().Front()->time_stamp_s;
    for (auto &buffer: multi_view_image_buffer_) {
        min_image_timestamp_s = std::min(min_image_timestamp_s, buffer.Front()->time_stamp_s);
    }
    const double max_tolerance_image_timestamp_s = min_image_timestamp_s + options_.kMaxToleranceTimeDifferenceBetweenMultiViewImagesInSeconds;
    if (imu_buffer_.Back()->time_stamp_s < max_tolerance_image_timestamp_s) {
        return false;
    }

    // Try to pack multi-view images.
    double max_image_timestamp_s = min_image_timestamp_s;
    for (auto &buffer: multi_view_image_buffer_) {
        CONTINUE_IF(buffer.Front()->time_stamp_s > max_tolerance_image_timestamp_s);
        max_image_timestamp_s = std::max(max_image_timestamp_s, buffer.Front()->time_stamp_s);
        measure.camera_measures.emplace_back(std::move(buffer.Front()));
        buffer.PopFront();
    }

    // Try to pack imu data.
    while (imu_buffer_.Front()->time_stamp_s <= max_image_timestamp_s) {
        measure.imu_measures.emplace_back(std::move(imu_buffer_.Front()));
        imu_buffer_.PopFront();
    }

    if (options_.kEnableImuMeasurementInterpolation) {
        // Interpolate imu data at the timestamp of the last image.
        if (measure.imu_measures.back()->time_stamp_s == max_image_timestamp_s) {
            // Copy the last imu data into the buffer. So that the imu data in next package can be completed.
            auto new_item = imu_pool_.Get();
            new_item->time_stamp_s = measure.imu_measures.back()->time_stamp_s;
            new_item->accel = measure.imu_measures.back()->accel;
            new_item->gyro = measure.imu_measures.back()->gyro;
            imu_buffer_.MovePushFront(new_item);
        } else {
            // Linear interpolation for imu at the timestamp of the last image.
            auto mid = imu_pool_.Get();
            auto prev = measure.imu_measures.back().get();
            auto next = imu_buffer_.Front().get();
            const float scale = (mid->time_stamp_s - prev->time_stamp_s) / (next->time_stamp_s - prev->time_stamp_s);
            mid->time_stamp_s = max_image_timestamp_s;
            mid->accel = prev->accel * (1 - scale) + next->accel * scale;
            mid->gyro = prev->gyro * (1 - scale) + next->gyro * scale;

            auto new_item = imu_pool_.Get();
            new_item->time_stamp_s = mid->time_stamp_s;
            new_item->accel = mid->accel;
            new_item->gyro = mid->gyro;

            measure.imu_measures.emplace_back(std::move(mid));
            imu_buffer_.MovePushFront(new_item);
        }
    } else if (options_.kEnableOneMoreImuMeasurement) {
        if (!imu_buffer_.Empty()) {
            measure.imu_measures.emplace_back(std::move(imu_buffer_.Front()));
            imu_buffer_.PopFront();
        }
    }

    return true;
}

bool MixedDataLoader::PopLidarMeasurement(ObjectPtr<LidarMeasurement> &lidar_measure) {
    std::unique_lock<std::mutex> lck(*lidar_mutex_);
    RETURN_FALSE_IF(lidar_buffer_.Empty());

    lidar_measure = std::move(lidar_buffer_.Front());
    lidar_buffer_.PopFront();
    return true;
}

void MixedDataLoader::ReportPackedMeasurements(const VisualInertialMeasurePackage &measure) {
    ReportInfo("Packed visual-inertial measurements:");
    for (const auto &imu_measure: measure.imu_measures) {
        ReportInfo(" - imu " << LogTime(imu_measure->time_stamp_s) << ", accel " << LogVec(imu_measure->accel) << ", gyro " << LogVec(imu_measure->gyro)
                             << ".");
    }
    for (const auto &camera_measure: measure.camera_measures) {
        ReportInfo(" - image " << LogTime(camera_measure->time_stamp_s) << ", image size [" << camera_measure->image.rows() << " x "
                               << camera_measure->image.cols() << "].");
    }
}

bool MixedDataLoader::ValidateVisualInertialMeasurePackage(const VisualInertialMeasurePackage &measure) {
    // Check integrity of the packaged measurements.
    if (measure.camera_measures.size() != options_.kMaxNumberOfMultiViewCameras) {
        ReportError("Failed to load multi-view image from packed measurement.");
        return false;
    }
    for (const auto &camera_measure: measure.camera_measures) {
        if (camera_measure->image.data() == nullptr || camera_measure->image.rows() == 0 || camera_measure->image.cols() == 0) {
            ReportError("Failed to load image data from packed measurement.");
            return false;
        }
    }

    return true;
}
}  // namespace dataloader
