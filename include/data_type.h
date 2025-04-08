#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

namespace data_recorder_structure {
struct ExamplePointType {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct SerpentlioPointType {
    PCL_ADD_POINT4D;

    int frame_num;
    float intensity;
    uint32_t time;

    double range_from_origin;
    double range_from_origin_body;

    double cov_lidar;
    double cov_body;
    double cov_world;
    double measurement_noise;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace data_recorder_structure

POINT_CLOUD_REGISTER_POINT_STRUCT(data_recorder_structure::ExamplePointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(uint32_t, time,
                                                                                     time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    data_recorder_structure::SerpentlioPointType,
    (float, x, x)(float, y, y)(float, z, z)(int, frame_num, frame_num)(float, intensity, intensity)(
        uint32_t, time, time)(double, range_from_origin, range_from_origin)(
        double, range_from_origin_body, range_from_origin_body)(double, cov_lidar, cov_lidar)(
        double, cov_body, cov_body)(double, cov_world, cov_world)(double, measurement_noise,
                                                                  measurement_noise))
