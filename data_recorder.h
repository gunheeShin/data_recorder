#ifndef DATA_RECORDER_H
#define DATA_RECORDER_H

#include <filesystem>
#include <iostream>
#include <map>

#include <pcl/io/pcd_io.h>

#include "include/data_type.h"

typedef data_recorder_structure::SerpentlioPointType RecordPointType;

template <typename PointType> // custom pcl point type
class DataRecorder {

    typedef typename pcl::PointCloud<PointType>::Ptr CloudPtr;
    typedef std::tuple<Eigen::Matrix4d, Eigen::Matrix<double, 6, 6>> PoseWithCov;

    struct cloud_data {
        CloudPtr cloud_ptr;
        double stamp;
    };

public:
    DataRecorder();

    void recordCloud(const CloudPtr &cloud, double stamp);
    void recordValue(const std::string &name, double stamp, double value);
    void recordPose(double stamp, const PoseWithCov &pose);

    template <class F> void recordTime(F &&func, const std::string &name, double stamp) {
        auto t1 = std::chrono::high_resolution_clock::now();
        std::forward<F>(func)();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() *
            1000; // in ms

        if (time_recorder_.find(name) != time_recorder_.end()) {
            time_recorder_[name].emplace_back(std::make_tuple(stamp, time_used));
        } else {
            time_recorder_.insert(
                {name, std::vector<std::tuple<double, double>>{std::make_tuple(stamp, time_used)}});
        }
    }

    void saveCloud();
    void saveValue();
    void saveTime();
    void savePose();
    void savePoseWithCov();
    void saveStatus(std::string status);

    void init(const std::string &dir, int save_mode, bool verbose);

    bool isInit() { return is_init_; }
    bool isCloudRecordEnabled() { return enable_cloud_record_; }

private:
    std::map<std::string, std::vector<std::tuple<double, double>>> value_recorder_;
    std::map<std::string, std::vector<std::tuple<double, double>>> time_recorder_;
    std::vector<std::tuple<double, PoseWithCov>> pose_recorder_;

    cloud_data record_cloud_;

    bool is_init_ = false;
    bool enable_cloud_record_ = false;
    bool cloud_enabled_ = false;
    bool verbose_ = false;

    std::string save_dir_;
};
#endif // DATA_RECORDER_H