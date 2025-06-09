#include "data_recorder.h"

template <typename PointType> DataRecorder<PointType>::DataRecorder() {}

template <typename PointType>
void DataRecorder<PointType>::init(const std::string &dir, int save_mode, bool verbose) {
    save_dir_ = dir;
    is_init_ = true;
    verbose_ = verbose;

    std::string mkdir_cmd = "mkdir -p " + save_dir_ + "/values " + save_dir_ + "/times";

    if (save_mode == 2) {
        enable_cloud_record_ = true;
        mkdir_cmd += " " + save_dir_ + "/clouds";
        std::cout << "Cloud recording enabled. Please make sure the directory is empty"
                  << std::endl;
    }
    int ret_mk = system(mkdir_cmd.c_str());

    if (ret_mk != 0) {
        std::cerr << "[Error] Failed to initialize directories in " << save_dir_ << std::endl;
        is_init_ = false;
        return;
    }

    status_path_ = save_dir_ + "/status.txt";
    std::ofstream status_file(status_path_);
    status_file << "Initialized" << std::endl;
    status_file.close();

    if (verbose_) {
        std::cout << "DataRecorder initialized with directory with save_mode: " << save_mode
                  << std::endl;
        std::cout << "DataRecorder save_dir_: " << save_dir_ << std::endl;
    }
}

template <typename PointType>
void DataRecorder<PointType>::recordCloud(const CloudPtr &cloud, double stamp) {

    record_cloud_.cloud_ptr.reset(new pcl::PointCloud<PointType>);
    *record_cloud_.cloud_ptr = *cloud;

    record_cloud_.stamp = stamp;

    cloud_enabled_ = true;
}

template <typename PointType>
void DataRecorder<PointType>::recordValue(const std::string &name, double stamp, double value) {
    if (value_recorder_.find(name) == value_recorder_.end()) {
        value_recorder_[name] = std::vector<std::tuple<double, double>>();
    }
    value_recorder_[name].push_back(std::make_tuple(stamp, value));
}

template <typename PointType>
void DataRecorder<PointType>::recordPose(double stamp, const PoseWithCov &pose) {
    pose_recorder_.push_back(std::make_tuple(stamp, pose));
}

template <typename PointType> void DataRecorder<PointType>::saveCloud() {

    if (!cloud_enabled_) {
        std::cout << "DataRecorder: saveCloud called, but no cloud recorded!" << std::endl;
        return;
    }

    std::cout << "DataRecorder: Saving cloud to " << save_dir_ + "/clouds/" << std::endl;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << record_cloud_.stamp;
    std::string stamp_str = oss.str();
    std::string cloud_pcd_path = save_dir_ + "/clouds/" + stamp_str + ".pcd";
    pcl::io::savePCDFileBinary(cloud_pcd_path, *record_cloud_.cloud_ptr);

    cloud_enabled_ = false;
}

template <typename PointType> void DataRecorder<PointType>::saveValue() {
    for (const auto &pair : value_recorder_) {
        std::string name = pair.first;
        std::vector<std::tuple<double, double>> values = pair.second;

        std::string value_path = save_dir_ + "/values/" + name + ".txt";
        std::ofstream value_file(value_path);

        for (const auto &value : values) {
            value_file << std::fixed << std::setprecision(6) << std::get<0>(value) << " "
                       << std::get<1>(value) << "\n";
        }
        value_file.close();
    }
}

template <typename PointType> void DataRecorder<PointType>::saveTime() {
    for (const auto &pair : time_recorder_) {
        std::string name = pair.first;
        std::vector<std::tuple<double, double>> times = pair.second;

        std::string time_path = save_dir_ + "/times/" + name + ".txt";
        std::ofstream time_file(time_path);

        for (const auto &time : times) {
            time_file << std::fixed << std::setprecision(6) << std::get<0>(time) << " "
                      << std::get<1>(time) << "\n";
        }
        time_file.close();
    }
}

template <typename PointType> void DataRecorder<PointType>::savePose() {
    std::string pose_path = save_dir_ + "/poses.txt";
    std::ofstream pose_file(pose_path);

    for (const auto &pose_tuple : pose_recorder_) {

        double stamp = std::get<0>(pose_tuple);
        PoseWithCov pose_with_cov = std::get<1>(pose_tuple);

        Eigen::Matrix4d pose = std::get<0>(pose_with_cov);

        Eigen::Vector3d pos = pose.block<3, 1>(0, 3);
        Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));

        pose_file << std::fixed << std::setprecision(6) << stamp << " " << pos(0) << " " << pos(1)
                  << " " << pos(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " "
                  << quat.w() << "\n";
    }
    pose_file.close();
}

template <typename PointType> void DataRecorder<PointType>::savePoseWithCov() {
    std::string pose_path = save_dir_ + "/poses_w_cov.txt";
    std::ofstream pose_file(pose_path);

    for (const auto &pose_tuple : pose_recorder_) {

        double stamp = std::get<0>(pose_tuple);
        PoseWithCov pose_with_cov = std::get<1>(pose_tuple);

        Eigen::Matrix4d pose = std::get<0>(pose_with_cov);
        Eigen::Matrix<double, 6, 6> cov = std::get<1>(pose_with_cov);

        Eigen::Vector3d pos = pose.block<3, 1>(0, 3);
        Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));

        pose_file << std::fixed << std::setprecision(6) << stamp << " " << pos(0) << " " << pos(1)
                  << " " << pos(2) << " " << quat.x() << " " << quat.y() << " " << quat.z() << " "
                  << quat.w() << " ";

        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (i == 5 && j == 5) {
                    pose_file << cov(i, j);
                } else {
                    pose_file << cov(i, j) << " ";
                }
            }
        }

        pose_file << "\n";
    }
    pose_file.close();
}

template <typename PointType> void DataRecorder<PointType>::saveStatus(std::string status) {
    std::ofstream status_file(status_path_);

    status_file << status << "\n";

    status_file.close();
}

template class DataRecorder<RecordPointType>;