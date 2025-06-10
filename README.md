# Data Recorder

## 1. Build & Use
1. Build the package
   ```bash
   mkdir build && cd build && cmake .. && make
   ```
2. Use the package in other projects. In `CMakeLists.txt`, add the following line:
   ```cmake
   target_link_libraries({your_target} {path_to_data_recorder}/build/libdata_recorder.so)
   target_include_directories({your_target} PUBLIC {path_to_data_recorder}/include/data_recorder)
   ```
   for example:
   ```cmake
    target_link_libraries(fastlio_mapping /workspace/lio_baselines_ws/src/lio_baselines/include/data_recorder/build/libdata_recorder.so)
    target_include_directories(fastlio_mapping PRIVATE /workspace/lio_baselines_ws/src/lio_baselines/include/data_recorder)
   ```
## 2. How to use
1. include시에는 `#include "data_recorder.h"`
2. 객체 선언
     ```cpp
     std::shared_ptr<DataRecorder<RecordPointType>> recorder_ptr_;
     ```
3. 변수 불러오기
   1. Yaml 파일에 다음과 같이 추가
         ```yaml
         data_recorder:
             result_dir: "/workspace/lio_baselines_ws/src/lio_baselines/result/NTU"
             data_id: "nya_01"
             lidar_names: ["OsHori","OsVert"]
             lidar_indices: [0]
             test_topic: "baselines"
             param_set_name: "default"
         ```
   2. 소스파일에서 ros 파라미터 불러오기
         ```cpp
         // Data Recorder Configurations
         nh.param<std::string>("data_recorder/result_dir", result_dir, "/");
         nh.param<std::string>("data_recorder/data_id", data_id, "data_id");
         nh.param<std::string>("data_recorder/test_topic", test_topic, "test_topic");
         nh.param<std::string>("data_recorder/param_set_name", param_set_name, "default");
         nh.param<std::vector<std::string>>("data_recorder/lidar_names", lidar_names,
                                         std::vector<std::string>());
         nh.param<std::vector<int>>("data_recorder/lidar_indices", lidar_indices, std::vector<int>());

         std::string lidars_combination = "";
         for (auto &index : lidar_indices) {
             lidars_combination += lidar_names[index] + "_";
         }
         lidars_combination = lidars_combination.substr(0, lidars_combination.size() - 1);

         save_dir = result_dir + "/" + data_id + "/" + lidars_combination + "/" + test_topic + "/" +
                 lidars_combination + "/fastlio" + "/" + param_set_name;

         // Check variables
         std::cout << "\033[32m" << "Data Recorder Configurations:" << std::endl;
         std::cout << "Result Directory: " << result_dir << std::endl;
         std::cout << "Data ID: " << data_id << std::endl;
         std::cout << "Test Topic: " << test_topic << std::endl;
         std::cout << "Parameter Set Name: " << param_set_name << std::endl;
         std::cout << "LiDAR Comb.: " << lidars_combination << std::endl;
         std::cout << "Save Directory: " << save_dir << std::endl;
         std::cout << "\033[0m" << std::endl;
         ```
4. 객체 초기화
     ```cpp 
     recorder_ptr_.reset(new DataRecorder<RecordPointType>());
     recorder_ptr_->init(save_dir, 0, true);
     ```
5. 데이터 저장
   1. Pose reord    
         ```cpp
         Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
         pose.block<3, 3>(0, 0) = state_point.rot.toRotationMatrix();
         pose.block<3, 1>(0, 3) = state_point.pos;

         Eigen::Matrix<double, 6, 6> pose_cov = kf.get_P().block<6, 6>(0, 0);

         recorder_ptr_->recordPose(lidar_end_time, std::tuple(pose, pose_cov));
         ```
      - `double` type의 stamp, `Eigen::Matrix4d` type의 pose, `Eigen::Matrix<double, 6, 6>` type의 covariance를 저장
   2. Time reord
         ```cpp
         // p_imu->Process(Measures, kf, feats_undistort); // 원래 코드
         recorder_ptr_->recordTime([&]() { p_imu->Process(Measures, kf, feats_undistort); },
                                     "imu_process", lidar_end_time); 
         // "imu_process" 위치는 해당 부분을 부르는 이름이 들어가고, lidar_end_time는 해당 부분의 시간
         ```
      - Core한 부분을 다음과 같이 저장
   3. RAM 사용량 reord
         ```cpp
         void recordRamUsage(double stamp) {
             pid_t pid = getpid();
             std::string path = "/proc/" + std::to_string(pid) + "/status";
             std::ifstream file(path);
             std::string line;
             double mem_usage = 0.0;
             while (std::getline(file, line)) {
                 if (line.find("VmRSS:") == 0) {
                     mem_usage = std::stod(line.substr(6)) / 1024.0; // Convert to MB
                     break;
                 }
             }

             recorder_ptr_->recordValue("RAM_usage", stamp, mem_usage);
         }
         ```
   4. 서비스 콜백을 통한 최종 저장
      1. 서비스 콜백 함수에서 다음과 같이 저장 및 선언
         ```cpp
         bool data_recorder_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
             if (recorder_ptr_ != nullptr) {

                 recorder_ptr_->savePose();
                 recorder_ptr_->saveTime();
                 recorder_ptr_->saveValue();
                 recorder_ptr_->saveStatus("Finished");

                 res.success = true;
                 res.message = "Data saved successfully.";
             } else {
                 res.success = false;
                 res.message = "Recorder pointer is null, cannot save data.";
             }
             return true;
         }

         recorder_server_ = nh.advertiseService("save_data", data_recorder_callback);

         ```
      2. Bag 파일 끝나면 서비스 콜
         ```bash
         rosservice call /save_data
         ```
## 3. Show Result
1. Result 확인
   - 다음과 같이 결과가 저장됨
    ```bash
    result
    |___{dataset}
    |   |___{data_id}
    |       |___{lidar_combination}
    |           |___{test_topic}
    |               |___{algorithm}
    |                   |___{param_set_name}
    |                       |___poses.txt
    |                       |___status.txt
    |                       |___time
    |                           |___{modul_1}.txt
    |                           |___{modul_2}.txt 
    |                           |___...
    |                       |___values
    |                           |___{RAM_usage}.txt
    ```
2. EVO를 통한 APE RMSE 확인
   1. evo 설치
   2. 다음과 같이 실행
        ```bash
        evo_ape tum {gt_path} {est_path} --t_max_diff 0.1 --align --plot
        ```