#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <execution>
#include <fstream>

#include "laser_mapping.h"
#include "utils.h"


// #define DEBUG
// #define TEST_VOXEL

namespace lvio_lite {

bool LaserMapping::InitROS(ros::NodeHandle &nh) {
    LoadParams(nh);
    SubAndPubToROS(nh);

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    if(!flg_islocation_mode_){
        kf_.init_dyn_share(
            get_f, df_dx, df_dw,
            [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
            options::NUM_MAX_ITERATIONS, epsi.data());
        return true;
    }
    
    kf_.init_dyn_share(
            get_f, df_dx, df_dw,
            [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel_location(s, ekfom_data); },
            options::NUM_MAX_ITERATIONS, epsi.data());
    return true;
}

bool LaserMapping::InitWithoutROS(const std::string &config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    // localmap init (after LoadParams)
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    // esekf init
    std::vector<double> epsi(23, 0.001);
    kf_.init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) { ObsModel(s, ekfom_data); },
        options::NUM_MAX_ITERATIONS, epsi.data());

    if (std::is_same<IVoxType, IVox<3, IVoxNodeType::PHC, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using phc ivox";
    } else if (std::is_same<IVoxType, IVox<3, IVoxNodeType::DEFAULT, pcl::PointXYZI>>::value == true) {
        LOG(INFO) << "using default ivox";
    }

    return true;
}

bool LaserMapping::LoadParams(ros::NodeHandle &nh) {
    // get params from param server
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    nh.param<bool>("location_mode", flg_islocation_mode_, false);
    nh.param<bool>("path_save_en", path_save_en_, true);
    nh.param<bool>("publish/path_publish_en", path_pub_en_, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en_, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en_, false);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en_, true);
    nh.param<bool>("publish/scan_effect_pub_en", scan_effect_pub_en_, false);
    nh.param<bool>("publish/keyframe_pub_en", b_pub_keyframe_, true);

    nh.param<int>("max_iteration", options::NUM_MAX_ITERATIONS, 4);
    nh.param<float>("esti_plane_threshold", options::ESTI_PLANE_THRESHOLD, 0.1);
    nh.param<std::string>("map_file_path", map_file_path_, "");
    nh.param<bool>("common/time_sync_en", time_sync_en_, false);
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min_, 0.0);
    nh.param<double>("cube_side_length", cube_len_, 200);
    nh.param<float>("mapping/det_range", det_range_, 300.f);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind2", preprocess_->Blind(), 0.01);
    nh.param<double>("preprocess/max_range2", preprocess_->MaxRange(), 10000);
    nh.param<float>("preprocess/time_scale", preprocess_->TimeScale(), 1e-3);
    nh.param<int>("preprocess/lidar_type", lidar_type, 1);
    nh.param<int>("preprocess/scan_line", preprocess_->NumScans(), 16);
    nh.param<int>("point_filter_num", preprocess_->PointFilterNum(), 2);
    nh.param<bool>("feature_extract_enable", preprocess_->FeatureEnabled(), false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log_, true);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en_, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en_, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval_, -1);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT_, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR_, std::vector<double>());

    nh.param<float>("ivox_grid_resolution", ivox_options_.resolution_, 0.2);
    nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);
    int _capacity;
    nh.param<int>("ivox_capacity", _capacity, 1000000);
    ivox_options_.capacity_ = static_cast<std::size_t>(_capacity);

    nh.param<std::string>("load_g_map", str_g_map_, "empty");
    nh.param<std::string>("load_f_map", str_f_map_, "empty");
    nh.param<double>("load_eaf_size", load_eaf_size_, 0.5);

    nh.param<bool>("split_map", split_map_, false);
    nh.param<float>("sub_grid_resolution", sub_grid_resolution_, 100);

    std::vector<double> _init_trans;
    std::vector<double> _init_rpy;
    nh.param<std::vector<double>>("init_trans", _init_trans, std::vector<double>());
    nh.param<std::vector<double>>("init_rpy", _init_rpy, std::vector<double>());
    yaml_init_translation_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(_init_trans.data(), 3, 1);
    yaml_init_rotation_ = Eigen::AngleAxisd(_init_rpy[0]/180 * M_PI, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(_init_rpy[1]/180 * M_PI, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(_init_rpy[2]/180 * M_PI, Eigen::Vector3d::UnitZ());

    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "\033[1;32m Using Mid-360 Lidar \033[0m";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "\033[1;32m Using Velodyne 32 Lidar \033[0m";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "\033[1;32m Using OUST 64 Lidar \033[0m";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::HESAI16);
        LOG(INFO) << "\033[1;32m Using HESAI XT16 Lidar \033[0m";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "map";

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string &yaml_file) {
    // get params from yaml
    int lidar_type, ivox_nearby_type;
    double gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    double filter_size_surf_min;
    common::V3D lidar_T_wrt_IMU;
    common::M3D lidar_R_wrt_IMU;

    auto yaml = YAML::LoadFile(yaml_file);
    try {
        path_pub_en_ = yaml["publish"]["path_publish_en"].as<bool>();
        scan_pub_en_ = yaml["publish"]["scan_publish_en"].as<bool>();
        dense_pub_en_ = yaml["publish"]["dense_publish_en"].as<bool>();
        scan_body_pub_en_ = yaml["publish"]["scan_bodyframe_pub_en"].as<bool>();
        scan_effect_pub_en_ = yaml["publish"]["scan_effect_pub_en"].as<bool>();
        path_save_en_ = yaml["path_save_en"].as<bool>();

        options::NUM_MAX_ITERATIONS = yaml["max_iteration"].as<int>();
        options::ESTI_PLANE_THRESHOLD = yaml["esti_plane_threshold"].as<float>();
        time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();

        filter_size_surf_min = yaml["filter_size_surf"].as<float>();
        filter_size_map_min_ = yaml["filter_size_map"].as<float>();
        cube_len_ = yaml["cube_side_length"].as<int>();
        det_range_ = yaml["mapping"]["det_range"].as<float>();
        gyr_cov = yaml["mapping"]["gyr_cov"].as<float>();
        acc_cov = yaml["mapping"]["acc_cov"].as<float>();
        b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<float>();
        b_acc_cov = yaml["mapping"]["b_acc_cov"].as<float>();
        preprocess_->Blind() = yaml["preprocess"]["blind"].as<double>();
        preprocess_->TimeScale() = yaml["preprocess"]["time_scale"].as<double>();
        lidar_type = yaml["preprocess"]["lidar_type"].as<int>();
        preprocess_->NumScans() = yaml["preprocess"]["scan_line"].as<int>();
        preprocess_->PointFilterNum() = yaml["point_filter_num"].as<int>();
        preprocess_->FeatureEnabled() = yaml["feature_extract_enable"].as<bool>();
        extrinsic_est_en_ = yaml["mapping"]["extrinsic_est_en"].as<bool>();
        pcd_save_en_ = yaml["pcd_save"]["pcd_save_en"].as<bool>();
        pcd_save_interval_ = yaml["pcd_save"]["interval"].as<int>();
        extrinT_ = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = yaml["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = yaml["ivox_nearby_type"].as<int>();
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::HESAI16);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    lidar_T_wrt_IMU = common::VecFromArray<double>(extrinT_);
    lidar_R_wrt_IMU = common::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(lidar_T_wrt_IMU, lidar_R_wrt_IMU);
    p_imu_->SetGyrCov(common::V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(common::V3D(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(common::V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(common::V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    run_in_offline_ = true;
    return true;
}

void LaserMapping::SubAndPubToROS(ros::NodeHandle &nh) {
    // ROS subscribe initialization
    std::string lidar_topic, imu_topic;
    nh.param<std::string>("common/lid_topic", lidar_topic, "/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic, "/livox/imu");

    if (preprocess_->GetLidarType() == LidarType::AVIA) {
        sub_pcl_ = nh.subscribe<livox_ros_driver::CustomMsg>(
            lidar_topic, 200000, [this](const livox_ros_driver::CustomMsg::ConstPtr &msg) { LivoxPCLCallBack(msg); });
    } else {
        sub_pcl_ = nh.subscribe<sensor_msgs::PointCloud2>(
            lidar_topic, 200000, [this](const sensor_msgs::PointCloud2::ConstPtr &msg) { StandardPCLCallBack(msg); });
    }

    sub_imu_ = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200000,
                                              [this](const sensor_msgs::Imu::ConstPtr &msg) { IMUCallBack(msg); });

    // ROS publisher init
    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "map";

    pub_laser_cloud_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);
    pub_laser_cloud_body_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 1000);
    pub_laser_cloud_effect_world_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_effect_world", 10);
    pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/Odometry", 10);
    pub_path_ = nh.advertise<nav_msgs::Path>("/path", 10);
    

    // location
    if(flg_islocation_mode_){
        sub_init_pose_ = nh.subscribe("/initialpose", 1, &LaserMapping::initialpose_callback, this);
        pub_global_map_ = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1);
        pub_feature_map_ = nh.advertise<sensor_msgs::PointCloud2>("/feature_map", 1);
        visual_timer_ = nh.createTimer(ros::Duration(2), &LaserMapping::VisualMap, this);
    }
    // for uav;
    pub_msg2uav_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);


    if(b_pub_keyframe_){
        pub_keyframe_ = nh.advertise<lvio_lite::KeyFrame>("/lvio/keyframe", 10);
        fov_segment_thread_ = new std::thread(std::bind(&LaserMapping::FovSegment_pub_thread, this));
        fov_segment_thread_->detach();
    }

}

LaserMapping::LaserMapping() {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

void LaserMapping::Run() {
    if (!SyncPackages()) {
        return;
    }

    /// IMU process, kf prediction, undistortion
    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    /// the first scan
    if (flg_first_scan_) {
        CloudPtr first_scan(new PointCloudType);
        first_scan->resize(scan_undistort_->size());
        size_t cur_size = scan_undistort_->size();
        for (int i = 0; i < cur_size; i++) {
            PointBodyToWorld(&scan_undistort_->points[i], &first_scan->points[i]);
        }
        ivox_->AddPoints(first_scan->points);
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, true);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    // update local map
    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    #ifdef TEST_VOXEL
    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " downsamp " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_num_;
    #endif

    // publish or save map pcd
    {
        if(b_pub_keyframe_){
            SaveKeyFrame6D();
        }

        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        if (scan_pub_en_ || pcd_save_en_) {
            PublishFrameWorld();
        }
        if (scan_pub_en_ && scan_body_pub_en_) {
            PublishFrameBody(pub_laser_cloud_body_);
        }
        if (scan_pub_en_ && scan_effect_pub_en_) {
            PublishFrameEffectWorld(pub_laser_cloud_effect_world_);
        }
    }

    frame_num_++;
}

void LaserMapping::Run_location(){
    if (!SyncPackages()) {
        return;
    }

    p_imu_->Process(measures_, kf_, scan_undistort_);
    if (scan_undistort_->empty() || (scan_undistort_ == nullptr)) {
        LOG(WARNING) << "No point, skip this scan!";
        return;
    }

    if (flg_first_scan_) {
        first_lidar_time_ = measures_.lidar_bag_time_;
        flg_first_scan_ = false;
        return;
    }
    flg_EKF_inited_ = (measures_.lidar_bag_time_ - first_lidar_time_) >= options::INIT_TIME;

    if(!flg_location_inited_){
        initialpose();
        return;
    }
    
    /// downsample
    Timer::Evaluate(
        [&, this]() {
            voxel_scan_.setInputCloud(scan_undistort_);
            voxel_scan_.filter(*scan_down_body_);
        },
        "Downsample PointCloud");

    int cur_pts = scan_down_body_->size();
    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return;
    }
    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, false);
    plane_coef_.resize(cur_pts, common::V4F::Zero());

    // ICP and iterated Kalman filter update
    Timer::Evaluate(
        [&, this]() {
            // iterated state estimation
            double solve_H_time = 0;
            // update the observation model, will call nn and point-to-plane residual computation
            kf_.update_iterated_dyn_share_modified(options::LASER_POINT_COV, solve_H_time);
            // save the state
            state_point_ = kf_.get_x();
            euler_cur_ = SO3ToEuler(state_point_.rot);
            pos_lidar_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        },
        "IEKF Solve and Update");

    if(split_map_){
        DynamicLoadMap(state_point_.pos);
    }

    {
        if (pub_odom_aft_mapped_) {
            PublishOdometry(pub_odom_aft_mapped_);
        }
        if (path_pub_en_ || path_save_en_) {
            PublishPath(pub_path_);
        }
        if (scan_pub_en_ || pcd_save_en_) {
            PublishFrameWorld();
        }
        if (scan_pub_en_ && scan_body_pub_en_) {
            PublishFrameBody(pub_laser_cloud_body_);
        }
        if (scan_pub_en_ && scan_effect_pub_en_) {
            PublishFrameEffectWorld(pub_laser_cloud_effect_world_);
        }
        

    }
}


void LaserMapping::DynamicLoadMap(Vec3d pose){
    static std::string split_map_path(std::string(ROOTDIR + "maps/split_map/"));
    auto t1 = std::chrono::high_resolution_clock::now();

    int gx = floor((pose[0] - sub_grid_resolution_/2)/sub_grid_resolution_);
    int gy = floor((pose[1] - sub_grid_resolution_/2)/sub_grid_resolution_);
    Vec2i key(gx, gy);

    std::set<Vec2i, less_vec<2>> surrounding_index{
        key + Vec2i(0, 0), key + Vec2i(-1, 0), key + Vec2i(-1, -1), key + Vec2i(-1, 1), key + Vec2i(0, -1),
        key + Vec2i(0, 1), key + Vec2i(1, 0),  key + Vec2i(1, -1),  key + Vec2i(1, 1),
    };

    // 加载必要区域
    bool map_data_changed = false;
    int cnt_new_loaded = 0, cnt_unload = 0;
    for (auto& k : surrounding_index) {
        if (map_data_index_.find(k) == map_data_index_.end()) {
            continue;
        }
        if (hold_map_.find(k) == hold_map_.end()) {
            PointCloudType::Ptr cloud(new PointCloudType);
            pcl::io::loadPCDFile(split_map_path + std::to_string(k[0]) + "_" 
                                 + std::to_string(k[1]) + ".pcd", *cloud);
            hold_map_.emplace(k);
            ivox_->AddPoints(cloud->points);
            map_data_changed = true;
            cnt_new_loaded++;
        }
    }

    for (auto iter = hold_map_.begin(); iter != hold_map_.end();) {
        if ((*iter - key).cast<float>().norm() > 3.0) {
            iter = hold_map_.erase(iter);
            cnt_unload++;
            map_data_changed = true;
        } else {
            iter++;
        }
    }

    if (map_data_changed) {
        auto t2 = std::chrono::high_resolution_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
        LOG(INFO) << "new loaded: " << cnt_new_loaded << ", unload: " << cnt_unload << "  size: " << ivox_->NumValidGrids()
                  << "  time: "  << time_used << " ms";
    }
}


void LaserMapping::initialpose(){
    Eigen::Affine3d init_guess;
    if(flg_get_init_guess_){
        init_lock_.lock();
        init_guess.translation() = init_translation_;
        init_guess.rotate(init_rotation_);
        init_lock_.unlock();
    }else{
        init_guess = Eigen::Affine3d::Identity();
        init_guess.translation() = yaml_init_translation_;
        init_guess.rotate(yaml_init_rotation_);
    }

    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    ndt.setTransformationEpsilon(1e-4);
    ndt.setEuclideanFitnessEpsilon(1e-4);
    ndt.setMaximumIterations(40);
    ndt.setResolution(0.8);
    ndt.setInputSource(scan_undistort_);
    ndt.setInputTarget(global_map_);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(40);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(scan_undistort_);
    icp.setInputTarget(global_map_);

    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    ndt.align(*unused_result, init_guess.matrix().cast<float>());
    icp.align(*unused_result, ndt.getFinalTransformation());

    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.25)
    {
        ROS_ERROR("Global Initializing Fail! ");
        flg_location_inited_ = false;
        if(flg_get_init_guess_){
            flg_get_init_guess_ = false;
        }
        return;
    } else{
        init_guess = icp.getFinalTransformation().cast<double>();
        Eigen::Vector3d final_position = init_guess.translation();
        Eigen::Quaterniond final_rotation(init_guess.linear());
        ROS_INFO("\033[1;35m Initializing Succeed! \033[0m");
        state_ikfom init_state = kf_.get_x();
        init_state.pos = final_position;
        init_state.rot = final_rotation;
        kf_.change_x(init_state);
        flg_location_inited_ = true;
        sub_init_pose_.shutdown();
        global_map_ = nullptr;
        pcl_feature_point_ = nullptr;
        DynamicLoadMap(final_position);
    }
}

void LaserMapping::initialpose2(){
    Eigen::Affine3d init_guess;
    if(flg_get_init_guess_){
        init_lock_.lock();
        init_guess.translation() = init_translation_;
        init_guess.rotate(init_rotation_);
        init_lock_.unlock();
    }else{
        init_guess = Eigen::Affine3d::Identity();
    }

    P2P::Ndt3d ndt;
    ndt.SetTarget(global_map_);
    ndt.SetSource(scan_undistort_);
    ndt.AlignNdt(init_guess);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(40);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(scan_undistort_);
    icp.setInputTarget(global_map_);

    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());

    icp.align(*unused_result, init_guess.matrix().cast<float>());

    if (icp.hasConverged() == false || icp.getFitnessScore() > 0.25)
    {
        ROS_ERROR("Global Initializing Fail! ");
        flg_location_inited_ = false;
        if(flg_get_init_guess_){
            flg_get_init_guess_ = false;
        }
        return;
    } else{
        init_guess = icp.getFinalTransformation().cast<double>();
        Eigen::Vector3d final_position = init_guess.translation();
        Eigen::Quaterniond final_rotation(init_guess.linear());
        ROS_INFO("\033[1;35m Initializing Succeed! \033[0m");
        state_ikfom init_state = kf_.get_x();
        init_state.pos = final_position;
        init_state.rot = final_rotation;
        kf_.change_x(init_state);
        flg_location_inited_ = true;
        sub_init_pose_.shutdown();
        global_map_ = nullptr;
        pcl_feature_point_ = nullptr;
    }
}


void LaserMapping::Load_map(){
    LOG(INFO) << "\033[1;33m Load GlobalMap now, please wait...\033[0m";
    std::string all_points_dir(std::string(ROOTDIR + "maps/") + str_g_map_);
    pcl::io::loadPCDFile(all_points_dir, *global_map_);

    std::string feature_dir(std::string(ROOTDIR + "maps/") + str_f_map_);
    pcl::io::loadPCDFile(feature_dir, *pcl_feature_point_);

    LOG(INFO) << "\033[1;32m "<< str_g_map_ << " point size: " 
              <<  global_map_->size() <<  "\033[0m";

    LOG(INFO) << "\033[1;32m "<< str_f_map_ << " point size: " 
              <<  pcl_feature_point_->size() <<  "\033[0m";

    pcl::PointCloud<PointType>::Ptr map_ds(new pcl::PointCloud<PointType>()); 
    pcl::VoxelGrid<PointType> VoxelGridFilter;
    VoxelGridFilter.setLeafSize(load_eaf_size_, load_eaf_size_, load_eaf_size_);
    VoxelGridFilter.setInputCloud(pcl_feature_point_);
    VoxelGridFilter.filter(*map_ds);
    PointVector points_to_add;
    points_to_add.reserve(map_ds->points.size());
    for(size_t i=0; i < map_ds->points.size(); i++){
        PointType temp_p = map_ds->points[i];
        points_to_add.push_back(temp_p);
    }

    if(!split_map_){
        ivox_->AddPoints(points_to_add);  // load whole map.
    }else{
        std::string index_path(std::string(ROOTDIR + "maps/split_map/map_index.txt"));
        std::ifstream fin(index_path);
        while (!fin.eof()) {
            int x, y;
            fin >> x >> y;
            map_data_index_.emplace(Eigen::Vector2i(x, y));
        }
        fin.close();
    }
    
    pcl::toROSMsg(*map_ds, msg_feature_);
    msg_feature_.header.frame_id = "map";
    msg_feature_.header.stamp = ros::Time::now();

    #ifdef TEST_VOXEL
    LOG(INFO) << "\033[1;35m "<< str_f_map_ << " downsample point size: " 
              <<  map_ds->size() <<  "\033[0m";
    LOG(INFO) << "\033[1;35m " << " ivox_->NumValidGrids: " 
              <<  ivox_->NumValidGrids() <<  "\033[0m";
    #endif

    pcl::PointCloud<PointType>::Ptr glo_map_ds(new pcl::PointCloud<PointType>());
    VoxelGridFilter.setInputCloud(global_map_);
    VoxelGridFilter.filter(*glo_map_ds);
    pcl::toROSMsg(*glo_map_ds, msg_map_);
    msg_map_.header.frame_id = "map";
    msg_map_.header.stamp = ros::Time::now();

    #ifdef DEBUG
    LOG(INFO) << "\033[1;32m "<< str_g_map_ << " downsample point size: " 
              <<  map_ds->size() <<  "\033[0m";
    #endif

    LOG(INFO)<< "\033[1;32m Load GlobalMap done!\033[0m";
}


void LaserMapping::VisualMap(const ros::TimerEvent &e){
    if(pub_global_map_.getNumSubscribers() != 0){
        msg_map_.header.stamp = ros::Time::now();
        pub_global_map_.publish(msg_map_);
    }
    if(pub_feature_map_.getNumSubscribers() != 0){
        msg_feature_.header.stamp = ros::Time::now();
        pub_feature_map_.publish(msg_feature_);
    }
}


void LaserMapping::initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg)
{
    if(flg_location_inited_)
        return;
    init_lock_.lock();
        init_translation_[0] = pose_msg->pose.pose.position.x;
        init_translation_[1] = pose_msg->pose.pose.position.y;
        init_translation_[2] = 0.2;
        double x,y,z,w;
        x = pose_msg->pose.pose.orientation.x;
        y = pose_msg->pose.pose.orientation.y;
        z = pose_msg->pose.pose.orientation.z;
        w = pose_msg->pose.pose.orientation.w;
        init_rotation_ = Eigen::Quaterniond(w,x,y,z);
    init_lock_.unlock();
    flg_get_init_guess_ = true;
}


void LaserMapping::StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.push_back(ptr);
            time_buffer_.push_back(msg->header.stamp.toSec());
            last_timestamp_lidar_ = msg->header.stamp.toSec();
        },
        "Preprocess (Standard)");
    mtx_buffer_.unlock();
}

void LaserMapping::LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    mtx_buffer_.lock();
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            if (msg->header.stamp.toSec() < last_timestamp_lidar_) {
                LOG(WARNING) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            last_timestamp_lidar_ = msg->header.stamp.toSec();

            if (!time_sync_en_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0 && !imu_buffer_.empty() &&
                !lidar_buffer_.empty()) {
                LOG(INFO) << "IMU and LiDAR not Synced, IMU time: " << last_timestamp_imu_
                          << ", lidar header time: " << last_timestamp_lidar_;
            }

            if (time_sync_en_ && !timediff_set_flg_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1 &&
                !imu_buffer_.empty()) {
                timediff_set_flg_ = true;
                timediff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
                LOG(INFO) << "Self sync IMU and LiDAR, time diff is " << timediff_lidar_wrt_imu_;
            }

            PointCloudType::Ptr ptr(new PointCloudType());
            preprocess_->Process(msg, ptr);
            lidar_buffer_.emplace_back(ptr);
            time_buffer_.emplace_back(last_timestamp_lidar_);
        },
        "Preprocess (Livox)");

    mtx_buffer_.unlock();
}

void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count_++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu_ + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer_.lock();
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(msg);
    mtx_buffer_.unlock();
}

bool LaserMapping::SyncPackages() {
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        measures_.lidar_ = lidar_buffer_.front();
        measures_.lidar_bag_time_ = time_buffer_.front();

        if (measures_.lidar_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else if (measures_.lidar_->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_bag_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_bag_time_ + measures_.lidar_->points.back().curvature / double(1000);
            lidar_mean_scantime_ +=
                (measures_.lidar_->points.back().curvature / double(1000) - lidar_mean_scantime_) / scan_num_;
        }

        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu_ data, and pop from imu_ buffer ***/
    double imu_time = imu_buffer_.front()->header.stamp.toSec();
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time_) break;
        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void LaserMapping::PrintState(const state_ikfom &s) {
    LOG(INFO) << "state r: " << s.rot.coeffs().transpose() << ", t: " << s.pos.transpose()
              << ", off r: " << s.offset_R_L_I.coeffs().transpose() << ", t: " << s.offset_T_L_I.transpose();
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    int cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (int i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
        /* transform to world frame */
        PointBodyToWorld(&(scan_down_body_->points[i]), &(scan_down_world_->points[i]));

        /* decide if need add to map */
        PointType &point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector &points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = common::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= options::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < options::NUM_MATCH_POINTS; readd_i++) {
                    if (common::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

/**
 * Lidar point cloud registration
 * will be called by the eskf custom observation model
 * compute point-to-plane residual here
 * @param s kf state
 * @param ekfom_data H matrix
 */
void LaserMapping::ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    int selected_surf_count = 0;

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;
                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);
                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                        selected_surf_count ++;
                    }else{
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    feature_cloud_->clear();
    feature_cloud_->reserve(selected_surf_count);
    #ifdef DEBUG
        LOG(INFO) << "\033[1;34m "<< "============================================================= " << "\033[0m";
        LOG(INFO) << "\033[1;34m "<< "ObsModel: scan_down_body_ size: " << scan_down_body_->size() << "\033[0m";
        LOG(INFO) << "\033[1;34m "<< "ObsModel: feature_cloud_ size: " << feature_cloud_->size() << "\033[0m";
    #endif

    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];
            PointType temp_point = scan_down_body_->points[i];
            feature_cloud_->points.push_back(temp_point);
            effect_feat_num_++;
        }
    }
    feature_cloud_->points.shrink_to_fit();

    #ifdef DEBUG
        LOG(INFO) << "\033[1;32m "<< "selected_surf_count:  " << selected_surf_count << "  effect_feat_num_:  " << effect_feat_num_ << "\033[0m";
        LOG(INFO) << "\033[1;32m "<< "ObsModel: scan_down_body_ size: " << scan_down_body_->size() << "\033[0m";
        LOG(INFO) << "\033[1;32m "<< "ObsModel: feature_cloud_ size: " << feature_cloud_->size() << "\033[0m";
    #endif


    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

void LaserMapping::ObsModel_location(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data){
    int cnt_pts = scan_down_body_->size();

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            auto R_wl = (s.rot * s.offset_R_L_I).cast<float>();
            auto t_wl = (s.rot * s.offset_T_L_I + s.pos).cast<float>();

            /** closest surface search and residual computation **/
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) 
            {
                PointType &point_body = scan_down_body_->points[i];
                PointType &point_world = scan_down_world_->points[i];

                /* transform to world frame */
                common::V3F p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto &points_near = nearest_points_[i];
                if (ekfom_data.converge) {
                    /** Find the closest surfaces in the map **/
                    ivox_->GetClosestPoint(point_world, points_near, options::NUM_MATCH_POINTS);
                    point_selected_surf_[i] = points_near.size() >= options::MIN_NUM_MATCH_POINTS;

                    if (point_selected_surf_[i]) {
                        point_selected_surf_[i] =
                            common::esti_plane(plane_coef_[i], points_near, options::ESTI_PLANE_THRESHOLD);
                    }
                }
                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    bool valid_corr = p_body.norm() > 81 * pd2 * pd2;
                    if (valid_corr) {
                        point_selected_surf_[i] = true;
                        residuals_[i] = pd2;
                    }
                }

            });
        },
        "ObsModel (Lidar Match)");

    effect_feat_num_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_num_] = plane_coef_[i];
            corr_pts_[effect_feat_num_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_num_][3] = residuals_[i];

            effect_feat_num_++;
        }
    }
    corr_pts_.resize(effect_feat_num_);
    corr_norm_.resize(effect_feat_num_);

    if (effect_feat_num_ < 1) {
        ekfom_data.valid = false;
        LOG(WARNING) << "No Effective Points!";
        return;
    }

    Timer::Evaluate(
        [&, this]() {
            /*** Computation of Measurement Jacobian matrix H and measurements vector ***/
            ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_num_, 12);  // 23
            ekfom_data.h.resize(effect_feat_num_);

            index.resize(effect_feat_num_);
            const common::M3F off_R = s.offset_R_L_I.toRotationMatrix().cast<float>();
            const common::V3F off_t = s.offset_T_L_I.cast<float>();
            const common::M3F Rt = s.rot.toRotationMatrix().transpose().cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &i) {
                common::V3F point_this_be = corr_pts_[i].head<3>();
                common::M3F point_be_crossmat = SKEW_SYM_MATRIX(point_this_be);
                common::V3F point_this = off_R * point_this_be + off_t;
                common::M3F point_crossmat = SKEW_SYM_MATRIX(point_this);

                /*** get the normal vector of closest surface/corner ***/
                common::V3F norm_vec = corr_norm_[i].head<3>();

                /*** calculate the Measurement Jacobian matrix H ***/
                common::V3F C(Rt * norm_vec);
                common::V3F A(point_crossmat * C);

                if (extrinsic_est_en_) {
                    common::V3F B(point_be_crossmat * off_R.transpose() * C);
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], B[0],
                        B[1], B[2], C[0], C[1], C[2];
                } else {
                    ekfom_data.h_x.block<1, 12>(i, 0) << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2], 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0;
                }

                /*** Measurement: distance to the closest surface/corner ***/
                ekfom_data.h(i) = -corr_pts_[i][3];
            });
        },
        "    ObsModel (IEKF Build Jacobian)");
}

/////////////////////////////////////  debug save / show /////////////////////////////////////////////////////

void LaserMapping::PublishPath(const ros::Publisher pub_path) {
    SetPosestamp(msg_body_pose_);
    msg_body_pose_.header.stamp = ros::Time().fromSec(lidar_end_time_);
    msg_body_pose_.header.frame_id = "map";

    /*** if path is too large, the rvis will crash ***/
    path_.poses.push_back(msg_body_pose_);
    if (run_in_offline_ == false) {
        pub_path.publish(path_);
    }
}

void LaserMapping::PublishOdometry(const ros::Publisher &pub_odom_aft_mapped) {
    odom_aft_mapped_.header.frame_id = "map";
    odom_aft_mapped_.child_frame_id = "body";
    odom_aft_mapped_.header.stamp = ros::Time().fromSec(lidar_end_time_);  // ros::Time().fromSec(lidar_end_time_);
    SetPosestamp(odom_aft_mapped_.pose);
    pub_odom_aft_mapped.publish(odom_aft_mapped_);
    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odom_aft_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_aft_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_aft_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_aft_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_aft_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_aft_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::PoseStamped msg2uav;
    msg2uav.header.stamp = odom_aft_mapped_.header.stamp;
    msg2uav.header.frame_id = "map";
    msg2uav.pose = odom_aft_mapped_.pose.pose;
    pub_msg2uav_.publish(msg2uav);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_aft_mapped_.pose.pose.position.x, odom_aft_mapped_.pose.pose.position.y,
                                    odom_aft_mapped_.pose.pose.position.z));
    q.setW(odom_aft_mapped_.pose.pose.orientation.w);
    q.setX(odom_aft_mapped_.pose.pose.orientation.x);
    q.setY(odom_aft_mapped_.pose.pose.orientation.y);
    q.setZ(odom_aft_mapped_.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odom_aft_mapped_.header.stamp, "map", "body"));
}

void LaserMapping::PublishFrameWorld() {
    if (!(run_in_offline_ == false && scan_pub_en_) && !pcd_save_en_) {
        return;
    }

    PointCloudType::Ptr laserCloudWorld;
    if (dense_pub_en_) {
        PointCloudType::Ptr laserCloudFullRes(scan_undistort_);
        int size = laserCloudFullRes->points.size();
        laserCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }
    } else {
        laserCloudWorld = scan_down_world_;
    }

    PointCloudType::Ptr featureCloudWorld;
    {
        int size = feature_cloud_->points.size();
        featureCloudWorld.reset(new PointCloudType(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&feature_cloud_->points[i], &featureCloudWorld->points[i]);
        }
    }

    #ifdef DEBUG
        LOG(INFO) << "\033[1;32m "<< "PublishFrameWorld: scan_down_world_ size: " << scan_down_world_->size() << "\033[0m";
        LOG(INFO) << "\033[1;32m "<< "PublishFrameWorld: feature_cloud_ size: " << feature_cloud_->size() << "\033[0m";
    #endif

    if (run_in_offline_ == false && scan_pub_en_) {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
        laserCloudmsg.header.frame_id = "map";
        pub_laser_cloud_world_.publish(laserCloudmsg);
        publish_count_ -= options::PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performences
    if (pcd_save_en_) {
        *pcl_wait_save_ += *laserCloudWorld;
        *pcl_feature_point_ += *featureCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save_->size() > 0 && pcd_save_interval_ > 0 && scan_wait_num >= pcd_save_interval_) {
            pcd_index_++;
            std::string all_points_dir(std::string(ROOTDIR + "PCD/scans_") + std::to_string(pcd_index_) +
                                       std::string(".pcd"));
            pcl::PCDWriter pcd_writer;
            LOG(INFO) << "current scan saved to /PCD/" << all_points_dir;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
            pcl_wait_save_->clear();
            scan_wait_num = 0;
        }
    }
}

void LaserMapping::PublishFrameBody(const ros::Publisher &pub_laser_cloud_body) {
    int size = scan_undistort_->points.size();
    PointCloudType::Ptr laser_cloud_imu_body(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyLidarToIMU(&scan_undistort_->points[i], &laser_cloud_imu_body->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud_imu_body, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "body";
    pub_laser_cloud_body.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::PublishFrameEffectWorld(const ros::Publisher &pub_laser_cloud_effect_world) {
    int size = corr_pts_.size();
    PointCloudType::Ptr laser_cloud(new PointCloudType(size, 1));

    for (int i = 0; i < size; i++) {
        PointBodyToWorld(corr_pts_[i].head<3>(), &laser_cloud->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laser_cloud, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time_);
    laserCloudmsg.header.frame_id = "map";
    pub_laser_cloud_effect_world.publish(laserCloudmsg);
    publish_count_ -= options::PUBFRAME_PERIOD;
}

void LaserMapping::Savetrajectory(const std::string &traj_file) {
    std::ofstream ofs;
    ofs.open(traj_file, std::ios::out);
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open traj_file: " << traj_file;
        return;
    }

    ofs << "#timestamp x y z q_x q_y q_z q_w" << std::endl;
    for (const auto &p : path_.poses) {
        ofs << std::fixed << std::setprecision(6) << p.header.stamp.toSec() << " " << std::setprecision(15)
            << p.pose.position.x << " " << p.pose.position.y << " " << p.pose.position.z << " " << p.pose.orientation.x
            << " " << p.pose.orientation.y << " " << p.pose.orientation.z << " " << p.pose.orientation.w << std::endl;
    }

    ofs.close();
}

///////////////////////////  private method /////////////////////////////////////////////////////////////////////
template <typename T>
void LaserMapping::SetPosestamp(T &out) {
    out.pose.position.x = state_point_.pos(0);
    out.pose.position.y = state_point_.pos(1);
    out.pose.position.z = state_point_.pos(2);
    out.pose.orientation.x = state_point_.rot.coeffs()[0];
    out.pose.orientation.y = state_point_.rot.coeffs()[1];
    out.pose.orientation.z = state_point_.rot.coeffs()[2];
    out.pose.orientation.w = state_point_.rot.coeffs()[3];
}

void LaserMapping::PointBodyToWorld(const PointType *pi, PointType *const po) {
    common::V3D p_body(pi->x, pi->y, pi->z);
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void LaserMapping::PointBodyToWorld(const common::V3F &pi, PointType *const po) {
    common::V3D p_body(pi.x(), pi.y(), pi.z());
    common::V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) +
                         state_point_.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = std::abs(po->z);
}

void LaserMapping::PointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    common::V3D p_body_lidar(pi->x, pi->y, pi->z);
    common::V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void LaserMapping::Finish() {
    /**************** save map ****************/
    // 1. make sure you have enough memories
    // 2. pcd save will largely influence the real-time performences

    if(!pcd_save_en_){
        LOG(INFO) << "\033[1;32m "<< " finish done " << "\033[0m";
        return;
    }

    if (pcl_wait_save_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("GlobalMap.pcd");
        std::string all_points_dir(std::string(ROOTDIR + "maps/") + file_name);
        pcl::PCDWriter pcd_writer;
        #ifdef DEBUG
        LOG(INFO) << "\033[1;32m " << "current scan saved to /maps/" << file_name << "\033[0m";
        LOG(INFO) << "\033[1;36m "<< file_name << " point size: " 
              <<  pcl_wait_save_->size() <<  "\033[0m";
        #endif
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_);
    }

    if (pcl_feature_point_->size() > 0 && pcd_save_en_) {
        std::string file_name = std::string("FeatureMap.pcd");
        std::string all_points_dir(std::string(ROOTDIR + "maps/") + file_name);
        pcl::PCDWriter pcd_writer;
        #ifdef DEBUG
        LOG(INFO) << "\033[1;32m " << "current scan saved to /maps/" << file_name << "\033[0m";
        LOG(INFO) << "\033[1;36m " << file_name << " point size: " 
              <<  pcl_feature_point_->size() <<  "\033[0m";
        #endif
        pcd_writer.writeBinary(all_points_dir, *pcl_feature_point_);
        
        if(split_map_){
            SplitMap(pcl_feature_point_);
        }
    }

    LOG(INFO) << "\033[1;32m "<< " finish done " << "\033[0m";
}

void LaserMapping::SplitMap(CloudPtr map){
    float half_resolution = sub_grid_resolution_ /2;
    std::map<Eigen::Vector2i, PointCloudType::Ptr, less_vec<2>> map_data;  
    for (const auto& pt : map->points) {
        int gx = floor((pt.x - half_resolution) / sub_grid_resolution_);
        int gy = floor((pt.y - half_resolution) / sub_grid_resolution_);
        Eigen::Vector2i key(gx, gy);
        auto iter = map_data.find(key);
        if (iter == map_data.end()) {
            CloudPtr cloud(new PointCloudType);
            cloud->points.emplace_back(pt);
            cloud->is_dense = false;
            cloud->height = 1;
            map_data.emplace(key, cloud);
        } else {
            iter->second->points.emplace_back(pt);
        }
    }
    LOG(INFO) << "saving maps, grids: " << map_data.size();
    std::string save_dir(std::string(ROOTDIR + "maps/split_map/"));
    std::string mkdir_dir = "mkdir -p " + save_dir;
    std::string rm_rf = "rm -rf " + save_dir + "*";
    std::system(mkdir_dir.data());
    std::system(rm_rf.data());
    std::ofstream fout(save_dir + "/map_index.txt");
    for (auto& dp : map_data) {
        fout << dp.first[0] << " " << dp.first[1] << std::endl;
        dp.second->width = dp.second->size();
        dp.second->height = 1;
        std::string file_path = (save_dir + std::to_string(dp.first[0]) + "_" + std::to_string(dp.first[1]) + ".pcd");
        pcl::io::savePCDFileBinary(file_path, *dp.second);
    }
    fout.close();
}


///////////////////////////  render relative /////////////////////////////////////////////////////////////////////
void LaserMapping::SaveKeyFrame6D(){
    Cloud6D_t this_cloud;
    this_cloud.cloud.reset(new PointCloudType());
    *(this_cloud.cloud) = *scan_undistort_;
    this_cloud.time = lidar_end_time_;
    this_cloud.pose_q = state_point_.rot.coeffs();
    this_cloud.pose_t = state_point_.pos;
    mut_keyFrame_.lock();
        keyFrames_.push_back(this_cloud);
    mut_keyFrame_.unlock();
}


void LaserMapping::FovSegment_pub_thread(){
    ros::Rate loop(100);

    while(ros::ok()){
        mut_keyFrame_.lock();
        if(keyFrames_.empty()){
            mut_keyFrame_.unlock();
            loop.sleep();
            continue;
        }
        auto data = keyFrames_.front();
        keyFrames_.pop_front();
        mut_keyFrame_.unlock();

        // 根据相机视角进行数据裁剪.
        CloudPtr seg_cloud(new PointCloudType());
        seg_cloud->reserve(data.cloud->size()/2);

        for(auto& point : data.cloud->points){
            if(point.x < 0){
                continue;
            }

            if(abs(std::atan(point.y/point.x)) > M_PI_4 + 0.1){
                continue;
            }

            seg_cloud->push_back(point);
        }

        lvio_lite::KeyFrame kf_msg;
        pcl::toROSMsg(*seg_cloud, kf_msg.cloud);
        kf_msg.cloud.header.stamp = ros::Time().fromSec(data.time);
        kf_msg.cloud.header.frame_id = "body";
        kf_msg.header.stamp = ros::Time().fromSec(data.time);
        kf_msg.position.x = data.pose_t[0];
        kf_msg.position.y = data.pose_t[1];
        kf_msg.position.z = data.pose_t[2];
        kf_msg.quaternion.x = data.pose_q[0];
        kf_msg.quaternion.y = data.pose_q[1];
        kf_msg.quaternion.z = data.pose_q[2];
        kf_msg.quaternion.w = data.pose_q[3];
        pub_keyframe_.publish(kf_msg);
        loop.sleep();
    }
}



}  // namespace lvio_lite