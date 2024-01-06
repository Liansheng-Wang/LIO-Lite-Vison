
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


std::string ROOTDIR = std::string(ROOT_DIR);
std::string MAP_PATH = ROOTDIR + "maps/FeatureMap.pcd";
std::string SAVE_DIR = ROOTDIR + "maps/test_split/";

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template<typename PointCloudType> 
void SaveCloudToFile(const std::string &filePath, PointCloudType &cloud) {
    cloud.height = 1;
    cloud.width = cloud.size();
    pcl::io::savePCDFileBinary(filePath, cloud);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::map<Eigen::Vector2i, PointCloudType::Ptr, less_vec<2>> map_data;
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    float resolution = 0.5;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::io::loadPCDFile(MAP_PATH, *cloud);

    LOG(INFO) << "cloud.size:  " << cloud->size() << std::endl;

    PointCloudType::Ptr kf_cloud_voxeled(new PointCloudType);
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.filter(*kf_cloud_voxeled);

    LOG(INFO) << "kf_cloud_voxeled.size:  " << kf_cloud_voxeled->size() << std::endl;

    // split
    for (const auto& pt : kf_cloud_voxeled->points) {
        int gx = floor((pt.x - 50.0) / 100);
        int gy = floor((pt.y - 50.0) / 100);
        Eigen::Vector2i key(gx, gy);
        auto iter = map_data.find(key);
        if (iter == map_data.end()) {
          PointCloudType::Ptr cloud(new PointCloudType);
          cloud->points.emplace_back(pt);
          cloud->is_dense = false;
          cloud->height = 1;
          map_data.emplace(key, cloud);
        } else {
          iter->second->points.emplace_back(pt);
        }
    }

    // save
    LOG(INFO) << "saving maps, grids: " << map_data.size();

    std::string mkdir_dir = "mkdir -p " + SAVE_DIR;
    std::string rm_rf = "rm -rf " + SAVE_DIR + "*";
    std::system(mkdir_dir.data());
    std::system(rm_rf.data());
    std::ofstream fout(SAVE_DIR + "map_index.txt");
    for (auto& dp : map_data) {
      fout << dp.first[0] << " " << dp.first[1] << std::endl;
      dp.second->width = dp.second->size();
      SaveCloudToFile(
      SAVE_DIR + std::to_string(dp.first[0]) + "_" + std::to_string(dp.first[1]) + ".pcd",
      *dp.second);
    }
    fout.close();

    LOG(INFO) << "done.";
    return 0;
}
