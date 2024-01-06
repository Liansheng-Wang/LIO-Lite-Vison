
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/random.h>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


std::string ROOTDIR = std::string(ROOT_DIR);
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
  pcl::io::savePCDFileASCII(filePath, cloud);
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::map<Eigen::Vector2i, PointCloudType::Ptr, less_vec<2>> map_data;

  std::set<Eigen::Vector2i, less_vec<2>> map_data_index_; 

  std::ifstream fin(SAVE_DIR + "map_index.txt");
  while (!fin.eof()) {
    int x, y;
    fin >> x >> y;
    map_data_index_.emplace(Eigen::Vector2i(x, y));
  }
  fin.close();

  std::cout<< "0: load done:  " << map_data_index_.size() << std::endl;

  int count = 0;


  for (auto& k : map_data_index_) {
    count++;
    auto t1 = std::chrono::high_resolution_clock::now();
    PointCloudType::Ptr cloud(new PointCloudType);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    std::cout << "1: load " << count << " / " << map_data_index_.size() << "  t1: " << time_used << std::endl;
    pcl::io::loadPCDFile(SAVE_DIR + std::to_string(k[0]) + "_" + std::to_string(k[1]) + ".pcd", *cloud);
    t1 = std::chrono::high_resolution_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count() * 1000;
    std::cout << "1: load " << count << " / " << map_data_index_.size() << "  t2: " << time_used << std::endl;
    map_data.emplace(k, cloud);
    t2 = std::chrono::high_resolution_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    std::cout << "1: load " << count << " / " << map_data_index_.size() << "  t3: " << time_used << std::endl;
    std::cout << "1: load " << count << " / " << map_data_index_.size() << "  done." << std::endl;
  }

  std::cout<< "1: load done." << std::endl;

  auto t1 = std::chrono::high_resolution_clock::now();
  PointCloudType::Ptr cloud(new PointCloudType);
  pcl::io::loadPCDFile("/home/dasheng/GitHub/LIO-Lite/src/LIO-Lite/maps/GlobalMap_copy.pcd", *cloud);
  auto t2 = std::chrono::high_resolution_clock::now();
  auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
  std::cout << "n: load " << time_used << std::endl;


  // 依据不同的颜色对 map_data 中的点云进行拼接和显示。
  pcl::visualization::PCLVisualizer viewer("Map Viewer");
  viewer.setBackgroundColor(0, 0, 0);

  int color_id = 0;
  for (const auto& kv : map_data) {
      PointCloudType::Ptr cloud = kv.second;

      // 为当前小块的点云分配随机颜色
      double r = static_cast<double>(rand()) / RAND_MAX;
      double g = static_cast<double>(rand()) / RAND_MAX;
      double b = static_cast<double>(rand()) / RAND_MAX;

      pcl::visualization::PointCloudColorHandlerCustom<PointType> handler(cloud, (int)(r * 255), (int)(g * 255), (int)(b * 255));

      viewer.addPointCloud(cloud, handler, "cloud_" + std::to_string(color_id));

      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_" + std::to_string(color_id));

      std::cout << "2: load " << color_id << " / " << map_data.size() << "  done." << std::endl;

      color_id++;
  }

  std::cout<< "2: load done." << std::endl;

  viewer.spin();

  LOG(INFO) << "done.";
  return 0;
}


