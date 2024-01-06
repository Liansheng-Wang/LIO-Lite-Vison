

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;

DEFINE_string(pcd_path, "/home/dasheng/GitHub/LIO-Lite/src/LIO-Lite/maps/GlobalMap_copy.pcd", "点云文件路径");
DEFINE_double(image_resolution, 0.1, "俯视图分辨率");
DEFINE_double(min_z, -0.2, "俯视图最低高度");
DEFINE_double(max_z, 3.0, "俯视图最高高度");
DEFINE_double(origin_x, 0.0, "原点X坐标");
DEFINE_double(origin_y, 0.0, "原点Y坐标");

/// 本节演示如何将一个点云转换为俯视图像
void GenerateBEVImage(PointCloudType::Ptr cloud) {
    // 计算点云边界
    auto minmax_x = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.x < p2.x; });
    auto minmax_y = std::minmax_element(cloud->points.begin(), cloud->points.end(),
                                        [](const PointType& p1, const PointType& p2) { return p1.y < p2.y; });
    double min_x = minmax_x.first->x;
    double max_x = minmax_x.second->x;
    double min_y = minmax_y.first->y;
    double max_y = minmax_y.second->y;

    const double inv_r = 1.0 / FLAGS_image_resolution;

    const int image_rows = int((max_x - min_x) * inv_r);
    const int image_cols = int((max_y - min_y) * inv_r);

    float x_center = 0.5 * (max_x + min_x);
    float y_center = 0.5 * (max_y + min_y);
    float x_center_image = image_rows / 2;
    float y_center_image = image_cols / 2;

    float _x_center_image = (max_x - FLAGS_origin_x) * inv_r;
    float _y_center_image = (max_y - FLAGS_origin_y) * inv_r;

    // 生成图像
    cv::Mat image(image_rows, image_cols, CV_8UC3, cv::Scalar(255, 255, 255));

    for (const auto& pt : cloud->points) {
        int x = int(x_center_image - (pt.x - x_center) * inv_r);
        int y = int(y_center_image - (pt.y - y_center) * inv_r);
        if (x < 0 || x >= image_rows || y < 0 || y >= image_cols || pt.z < FLAGS_min_z || pt.z > FLAGS_max_z) {
            continue;
        }
        image.at<cv::Vec3b>(x, y) = cv::Vec3b(227, 143, 79);   // 可以设置颜色
    }

    cv::imwrite("/home/dasheng/GitHub/LIO-Lite/src/LIO-Lite/maps/bev.png", image);

    cv::line(image, cv::Point(_y_center_image, _x_center_image), cv::Point(_y_center_image, 0), cv::Scalar(0, 0, 255), 2); // X轴为红色
    cv::line(image, cv::Point(_y_center_image, _x_center_image), cv::Point(0, _x_center_image), cv::Scalar(0, 255, 0), 2); // Y轴为绿色

    cv::imwrite("/home/dasheng/GitHub/LIO-Lite/src/LIO-Lite/maps/bev_axis.png", image);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_pcd_path.empty()) {
        LOG(ERROR) << "pcd path is empty";
        return -1;
    }

    // 读取点云
    PointCloudType::Ptr cloud(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_pcd_path, *cloud);

    if (cloud->empty()) {
        LOG(ERROR) << "cannot load cloud file";
        return -1;
    }

    LOG(INFO) << "cloud points: " << cloud->size();
    GenerateBEVImage(cloud);

    return 0;
}
