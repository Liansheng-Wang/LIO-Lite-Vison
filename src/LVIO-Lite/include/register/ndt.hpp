#ifndef SLAM_ULTRA_NDT_H_
#define SLAM_ULTRA_NDT_H_


#include <cmath>
#include <vector>
#include <execution>
#include <unordered_map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "so3_math.h"
#include "custom_logs.hpp"


namespace P2P{

using PointT = pcl::PointXYZINormal;
using PointCloudT = pcl::PointCloud<PointT>;
using CloudPtr = PointCloudT::Ptr;

using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec15d = Eigen::Matrix<double, 15, 1>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat3d = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Mat4d = Eigen::Matrix4d;
using Mat4f = Eigen::Matrix4f;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat15d = Eigen::Matrix<double, 15, 15>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;

template <int N>
struct hash_vec {
  inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943)) % 10000000);
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0] * 73856093) ^ (v[1] * 471943) ^ (v[2] * 83492791)) % 10000000);
}

inline Vec3d ToVec3d(const PointT& pt) { return pt.getVector3fMap().cast<double>(); }


/**
 * 计算一个容器内数据的均值与矩阵形式协方差
 * @tparam C    容器类型
 * @tparam int 　数据维度
 * @tparam Getter   获取数据函数, 接收一个容器内数据类型，返回一个Eigen::Matrix<double, dim,1> 矢量类型
 */
template <typename C, int dim, typename Getter>
inline void ComputeMeanAndCov(const C& data, Eigen::Matrix<double, dim, 1>& mean, Eigen::Matrix<double, dim, dim>& cov,
                       Getter&& getter) {
    using D = Eigen::Matrix<double, dim, 1>;
    using E = Eigen::Matrix<double, dim, dim>;
    size_t len = data.size();
    assert(len > 1);

    mean = std::accumulate(data.begin(), data.end(), Eigen::Matrix<double, dim, 1>::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov = std::accumulate(data.begin(), data.end(), E::Zero().eval(),
                          [&mean, &getter](const E& sum, const auto& data) -> E {
                              D v = getter(data) - mean;
                              return sum + v * v.transpose();
                          }) / (len - 1);
}

// template<typename PointT>
class Ndt3d {
public:
  enum class NearbyType {
    CENTER,   // 只考虑中心
    NEARBY6,  // 上下左右前后
  };

  struct Options {
    int max_iteration_ = 20;        // 最大迭代次数
    double voxel_size_ = 1.0;       // 体素大小
    double inv_voxel_size_ = 1.0;   //
    int min_effective_pts_ = 10;    // 最近邻点数阈值
    size_t min_pts_in_voxel_ = 3;      // 每个栅格中最小点数
    double eps_ = 1e-2;             // 收敛判定条件
    double res_outlier_th_ = 20.0;  // 异常值拒绝阈值
    bool remove_centroid_ = false;  // 是否计算两个点云中心并移除中心？

    NearbyType nearby_type_ = NearbyType::NEARBY6;
  };

  using KeyType = Vec3i;  // 体素的索引
  struct VoxelData {
    VoxelData() {}
    VoxelData(size_t id) { idx_.emplace_back(id); }

    std::vector<size_t> idx_;      // 点云中点的索引
    Vec3d mu_ = Vec3d::Zero();     // 均值
    Mat3d sigma_ = Mat3d::Zero();  // 协方差
    Mat3d info_ = Mat3d::Zero();   // 协方差之逆
  };

  Ndt3d() {
    options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
    GenerateNearbyGrids();
  }

  Ndt3d(Options options) : options_(options) {
    options_.inv_voxel_size_ = 1.0 / options_.voxel_size_;
    GenerateNearbyGrids();
  }

  /// 设置目标的Scan
  void SetTarget(CloudPtr target) {
    target_ = target;
    BuildVoxels();

    // 计算点云中心
    target_center_ = std::accumulate(target->points.begin(), target_->points.end(), Vec3d::Zero().eval(),
                                      [](const Vec3d& c, const PointT& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                      target_->size();
  }

  /// 设置被配准的Scan
  void SetSource(CloudPtr source) {
    source_ = source;

    source_center_ = std::accumulate(source_->points.begin(), source_->points.end(), Vec3d::Zero().eval(),
                                      [](const Vec3d& c, const PointT& pt) -> Vec3d { return c + ToVec3d(pt); }) /
                      source_->size();
  }

  void SetGtPose(const Eigen::Affine3d& gt_pose) {
    gt_pose_ = gt_pose;
    gt_set_ = true;
  }

  /// 使用gauss-newton方法进行ndt配准
  inline bool AlignNdt(Eigen::Affine3d& init_pose);

private:
  inline void BuildVoxels();

  /// 根据最近邻的类型，生成附近网格
  inline void GenerateNearbyGrids();

  CloudPtr target_ = nullptr;
  CloudPtr source_ = nullptr;

  Vec3d target_center_ = Vec3d::Zero();
  Vec3d source_center_ = Vec3d::Zero();

  Eigen::Affine3d gt_pose_;
  bool gt_set_ = false;

  Options options_;

  std::unordered_map<KeyType, VoxelData, hash_vec<3>> grids_;  // 栅格数据
  std::vector<KeyType> nearby_grids_;                          // 附近的栅格
};

inline void Ndt3d::BuildVoxels() {
  assert(target_ != nullptr);
  assert(target_->empty() == false);
  grids_.clear();

  /// 分配体素
  std::vector<size_t> index(target_->size());
  std::for_each(index.begin(), index.end(), [idx = 0](size_t& i) mutable { i = idx++; });

  std::for_each(index.begin(), index.end(), [this](const size_t& idx) {
    auto pt = ToVec3d(target_->points[idx]);
    auto key = (pt * options_.inv_voxel_size_).cast<int>();
    if (grids_.find(key) == grids_.end()) {
      grids_.insert({key, {idx}});
    } else {
      grids_[key].idx_.emplace_back(idx);
    }
  });

  /// 计算每个体素中的均值和协方差
  std::for_each(std::execution::par_unseq, grids_.begin(), grids_.end(), [this](auto& v) {
    if (v.second.idx_.size() > options_.min_pts_in_voxel_) {
      // 要求至少有３个点
      ComputeMeanAndCov(v.second.idx_, v.second.mu_, v.second.sigma_,
                              [this](const size_t& idx) { return ToVec3d(target_->points[idx]); });
      // SVD 检查最大与最小奇异值，限制最小奇异值

      Eigen::JacobiSVD svd(v.second.sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Vec3d lambda = svd.singularValues();
      if (lambda[1] < lambda[0] * 1e-3) {
          lambda[1] = lambda[0] * 1e-3;
      }

      if (lambda[2] < lambda[0] * 1e-3) {
          lambda[2] = lambda[0] * 1e-3;
      }

      Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();

      // v.second.info_ = (v.second.sigma_ + Mat3d::Identity() * 1e-3).inverse();  // 避免出nan
      v.second.info_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
  });

  /// 删除点数不够的
  for (auto iter = grids_.begin(); iter != grids_.end();) {
    if (iter->second.idx_.size() > options_.min_pts_in_voxel_) {
      iter++;
    } else {
      iter = grids_.erase(iter);
    }
  }
}

// template<typename PointT>
inline bool Ndt3d::AlignNdt(Eigen::Affine3d& init_pose) {
  LOG(INFO) << "aligning with ndt";
  assert(grids_.empty() == false);

  Eigen::Affine3d pose = init_pose;
  if (options_.remove_centroid_) {
    pose.translation() = target_center_ - source_center_;  // 设置平移初始值
    LOG(INFO) << "init trans set to " << pose.translation().transpose();
  }

  // 对点的索引，预先生成
  int num_residual_per_point = 1;
  if (options_.nearby_type_ == NearbyType::NEARBY6) {
    num_residual_per_point = 7;
  }

  std::vector<int> index(source_->points.size());
  for (size_t i = 0; i < index.size(); ++i) {
    index[i] = i;
  }

  int total_size = index.size() * num_residual_per_point;

  for (int iter = 0; iter < options_.max_iteration_; ++iter) {
    std::vector<bool> effect_pts(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(total_size);
    std::vector<Vec3d> errors(total_size);
    std::vector<Mat3d> infos(total_size);

    // gauss-newton 迭代
    // 最近邻，可以并发
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
      auto q = ToVec3d(source_->points[idx]);
      Vec3d qs = pose * q;  // 转换之后的q

      // 计算qs所在的栅格以及它的最近邻栅格
      Vec3i key = (qs * options_.inv_voxel_size_).cast<int>();

      for (size_t i = 0; i < nearby_grids_.size(); ++i) {
        auto key_off = key + nearby_grids_[i];
        auto it = grids_.find(key_off);
        int real_idx = idx * num_residual_per_point + i;
        if (it != grids_.end()) {
          auto& v = it->second;  // voxel
          Vec3d e = qs - v.mu_;

          // check chi2 th
          double res = e.transpose() * v.info_ * e;
          if (std::isnan(res) || res > options_.res_outlier_th_) {
              effect_pts[real_idx] = false;
              continue;
          }

          // build residual
          Eigen::Matrix<double, 3, 6> J;
          J.block<3, 3>(0, 0) = -pose.rotation() * lvio_lite::SKEW_SYM_MATRIX(q);
          J.block<3, 3>(0, 3) = Mat3d::Identity();

          jacobians[real_idx] = J;
          errors[real_idx] = e;
          infos[real_idx] = v.info_;
          effect_pts[real_idx] = true;
        } else {
          effect_pts[real_idx] = false;
        }
      }
    });

    // 累加Hessian和error,计算dx
    // 原则上可以用reduce并发，写起来比较麻烦，这里写成accumulate
    double total_res = 0;
    int effective_num = 0;

    Mat6d H = Mat6d::Zero();
    Vec6d err = Vec6d::Zero();

    for (size_t idx = 0; idx < effect_pts.size(); ++idx) {
      if (!effect_pts[idx]) {
          continue;
      }

      total_res += errors[idx].transpose() * infos[idx] * errors[idx];
      // chi2.emplace_back(errors[idx].transpose() * infos[idx] * errors[idx]);
      effective_num++;

      H += jacobians[idx].transpose() * infos[idx] * jacobians[idx];
      err += -jacobians[idx].transpose() * infos[idx] * errors[idx];
    }

    if (effective_num < options_.min_effective_pts_) {
      LOG(WARNING) << "effective num too small: " << effective_num;
      return false;
    }

    Vec6d dx = H.inverse() * err;
    pose.rotate(pose.rotation() * lvio_lite::Exp<double>(dx.head<3>()));
    pose.translation() += dx.tail<3>();

    // 更新
    // LOG(INFO) << "iter " << iter << " total res: " << total_res << ", eff: " << effective_num
    //           << ", mean res: " << total_res / effective_num << ", dxn: " << dx.norm()
    //           << ", dx: " << dx.transpose();

    if (gt_set_) {
      double pose_error = lvio_lite::Log<double>((gt_pose_.inverse() * pose).rotation()).norm();
      LOG(INFO) << "iter " << iter << " pose error: " << pose_error;
    }

    if (dx.norm() < options_.eps_) {
      LOG(INFO) << "converged, dx = " << dx.transpose();
      break;
    }
  }

  init_pose = pose;
  return true;
}


// template<typename PointT>
inline void Ndt3d::GenerateNearbyGrids() {
  if (options_.nearby_type_ == NearbyType::CENTER) {
    nearby_grids_.emplace_back(KeyType::Zero());
  } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
    nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                     KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
  }
}

}

#endif