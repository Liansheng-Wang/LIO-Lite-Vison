import open3d as o3d

# 读取PCD文件
pcd = o3d.io.read_point_cloud("pjlab_6.pcd")

# 可视化原始点云
o3d.visualization.draw_geometries([pcd])

# 进行离群点去除
pcd_filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=15, std_ratio=1.5)

# 可视化处理后的点云
o3d.visualization.draw_geometries([pcd_filtered])

# 保存处理后的点云
o3d.io.write_point_cloud("output.pcd", pcd_filtered)

