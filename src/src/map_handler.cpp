#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>  // Include the Odometry message
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <queue>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class OctoMapToGridMap {
public:
    OctoMapToGridMap() : nh_("~"), obstacle_tree_(0.4), free_tree_(0.4), scan_tree_(0.4), tf_listener_(tf_buffer_){
        nh_.param("grid_size_m", grid_size_m_, 40.0); // Default grid size 40m x 40m
        nh_.param("z_range", z_range_, 1.5); // Default Z range 1.5m
        nh_.param("resolution", grid_res_, 0.1); // Default resolution 0.1m
        nh_.param("sensor_range", sensor_range_, 20.0); // Default sensor range 20m

        nh_.param("hit_probability", hit_probability_, 0.7f); // Default hit probability
        nh_.param("miss_probability", miss_probability_, 0.4f); // Default miss probability
        nh_.param("only_ground_free", only_ground_free_, false);
        nh_.param("hit_max", hit_max_, 1.0f);
        nh_.param("miss_min", miss_min_, 0.1f);
        nh_.param("remove_dyn_obs", remove_dyn_obs_, true);
        nh_.param("obstacle_height_thr", obstacle_height_thr_, 0.2f);

        nh_.param("use_bounding_box", use_bounding_box_, false);
        nh_.param("bounding_box_min_x", bounding_box_x_min_, -20.0);
        nh_.param("bounding_box_max_x", bounding_box_x_max_, 20.0);
        nh_.param("bounding_box_min_y", bounding_box_y_min_, -20.0);
        nh_.param("bounding_box_max_y", bounding_box_y_max_, 20.0);

        ROS_INFO("Bounding Box: min_x=%f, max_x=%f, min_y=%f, max_y=%f", bounding_box_x_min_, bounding_box_x_max_, bounding_box_y_min_, bounding_box_y_max_);

        obstacle_tree_.setResolution(grid_res_);
        free_tree_.setResolution(grid_res_);
        scan_tree_.setResolution(grid_res_);

        obstacle_tree_.setProbHit(hit_probability_);
        obstacle_tree_.setProbMiss(miss_probability_);

        free_tree_.setProbHit(hit_probability_);
        free_tree_.setProbMiss(miss_probability_);

        scan_tree_.setProbHit(hit_probability_);
        scan_tree_.setProbMiss(miss_probability_);

        obstacle_tree_.setClampingThresMax(hit_max_);
        obstacle_tree_.setClampingThresMin(miss_min_);
        // obstacle_tree_.setOccupancyThres(0.5f);

        grid_size_ = static_cast<int>(grid_size_m_ / grid_res_);
        
        // Use ApproximateTime for synchronizer
        terrain_pointcloud_sub_.subscribe(nh_, "/terrain_cloud", 10000);
        pointcloud_sub_.subscribe(nh_, "/registered_scan", 10000);
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), terrain_pointcloud_sub_, pointcloud_sub_));
        
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(10), terrain_pointcloud_sub_, pointcloud_sub_));
        sync_->registerCallback(boost::bind(&OctoMapToGridMap::pointCloudCallback, this, _1, _2));
        
        gridmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& terrain_msg,
                        const sensor_msgs::PointCloud2::ConstPtr& scan_msg) {
        // Start measuring time
        ros::Time start_time = ros::Time::now();
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform("map", "sensor", scan_msg->header.stamp, ros::Duration(1));
        } catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Failed to transform scan_msg from map to sensor: " << ex.what());
            return;
        }

        robot_x_ = transform_stamped.transform.translation.x;
        robot_y_ = transform_stamped.transform.translation.y;
        robot_z_ = transform_stamped.transform.translation.z;

        pcl::PointCloud<pcl::PointXYZI> scan_pcl_cloud;
        pcl::fromROSMsg(*scan_msg, scan_pcl_cloud);

        tf2::Transform tf2_transform;
        tf2::fromMsg(transform_stamped.transform, tf2_transform);

        // Compute the inverse transform
        tf2::Transform tf2_inv_transform = tf2_transform.inverse();

        // Extract inverse rotation (quaternion)
        tf2::Quaternion q_inv = tf2_inv_transform.getRotation();
        Eigen::Quaternionf q(q_inv.w(), q_inv.x(), q_inv.y(), q_inv.z());

        // Extract inverse translation
        tf2::Vector3 t_inv = tf2_inv_transform.getOrigin();
        Eigen::Vector3f t(t_inv.x(), t_inv.y(), t_inv.z());

        // Construct transformation matrix
        Eigen::Matrix4f map_to_sensor_matrix = Eigen::Matrix4f::Identity();
        map_to_sensor_matrix.block<3,3>(0,0) = q.toRotationMatrix();
        map_to_sensor_matrix.block<3,1>(0,3) = t;

        pcl::transformPointCloud(scan_pcl_cloud, scan_pcl_cloud, map_to_sensor_matrix);

        pcl::PointCloud<pcl::PointXYZI> scan_cloud_clipped;
        for (const auto &pt : scan_pcl_cloud.points) {
            if (pt.z >= -z_range_ && pt.z <= z_range_) {
                if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
                    continue;
                scan_cloud_clipped.push_back(pt);
            }
        }

        Eigen::Matrix4f sensor_to_map_matrix = map_to_sensor_matrix.inverse();

        pcl::PointCloud<pcl::PointXYZI> scan_cloud_filtered;
        pcl::transformPointCloud(scan_cloud_clipped, scan_cloud_filtered, sensor_to_map_matrix);

        octomap::Pointcloud scan_cloud;
        for (const auto& point : scan_cloud_filtered.points) {
            octomap::point3d p(point.x, point.y, point.z);
            scan_cloud.push_back(p);
        }

        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZI> terrain_cloud;
        pcl::fromROSMsg(*terrain_msg, terrain_cloud);
        if (terrain_cloud.empty()) {
            return;
        }
        octomap::point3d robot_position(robot_x_, robot_y_, robot_z_);

        // Process the downsampled point cloud
        octomap::Pointcloud free_cloud;
        octomap::Pointcloud occupied_cloud;
        for (const auto& point : terrain_cloud.points) {
            // Calculate the distance from the robot to the point
            if (isnan(point.x) || isnan(point.y) || isnan(point.z))
                continue;
            octomap::point3d p(point.x, point.y, point.z);
            if (point.intensity < obstacle_height_thr_) {
                free_cloud.push_back(p);
            }
            else {
                occupied_cloud.push_back(p);
            }
        }

        if (remove_dyn_obs_) {
                    InsertFreeScan(robot_position, free_cloud, scan_cloud, obstacle_tree_);
        InsertFreeScan(robot_position, scan_cloud, scan_cloud, obstacle_tree_);
        }

        obstacle_tree_.insertPointCloud(occupied_cloud, robot_position, sensor_range_, false);

        // obstacle_tree_.updateInnerOccupancy();

        free_tree_.insertPointCloud(free_cloud, robot_position, sensor_range_, true);
        // InsertScan(robot_position, free_cloud, free_tree_, false);
        free_tree_.updateInnerOccupancy();

        if (not only_ground_free_) {
            scan_tree_.insertPointCloud(scan_cloud, robot_position, sensor_range_, true);
            scan_tree_.updateInnerOccupancy();
        }

        ros::Time octree_time = ros::Time::now();
        // Convert the OctoMap to 2D grid map and publish
        publishGridMap();
        ros::Time end_time = ros::Time::now();
        ROS_INFO_STREAM("Time taken to convert point cloud to grid map: " << (end_time - start_time).toSec() << " seconds");
        ROS_INFO_STREAM("Time taken to update OctoMap: " << (octree_time - start_time).toSec() << " seconds");
        ROS_INFO_STREAM("Time taken to publish grid map: " << (end_time - octree_time).toSec() << " seconds");
    }

    void InsertScan(const octomap::point3d& robot_position, const octomap::Pointcloud& cloud, octomap::OcTree& tree, bool free_scan) {
        octomap::KeySet free_cells, occupied_cells;
        for (const auto& point : cloud) {
            bool is_fake_point = false;
            octomap::point3d effective_point = point;

            if ((point - robot_position).norm() > sensor_range_) {
                is_fake_point = true;
                octomap::point3d direction = (point - robot_position).normalized(); // 计算方向
                effective_point = robot_position + direction * sensor_range_;
            }

            octomap::KeyRay key_ray;
            if (tree.computeRayKeys(robot_position, effective_point, key_ray)){
                if (free_scan) {
                    free_cells.insert(key_ray.begin(), key_ray.end() - 2);
                }
                else {
                    free_cells.insert(key_ray.begin(), key_ray.end());
                }
            }
            octomap::OcTreeKey key;
            if (not is_fake_point and not free_scan) {
                tree.coordToKeyChecked(point, key);
                occupied_cells.insert(key);
            }
        }

        for (octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
            if (occupied_cells.find(*it) == occupied_cells.end()){
                tree.updateNode(*it, false, true);
            }
        }

        for (octomap::KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
//            tree.updateNode(*it, true, true);
            tree.setNodeValue(*it, 1, true);
        }
    }

    void InsertFreeScan(const octomap::point3d& robot_position, const octomap::Pointcloud& free_cloud, const octomap::Pointcloud& occupied_cloud, octomap::OcTree& tree) {
        octomap::KeySet free_cells, occupied_cells;;
        for (const auto& point : free_cloud) {
            octomap::point3d effective_point = point;
            octomap::KeyRay key_ray;
            if (tree.computeRayKeys(robot_position, effective_point, key_ray)){
                free_cells.insert(key_ray.begin(), key_ray.end());
            }
        }

        for (const auto& point : occupied_cloud) {
            octomap::OcTreeKey key;
            tree.coordToKeyChecked(point, key);
            occupied_cells.insert(key);
        }

        for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ){
            if (occupied_cells.find(*it) != occupied_cells.end()){
                it = free_cells.erase(it);
            } else {
                ++it;
            }
        }

        for (octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
            // tree.setNodeValue(*it, -tree.getOccupancyThres(), true);
            tree.updateNode(*it, false, false);
        }
    }

    void publishGridMap() {
        double rounded_robot_x = round(robot_x_ / grid_res_) * grid_res_;
        double rounded_robot_y = round(robot_y_ / grid_res_) * grid_res_;

        double min_x, min_y, free_min_z, max_x, max_y, free_max_z;
        free_tree_.getMetricMin(min_x, min_y, free_min_z);
        free_tree_.getMetricMax(max_x, max_y, free_max_z);

        double min_x_, min_y_, occupied_min_z, max_x_, max_y_, occupied_max_z;
        obstacle_tree_.getMetricMin(min_x_, min_y_, occupied_min_z);
        obstacle_tree_.getMetricMax(max_x_, max_y_, occupied_max_z);

        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "map";
        grid_msg.info.resolution = grid_res_;
        grid_msg.info.width = grid_size_;
        grid_msg.info.height = grid_size_;
        grid_msg.info.origin.position.x = rounded_robot_x - grid_size_m_ / 2.0;
        grid_msg.info.origin.position.y = rounded_robot_y - grid_size_m_ / 2.0;
        grid_msg.info.origin.position.z = robot_z_ - 0.75;
        grid_msg.data.assign(grid_size_ * grid_size_, -1);
        double inv_res = 1.0 / grid_res_;
        double origin_x = rounded_robot_x - grid_size_m_ / 2.0;
        double origin_y = rounded_robot_y - grid_size_m_ / 2.0;

        octomap::point3d occupied_min_point(origin_x, origin_y, occupied_min_z);
        octomap::point3d occupied_max_point(origin_x + grid_size_m_, origin_y + grid_size_m_, occupied_max_z);

        octomap::point3d free_min_point(origin_x, origin_y, free_min_z);
        octomap::point3d free_max_point(origin_x + grid_size_m_, origin_y + grid_size_m_, free_max_z);

        // -------- Free Tree --------
        for (auto it = free_tree_.begin_leafs_bbx(free_min_point, free_max_point),
                  end = free_tree_.end_leafs_bbx(); it != end; ++it) {

            int x = static_cast<int>((it.getX() - origin_x) * inv_res);
            int y = static_cast<int>((it.getY() - origin_y) * inv_res);
            double size = it.getSize();

            if (x < 0 || x >= grid_size_ || y < 0 || y >= grid_size_) continue;

            if (size <= grid_res_) {
                grid_msg.data[y * grid_size_ + x] = 0; // Free
            } else {
                int scale = std::ceil(size / grid_res_);
                int half_scale = scale / 2;
                for (int dx = -half_scale; dx <= half_scale; ++dx) {
                    for (int dy = -half_scale; dy <= half_scale; ++dy) {
                        int ix = x + dx;
                        int iy = y + dy;
                        if (ix >= 0 && ix < grid_size_ && iy >= 0 && iy < grid_size_) {
                            grid_msg.data[iy * grid_size_ + ix] = 0;
                        }
                    }
                }
            }
        }

        // -------- Scan Tree --------
        if (not only_ground_free_) {
            for (auto it = scan_tree_.begin_leafs_bbx(occupied_min_point, occupied_max_point),
                      end = scan_tree_.end_leafs_bbx(); it != end; ++it) {

                if (it->getOccupancy() >= scan_tree_.getOccupancyThres()) continue;

                int x = static_cast<int>((it.getX() - origin_x) * inv_res);
                int y = static_cast<int>((it.getY() - origin_y) * inv_res);
                double size = it.getSize();

                if (x < 0 || x >= grid_size_ || y < 0 || y >= grid_size_) continue;

                if (size <= grid_res_) {
                    grid_msg.data[y * grid_size_ + x] = 0;
                } else {
                    int scale = std::ceil(size / grid_res_);
                    int half_scale = scale / 2;
                    for (int dx = -half_scale; dx <= half_scale; ++dx) {
                        for (int dy = -half_scale; dy <= half_scale; ++dy) {
                            int ix = x + dx;
                            int iy = y + dy;
                            if (ix >= 0 && ix < grid_size_ && iy >= 0 && iy < grid_size_) {
                                grid_msg.data[iy * grid_size_ + ix] = 0;
                            }
                        }
                    }
                }
            }
        }

        // -------- Obstacle Tree --------
        for (auto it = obstacle_tree_.begin_leafs_bbx(occupied_min_point, occupied_max_point),
                  end = obstacle_tree_.end_leafs_bbx(); it != end; ++it) {

            if (!obstacle_tree_.isNodeOccupied(*it)) continue;

            int x = static_cast<int>((it.getX() - origin_x) * inv_res);
            int y = static_cast<int>((it.getY() - origin_y) * inv_res);
            double size = it.getSize();

            if (x < 0 || x >= grid_size_ || y < 0 || y >= grid_size_) continue;

            if (size <= grid_res_) {
                grid_msg.data[y * grid_size_ + x] = 100; // Occupied
            } else {
                int scale = std::ceil(size / grid_res_);
                int half_scale = scale / 2;
                for (int dx = -half_scale; dx <= half_scale; ++dx) {
                    for (int dy = -half_scale; dy <= half_scale; ++dy) {
                        int ix = x + dx;
                        int iy = y + dy;
                        if (ix >= 0 && ix < grid_size_ && iy >= 0 && iy < grid_size_) {
                            grid_msg.data[iy * grid_size_ + ix] = 100;
                        }
                    }
                }
            }
        }


        gridmap_pub_.publish(grid_msg);
    }


private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> terrain_pointcloud_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    ros::Publisher gridmap_pub_;
    octomap::OcTree obstacle_tree_; // Stores obstacles
    octomap::OcTree free_tree_;     // Stores traversable space
    octomap::OcTree scan_tree_;
    // Parameters
    double grid_size_m_;  // Grid size in meters
    int grid_size_;       // Grid size in cells
    double grid_res_;     // Resolution
    double z_range_;      // Z-axis range
    double sensor_range_;  // Sensor range
    bool only_ground_free_;
    // Hit and miss probabilities
    float hit_probability_;
    float miss_probability_;
    float hit_max_;
    float miss_min_;
    // Robot position
    double robot_x_;
    double robot_y_;
    double robot_z_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    bool remove_dyn_obs_;
    float obstacle_height_thr_;
    bool use_bounding_box_;
    double bounding_box_x_min_;
    double bounding_box_y_min_;
    double bounding_box_x_max_;
    double bounding_box_y_max_;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_to_gridmap");
    OctoMapToGridMap node;
    ros::spin();
    return 0;
}
