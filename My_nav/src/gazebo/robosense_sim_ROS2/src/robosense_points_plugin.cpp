#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>  // Store the latest laser scans into laserMsg
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "robosense_sim/csv_reader.hpp"
#include "robosense_sim/robosense_points_plugin.h"
#include "robosense_sim/robosense_ode_multiray_shape.h"

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(RobosensePointsPlugin)

    RobosensePointsPlugin::RobosensePointsPlugin() {}

    RobosensePointsPlugin::~RobosensePointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; //转化成标准的右手系角度
            } else {
            RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
        }
        }
    }

    void RobosensePointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("RobosensePointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_INFO(rclcpp::get_logger("RobosensePointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");


        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("RobosensePointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        child_name = raySensor->Name();
        parent_name = raySensor->ParentName();
        size_t delimiter_pos = parent_name.find("::");
        parent_name = parent_name.substr(delimiter_pos + 2);

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        // PointCloud2 publisher
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            curr_scan_topic, rclcpp::SensorDataQoS());

        scanPub = node->Advertise<msgs::LaserScanStamped>(curr_scan_topic+"laserscan", 50);

        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("RobosensePointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        //SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::RobosenseOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("RobosensePointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("RobosensePointsPlugin"), "downsample: %ld", downSample);
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
            int index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }



    void RobosensePointsPlugin::OnNewLaserScans() {
        if (!rayShape) {
            return; // 检查是否已经初始化了 rayShape
        }

        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        // Publish the same PointCloud2 field layout as rslidar_sdk XYZIRT:
        // x, y, z, intensity, ring and absolute timestamp.
        sensor_msgs::msg::PointCloud2 cloud2;
        cloud2.header.stamp = node_->get_clock()->now();
        cloud2.header.frame_id = raySensor->Name();

        sensor_msgs::PointCloud2Modifier modifier(cloud2);
        modifier.setPointCloud2Fields(
            6,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring", 1, sensor_msgs::msg::PointField::UINT16,
            "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64);
        modifier.resize(points_pair.size());

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud2, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud2, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud2, "z");
        sensor_msgs::PointCloud2Iterator<float> out_intensity(cloud2, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> out_ring(cloud2, "ring");
        sensor_msgs::PointCloud2Iterator<double> out_timestamp(cloud2, "timestamp");
        const double scan_timestamp = rclcpp::Time(cloud2.header.stamp).seconds();
        const double point_period = points_pair.empty() ? 0.0 : 0.1 / points_pair.size();
        std::size_t point_index = 0;

        // 遍历射线扫描点对
        for (const auto &pair : points_pair) {
            auto range = rayShape->GetRange(pair.first);
            auto intensity = rayShape->GetRetro(pair.first);

            // 处理超出范围的数据
            if (range <= RangeMin() || range >= RangeMax()) {
                range = 0;
            }

            // 计算点云数据
            auto rotate_info = pair.second;
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            auto point = range * axis;

            *out_x = point.X();
            *out_y = point.Y();
            *out_z = point.Z();
            *out_intensity = intensity;
            *out_ring = static_cast<uint16_t>(pair.first % 96);
            *out_timestamp = scan_timestamp + point_index * point_period;

            ++out_x;
            ++out_y;
            ++out_z;
            ++out_intensity;
            ++out_ring;
            ++out_timestamp;
            ++point_index;
        }

        if (scanPub && scanPub->HasConnections()) {
            scanPub->Publish(laserMsg);
        }

        cloud2_pub->publish(cloud2);
    }


    void RobosensePointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::RobosenseOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose();
        int64_t end_index = currStartIndex + samplesStep;
        long unsigned int ray_index = 0;
        auto ray_size = rays.size();
        points_pair.reserve(rays.size());
        for (int k = currStartIndex; k < end_index; k += downSample)
        {
            auto index = k % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            if (ray_index < ray_size)
            {
                rays[ray_index]->SetPoints(start_point, end_point);
                points_pair.emplace_back(ray_index, rotate_info);
            }
            ray_index++;
        }
        currStartIndex += samplesStep;
    }

    void RobosensePointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle RobosensePointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle RobosensePointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double RobosensePointsPlugin::GetRangeMin() const { return RangeMin(); }

    double RobosensePointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double RobosensePointsPlugin::GetRangeMax() const { return RangeMax(); }

    double RobosensePointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double RobosensePointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double RobosensePointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double RobosensePointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double RobosensePointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int RobosensePointsPlugin::GetRayCount() const { return RayCount(); }

    int RobosensePointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int RobosensePointsPlugin::GetRangeCount() const { return RangeCount(); }

    int RobosensePointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int RobosensePointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int RobosensePointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int RobosensePointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int RobosensePointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle RobosensePointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle RobosensePointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double RobosensePointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double RobosensePointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }


}
