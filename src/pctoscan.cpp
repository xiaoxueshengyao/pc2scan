    #include <ros/ros.h>
    #include <sensor_msgs/PointCloud2.h>
    #include <sensor_msgs/LaserScan.h>
    #include <sensor_msgs/point_cloud2_iterator.h>
    #include <sensor_msgs/PointCloud.h>
    #include <sensor_msgs/point_cloud_conversion.h>


    class PointCloudtoScan
    {
    public:
    PointCloudtoScan();
    ~PointCloudtoScan(){}
    
    

    private:
    void Init();
    void ReadParameters();
    void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs);
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs);
    
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Publisher pub2_;

    std::string target_frame_;
    double tolerance_;
    double min_height_,max_height_,angle_min_, angle_max_;
    double angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
    double inf_epsilon_;


    };

    PointCloudtoScan::PointCloudtoScan()
    {
        Init();
    }

    void PointCloudtoScan::Init()
    {
        ROS_INFO("Transform init.");
        ReadParameters();
        sub_ = nh_.subscribe("/rslidar_points",1,&PointCloudtoScan::CloudCallback,this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan",10);
        // pub2_ = nh_.advertise<sensor_msgs::PointCloud>("/pc",10);

    }

    void PointCloudtoScan::ReadParameters()
    {
        ROS_INFO("Read parameters.");
        nh_.param<double>("/pointcloud_to_scan/min_height", min_height_, 0.3);
        nh_.param<double>("/pointcloud_to_scan/max_height", max_height_, 1.0);

        nh_.param<double>("/pointcloud_to_scan/angle_min", angle_min_, - M_PI);
        nh_.param<double>("/pointcloud_to_scan/angle_max", angle_max_, M_PI);
        nh_.param<double>("/pointcloud_to_scan/angle_increment", angle_increment_, M_PI/180.0);
        nh_.param<double>("/pointcloud_to_scan/scan_time", scan_time_, 1.0 / 30.0);
        nh_.param<double>("/pointcloud_to_scan/range_min", range_min_, 0.0);
        nh_.param<double>("/pointcloud_to_scan/range_max", range_max_, 100.0);
        nh_.param<double>("/pointcloud_to_scan/inf_epsilon", inf_epsilon_, 1.0);
        nh_.param<bool>("/pointcloud_to_scan/use_inf", use_inf_, false);

        std::cout<<"angle_min: "<<angle_min_<<std::endl;
        std::cout<<"angle_max: "<<angle_max_<<std::endl;
        std::cout<<"min_height: "<<min_height_<<std::endl;
        std::cout<<"max_height: "<<max_height_<<std::endl;
    }

    void PointCloudtoScan::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs)
    {
        sensor_msgs::LaserScan output_scan;
        output_scan.header = cloud_msgs->header;
        output_scan.angle_min = angle_min_;
        output_scan.angle_max = angle_max_;
        output_scan.angle_increment = angle_increment_;
        output_scan.time_increment = 0.0;
        output_scan.scan_time = scan_time_;
        output_scan.range_min = range_min_;
        output_scan.range_max = range_max_;

        uint32_t range_size = std::ceil((angle_max_ - angle_min_)/angle_increment_);
        // std::cout<<"range size : "<<range_size<<std::endl;
        if(use_inf_)
        {
            output_scan.ranges.assign(range_size,std::numeric_limits<double>::infinity());
        }
        else
        {
            output_scan.ranges.assign(range_size,range_max_ + inf_epsilon_);
        }

        sensor_msgs::PointCloud cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msgs,cloud);
        // pub2_.publish(cloud);

        for(int i = 0; i<cloud.points.size(); i++)
        {
            if(std::isnan(cloud.points[i].x) || std::isnan(cloud.points[i].y) || std::isnan(cloud.points[i].z))
            {
                continue;
            }
            if(cloud.points[i].z > max_height_ || cloud.points[i].z < min_height_)
            {
                // std::cout<<"reject by height"<<std::endl;
                continue;
            }
            double range = hypot(cloud.points[i].x,cloud.points[i].y);
            if(range < range_min_)
            {
                // std::cout<<"reject by range"<<std::endl;
                continue;
            }
            double angle = atan2(cloud.points[i].y,cloud.points[i].x);
            // std::cout<<"angle : "<<angle<<std::endl;
            // std::cout<<"y :"<<cloud.points[i].y<<"  x: "<<cloud.points[i].x<<std::endl;
            if(angle < angle_min_ || angle > angle_max_)
            {
                // std::cout<<"reject by angle"<<std::endl;
                continue;
            }
            int idx = (angle - angle_min_) / angle_increment_;
            // std::cout<<"index : "<<idx<<std::endl;
            if(range < output_scan.ranges[idx])
            {
                // std::cout<<"replaced by range"<<std::endl;
                output_scan.ranges[idx] = range;
            }
        }

        

        pub_.publish(output_scan);  
        // std::cout<<"pub once"<<std::endl;  
    }


    void PointCloudtoScan::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msgs)
    {

    }





    int main(int argc, char** argv)
    {
        ros::init(argc,argv, "pointcloudtoscan");
        ros::NodeHandle nh;
        ROS_INFO("Start transformation");
        PointCloudtoScan pc2scan;
        ros::spin();
        
        
        return 0;
    }
