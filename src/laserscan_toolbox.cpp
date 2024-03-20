#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
//#include "tf2_sensor_msgs/tf2_sensor_msgs.h" //was working on ros2 foxy
#include "rosgraph_msgs/msg/clock.hpp"  // Add the clock message
#include <cmath>
#include <string>
#include <fstream>

#include <Eigen/Dense>

int angle_to_index(double alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 angles.
    // Normalize the angle to the range [0, 2*M_PI)
    alpha = std::fmod(alpha, 2 * M_PI);
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    // Calculate the index
    return static_cast<int>(round((alpha * resolution) / (2 * M_PI)));
}

double index_to_angle(int ind, int resolution){
    return (ind*(2*M_PI))/resolution;
}

void get_pos(double &x, double &y,double alpha,double val,double x_off,double y_off){
    x = val*cos(alpha)+x_off;
    y = val*sin(alpha)+y_off;
}

bool save_dataset = false;//if true, the code will generate a dataset with the first values of the first lidar in home/scan_merger_dataset.txt
sensor_msgs::msg::LaserScan transform_data(sensor_msgs::msg::LaserScan::SharedPtr msg, double off_vect_x,double off_vect_y, double off_tetha){
    /*
    msg: current message to transform
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    return transformed_scan: message in which datas will be stored
    */
    //RCLCPP_INFO(this->get_logger(), "Transform_data: params : offx='%f'; offy='%f'; off_tetha='%f'",off_vect_x,off_vect_y,off_tetha);
    //RCLCPP_INFO(this->get_logger(), "transform 1");
    sensor_msgs::msg::LaserScan transformed_scan;
    transformed_scan.ranges.clear();
    int resolution = msg->ranges.size();

    // Transform the LaserScan message
    for (int i = 0; i < resolution; ++i) { //initiate
        transformed_scan.ranges.push_back(INFINITY);
    }
    //RCLCPP_INFO(this->get_logger(), "transform 2");
    //loop variables
    double init_x = 0;
    double init_y = 0;
    double init_val = 0;
    double init_angle = 0;
    double new_x = 0;
    double new_y = 0;
    double new_val = 0;
    double new_angle = 0;
    int new_index = 0;

    for (int ind = 0; ind < resolution; ++ind) { //transform
        init_angle = index_to_angle(ind,resolution);
        init_val = msg->ranges[ind];
        //we want angles from 0 to 2pi
        if (off_tetha<0){
            off_tetha=2*M_PI+off_tetha;
        }
        if (off_tetha == 2*M_PI){
            off_tetha=0;
        }
        if (!isinf(init_val)){ //if it is infinity, the output will be infinity whatever the new frame
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            //what would detect the lidar id not rotated
            new_x = init_x*cos(off_tetha)-init_y*sin(off_tetha);
            new_y = init_x*sin(off_tetha)+init_y*cos(off_tetha);
            //rotation correction due to frame orientation difference
            double new_x2 = -off_vect_x + new_x;
            double new_y2 = -off_vect_y + new_y;
            new_angle = atan2(new_y2,new_x2);
            //we want angles from 0 to 2pi
            if (new_angle<0){
                new_angle=2*M_PI+new_angle;
            }
            if (new_angle == 2*M_PI){
                new_angle=0;
            }
            //RCLCPP_INFO(this->get_logger(), "Transform_data: newangle='%f'",new_angle);

            //new norm
            new_val = sqrt(pow(new_x2,2)+pow(new_y2,2));

            new_index = angle_to_index(new_angle,resolution);
            transformed_scan.ranges[new_index] = std::min(static_cast<double>(transformed_scan.ranges[new_index]),new_val); //in case some area are no more accessible from the new origin
            //RCLCPP_INFO(this->get_logger(), "Transform_data: initindex='%d'; initangle='%f'; initval='%f'; initx='%f'; inity='%f'; newx2='%f'; newy2='%f'; newval='%f'; newangle='%f'; newindex='%f'",ind,init_angle,init_val,init_x,init_y,new_x2,new_y2,new_val,new_angle,new_index);
        }

    }
    //RCLCPP_INFO(this->get_logger(), "transform 3");

    //Possibility to generate dataset with first values of first lidar
    if(save_dataset){
        std::ofstream outputFile("scan_merger_dataset.txt");
        if (outputFile.is_open()){
            for (int ind = 0; ind < resolution; ++ind){
                outputFile << msg->ranges[ind] << ",";
            }
            outputFile.close();
        }
        save_dataset = false; 
    } 

    return transformed_scan;
}

geometry_msgs::msg::Vector3 adapt_angle(geometry_msgs::msg::Vector3 vect){
    //avoid flip with euleur angles
    geometry_msgs::msg::Vector3 new_vect;
    if(abs(abs(vect.x)-M_PI)<0.01 && abs(abs(vect.y)-M_PI)<0.01){
        new_vect.x = 0.0;
        new_vect.y = 0.0;
        new_vect.z = M_PI+vect.z; 
    }
    else{
        new_vect.x = vect.x;
        new_vect.y = vect.y;
        new_vect.z = vect.z;
    }
    return new_vect;
}

geometry_msgs::msg::Vector3 quaternion_to_euler(geometry_msgs::msg::Quaternion quat){

    // Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
    Eigen::Quaterniond eigen_quaternion(
        quat.w,
        quat.x,
        quat.y,
        quat.z
    );

    // Convert quaternion to Euler angles
    Eigen::Vector3d euler_angles = eigen_quaternion.toRotationMatrix().eulerAngles(0, 1, 2); 

    geometry_msgs::msg::Vector3 rot_vect;

    rot_vect.x = euler_angles(0);
    rot_vect.y = euler_angles(1);
    rot_vect.z = euler_angles(2);

    //RCLCPP_INFO(this->get_logger(), "ROT Quat: x='%.2f', y='%.2f', z='%.2f', w='%.2f'", quat.x,quat.y,quat.z,quat.w);
    //RCLCPP_INFO(this->get_logger(), "ROT Euler: x='%.2f', y='%.2f', z='%.2f'", rot_vect.x,rot_vect.y,rot_vect.z);
    return adapt_angle(rot_vect);

}


bool consider_val(int current_ind, int start_ind, int end_ind){
    // return true if current_ind is between start_ind and end_ind according to a circle reference.
    if(start_ind>end_ind){ //if interval pass throught the origin of the circle, we test considering the split into 2 interval
        return (current_ind>=start_ind || current_ind<=end_ind);
    }
    else{ // if values are equal, or classical ,we test as classic interval
        return (current_ind>=start_ind && current_ind<=end_ind);
    }
}

sensor_msgs::msg::LaserScan filter_data(sensor_msgs::msg::LaserScan::SharedPtr msg,double start_angle, double end_angle, double angle_origin_offset){

    sensor_msgs::msg::LaserScan filtered_scan;
    filtered_scan.ranges.clear();
    int resolution = msg->ranges.size();

    //compute index to consider according to min/max angles wanted
    int start_index = angle_to_index(start_angle+angle_origin_offset, resolution);
    int end_index = angle_to_index(end_angle+angle_origin_offset, resolution);

    // LaserScan message filling
    for (int i = 0; i < resolution; ++i) { 
        if(consider_val(i, start_index, end_index)){ //if the value is autorized, we add it
            filtered_scan.ranges.push_back(msg->ranges[i]);
        }
        else{
            filtered_scan.ranges.push_back(INFINITY);
        }
    }

    //we shift all the values to bring back the origin angle to the x axis (not needed)
    /*int index_shift = angle_to_index(angle_origin_offset,resolution); //index which is the first angle for the lidar
    sensor_msgs::msg::LaserScan shifted_scan;
    shifted_scan.ranges.clear();
    for (int i = 0; i < resolution; ++i) { 
        if(i+index_shift < resolution){ 
            shifted_scan.ranges.push_back(msg->ranges[i+index_shift]);
        }
        else{
            shifted_scan.ranges.push_back(msg->ranges[i+index_shift-resolution]);
        }
    }*/

    return filtered_scan;
}

sensor_msgs::msg::LaserScan::SharedPtr convert_raw_data(sensor_msgs::msg::LaserScan::SharedPtr laser_raw,std::string new_frame,double start_angle, double end_angle, double angle_origin_offset, geometry_msgs::msg::Vector3 vector_newframe, geometry_msgs::msg::Vector3 rotate_newframe){
    if (laser_raw != nullptr){
        //RCLCPP_INFO(this->get_logger(), "Lid1_run");
        //transform data and configure new message
        sensor_msgs::msg::LaserScan tempo = filter_data(laser_raw,start_angle, end_angle, angle_origin_offset);
        sensor_msgs::msg::LaserScan tempo2 = transform_data(std::make_shared<sensor_msgs::msg::LaserScan>(tempo),vector_newframe.x,vector_newframe.y,rotate_newframe.z);
        sensor_msgs::msg::LaserScan::SharedPtr scan_data_ = std::make_shared<sensor_msgs::msg::LaserScan>(tempo2); 
        //RCLCPP_INFO(this->get_logger(), "Lid1_transformed");
        scan_data_->header=laser_raw->header;
        scan_data_->header.frame_id=new_frame;
        scan_data_->angle_min=laser_raw->angle_min; 
        scan_data_->angle_max=laser_raw->angle_max; 
        scan_data_->angle_increment=laser_raw->angle_increment; 
        scan_data_->time_increment=laser_raw->time_increment; 
        scan_data_->scan_time=laser_raw->scan_time; 
        scan_data_->range_min=laser_raw->range_min; 
        scan_data_->range_max=laser_raw->range_max; 
        scan_data_->intensities=laser_raw->intensities; 
        //RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", msg->ranges[0],msg->ranges[100]);
        //RCLCPP_INFO(this->get_logger(), "Lid1_configured");
        //RCLCPP_INFO(this->get_logger(), "Lid1_end");
        return scan_data_;
    }
    else{
        return nullptr;
    }
}

class LaserScanToolboxNode : public rclcpp::Node {
public:
    LaserScanToolboxNode()
    : Node("laserscan_toolbox_node") {

        initialize_params();
        refresh_params();
        debug_params();
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        //QOS:
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        /*
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        //custom_qos.history=RMW_QOS_POLICY_HISTORY_KEEP_ALL;
        custom_qos.depth=10;
        //custom_qos.reliability=RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        custom_qos.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        custom_qos.durability=RMW_QOS_POLICY_DURABILITY_VOLATILE;
        //custom_qos.durability=RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
        */

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic_lid1, sensor_qos, std::bind(&LaserScanToolboxNode::scan1Callback, this, std::placeholders::_1));
        subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic_lid2, sensor_qos, std::bind(&LaserScanToolboxNode::scan2Callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_out, default_qos);
        // Subscribe to the clock topic
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos, std::bind(&LaserScanToolboxNode::ClockCallback, this, std::placeholders::_1));

        //timer for publisher (Lidars are around 7Hz)
        timer_ = create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&LaserScanToolboxNode::fuseAndPublish, this));

        RCLCPP_INFO(this->get_logger(), "Merger Started: Lidar Scan messages need to be same length and same angles correspondance on 360 elongation.");
        if(use_sim_time){ //if use_sim_time = true
            RCLCPP_INFO(this->get_logger(), "Use Simu time");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Use REAL time");
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for topics '%s' and '%s' ...",topic_lid1.c_str(),topic_lid2.c_str());
        clock = this->get_clock();
    }

private:
    void ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // Update the current timestamp when the clock message is received
        current_timestamp_ = msg->clock;
    }

    void scan1Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Callback1");
        //get frame
        if (frame_lid1 != msg->header.frame_id and !frames_got){
            frame_lid1 = msg->header.frame_id;
        }
        if (frames_got){
            raw_scan1_data_ = msg;
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Trying to get frames");
            try {
                //get translation vector new_frame=>lidar1
                geometry_msgs::msg::TransformStamped transform1 =tf_buffer_->lookupTransform(new_frame, frame_lid1, tf2::TimePointZero);
                vector_newframe_lid1 = transform1.transform.translation;  //translation vector from new_frame to fram_lid1
                rotate_newframe_lid1 = quaternion_to_euler(transform1.transform.rotation);
                //get translation vector new_frame=>lidar2
                geometry_msgs::msg::TransformStamped transform2 =tf_buffer_->lookupTransform(new_frame, frame_lid2, tf2::TimePointZero);
                vector_newframe_lid2 = transform2.transform.translation; //translation vector from new_frame to fram_lid2
                rotate_newframe_lid2 = quaternion_to_euler(transform2.transform.rotation);
                frames_got=true;
                RCLCPP_INFO(this->get_logger(), "Transform New_frame => L1: '%f' '%f'",vector_newframe_lid1.x,vector_newframe_lid1.y);
                RCLCPP_INFO(this->get_logger(), "ROT Euler1: x='%.2f', y='%.2f', z='%.2f'", rotate_newframe_lid1.x,rotate_newframe_lid1.y,rotate_newframe_lid1.z);
                RCLCPP_INFO(this->get_logger(), "Transform New_frame => L2: '%f' '%f'",vector_newframe_lid2.x,vector_newframe_lid2.y);
                RCLCPP_INFO(this->get_logger(), "ROT Euler2: x='%.2f', y='%.2f', z='%.2f'", rotate_newframe_lid2.x,rotate_newframe_lid2.y,rotate_newframe_lid2.z);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "%s (Ignore this error if node is launched correctly after)", e.what());
                frames_got=false;
                active_notif=true;
            }
        }

    }

    void scan2Callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Callback2");
        //get frame
        if (frame_lid2 != msg->header.frame_id and !frames_got){
            frame_lid2 = msg->header.frame_id;
        }
        if (frames_got){
            raw_scan2_data_ = msg;
        }
    }

    void fuseAndPublish() {
        scan1_data_ = convert_raw_data(raw_scan1_data_,new_frame,lidar1_start_angle, lidar1_end_angle,lidar1_angle_origin_offset, vector_newframe_lid1, rotate_newframe_lid1);
        scan2_data_ = convert_raw_data(raw_scan2_data_,new_frame,lidar2_start_angle, lidar2_end_angle,lidar2_angle_origin_offset, vector_newframe_lid2, rotate_newframe_lid2);

        if (scan1_data_ != nullptr && scan2_data_ != nullptr) {
            // Implemented fusion logic here 
            //RCLCPP_INFO(this->get_logger(), "fuse");
            fused_data = fuseScans(scan1_data_, scan2_data_);
            if(use_sim_time){ //if use_sim_time = true
                fused_data.header.stamp = current_timestamp_;
                //RCLCPP_INFO(this->get_logger(), "Use Simu time");
            }
            else{
                fused_data.header.stamp = clock->now();
                //RCLCPP_INFO(this->get_logger(), "Use REAL time");
            }
            
            //RCLCPP_INFO(this->get_logger(), "fused");
            //RCLCPP_INFO(this->get_logger(), "I heard merged: '%f' '%f'", fused_data.ranges[0],fused_data.ranges[100]);
            
            // Publish the fused laser scan
            publisher_->publish(fused_data);
            if(active_notif){
                RCLCPP_INFO(this->get_logger(), "Merging '%s' and '%s'",topic_lid1.c_str(),topic_lid2.c_str());
                RCLCPP_INFO(this->get_logger(), "Target frame: '%s'",new_frame.c_str());
                RCLCPP_INFO(this->get_logger(), "Merged Scan published on topic: '%s'",topic_out.c_str());
                active_notif=false;
            }
        }
        else{
            //RCLCPP_WARN(this->get_logger(), "Lidar merger need the topics: '%s' '%s' and the frames: '%s' '%s' '%s' to work.", topic_lid1.c_str(),topic_lid2.c_str(),frame_lid1.c_str(), frame_lid2.c_str(), new_frame.c_str());
            active_notif=true;
        }
    }

    sensor_msgs::msg::LaserScan fuseScans(const sensor_msgs::msg::LaserScan::SharedPtr scan1,const sensor_msgs::msg::LaserScan::SharedPtr scan2) {
        int resolution = scan1->ranges.size(); //assuming length are the same, we are using double resolution for more accurate result
        sensor_msgs::msg::LaserScan fused_data = *scan1; // Assuming headers are the same, the scan1 have already been transformed in the new_frame
        
        //init ranges and add inf values according to the resolution
        fused_data.ranges = {};
        for (int i = 0; i < resolution; ++i){
            fused_data.ranges.push_back(INFINITY);
        }

        //fuse datas
        for (int i = 0; i < resolution; ++i) { 
            //fused_data.ranges[i] = scan2->ranges[i]; 
            fused_data.ranges[i] = std::min(scan1->ranges[i],scan2->ranges[i]); //we take the closed signal
        }

        return fused_data;
    }

    void initialize_params(){

        //only if we want to publish the parameters to other nodes
        this->declare_parameter("lidar1_start_angle",0.0); 
        this->declare_parameter("lidar1_end_angle",2*M_PI); 
        this->declare_parameter("lidar1_angle_origin_offset",0.0);
        this->declare_parameter("lidar2_start_angle",0.0); 
        this->declare_parameter("lidar2_end_angle",2*M_PI);
        this->declare_parameter("lidar2_angle_origin_offset",0.0); 
        this->declare_parameter("topic_lid1","lidar1/scan");
        this->declare_parameter("topic_lid2","lidar2/scan"); 
        this->declare_parameter("topic_out","scan"); 
        this->declare_parameter("new_frame","base_link");
        this->declare_parameter("rate",20.0);
        //this->declare_parameter("use_sim_time",false); //usually declared in aunch file, default value given in variable definition directly

    }

    void refresh_params(){
        get_parameter("lidar1_start_angle",lidar1_start_angle);                  
        get_parameter("lidar1_end_angle",lidar1_end_angle);
        get_parameter("lidar1_angle_origin_offset",lidar1_angle_origin_offset);
        get_parameter("lidar2_start_angle",lidar2_start_angle);
        get_parameter("lidar2_end_angle",lidar2_end_angle);
        get_parameter("lidar2_angle_origin_offset",lidar2_angle_origin_offset);
        get_parameter("topic_lid1",topic_lid1);
        get_parameter("topic_lid2",topic_lid2);
        get_parameter("topic_out",topic_out);
        get_parameter("new_frame",new_frame);
        get_parameter("rate",rate);
        get_parameter("use_sim_time",use_sim_time);
    }

    void debug_params(){
        RCLCPP_INFO(this->get_logger(), 
        "\nPARAMETERS:\n"
        "lidar1_start_angle: %f \n"
        "lidar1_end_angle: %f \n"
        "lidar1_angle_origin_offset: %f\n"
        "lidar2_start_angle: %f\n"
        "lidar2_end_angle: %f\n"
        "lidar2_angle_origin_offset: %f\n"
        "topic_lid1: %s\n"
        "topic_lid2: %s\n"
        "topic_out: %s\n"
        "new_frame: %s\n"
        "rate: %f\n"
        "use_sim_time: %d\n",
        lidar1_start_angle,
        lidar1_end_angle,
        lidar1_angle_origin_offset,
        lidar2_start_angle,
        lidar2_end_angle,
        lidar2_angle_origin_offset,
        topic_lid1.c_str(),
        topic_lid2.c_str(),
        topic_out.c_str(),
        new_frame.c_str(),
        rate,
        use_sim_time);
    }

    sensor_msgs::msg::LaserScan::SharedPtr raw_scan1_data_ = nullptr;
    sensor_msgs::msg::LaserScan::SharedPtr raw_scan2_data_ = nullptr;
    sensor_msgs::msg::LaserScan::SharedPtr scan1_data_ = nullptr;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_data_ = nullptr;
    sensor_msgs::msg::LaserScan fused_data;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    //Lidar configurations
    double lidar1_start_angle; //start angle to consider for lidar1 (0 to 2pi circle)
    double lidar1_end_angle; //endangle to consider for lidar1 (0 to 2pi circle)
    double lidar1_angle_origin_offset; //offset on rotation axis for angle 0 from x axis
    double lidar2_start_angle;
    double lidar2_end_angle;
    double lidar2_angle_origin_offset;
    std::string topic_lid1;
    std::string topic_lid2;
    std::string topic_out;
    std::string frame_lid1;
    std::string frame_lid2;
    std::string new_frame;
    double rate;
    bool use_sim_time = false;
    geometry_msgs::msg::Vector3 vector_newframe_lid1;
    geometry_msgs::msg::Vector3 vector_newframe_lid2;
    geometry_msgs::msg::Vector3 rotate_newframe_lid1;
    geometry_msgs::msg::Vector3 rotate_newframe_lid2;
    bool frames_got = false;
    bool active_notif = true;

    //for transform considerations
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //for clock in simulations
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
    builtin_interfaces::msg::Time current_timestamp_;

    rclcpp::Clock::SharedPtr clock; //used if not a simulation

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanToolboxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
