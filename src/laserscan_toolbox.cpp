#include "rclcpp/rclcpp.hpp"
#include <string>
#include <map>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <cmath>
#include "rosgraph_msgs/msg/clock.hpp"  // Add the clock message
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

std::string vector3ToString(geometry_msgs::msg::Vector3& vector);
geometry_msgs::msg::Vector3 stringToVector3(std::string& str);
geometry_msgs::msg::Vector3 quaternion_to_euler(geometry_msgs::msg::Quaternion quat);
geometry_msgs::msg::Vector3 adapt_angle(geometry_msgs::msg::Vector3 vect);
double TimeToDouble(builtin_interfaces::msg::Time& stamp);
sensor_msgs::msg::LaserScan::SharedPtr convert_raw_data(sensor_msgs::msg::LaserScan::SharedPtr laser_raw,std::string new_frame,double start_angle, double end_angle, double angle_origin_offset, geometry_msgs::msg::Vector3 vector_newframe, geometry_msgs::msg::Vector3 rotate_newframe, std::stringstream &debug_ss);
sensor_msgs::msg::LaserScan filter_data(sensor_msgs::msg::LaserScan::SharedPtr msg,double start_angle, double end_angle, double angle_origin_offset);
int angle_to_index(double alpha, int resolution);
double index_to_angle(int ind, int resolution);
void get_pos(double &x, double &y,double alpha,double val,double x_off,double y_off);
sensor_msgs::msg::LaserScan transform_data(sensor_msgs::msg::LaserScan::SharedPtr msg, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss);


class LaserScanToolboxNode : public rclcpp::Node {
public:
    LaserScanToolboxNode() : Node("laserscan_toolbox_node") {
        /*
        Parameters initialisation
        */
        initialize_common_params();
        refresh_common_params();
        initialize_sources_params();
        refresh_sources_params();
        if(debug){
            debug_params();
        }

        /*
        Initialisation
        */

        //QOS:
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        //Tfs listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        //Clock
        clock = this->get_clock();
        //sources variables
        init_sources_variables();

        /*
        Subscriptions
        */

        //Clock
        if(use_sim_time){
            clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", sensor_qos, std::bind(&LaserScanToolboxNode::ClockCallback, this, std::placeholders::_1));
        }
        //Sources
        for (const auto& pair1 : sources_config) {
            std::string source_name = pair1.first;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(sources_config[source_name]["topic"], sensor_qos, [this,source_name](const sensor_msgs::msg::LaserScan::SharedPtr msg){ScanCallback(msg,source_name);});
            // Store the subscription in the map if needed for futur handling
            subscriptions_[pair1.first] = subscription_;
        }

        /*
        Publisher
        */

        //timer for publisher (RPLidars are around 7Hz for example)
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_out, default_qos);
        timer_ = create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&LaserScanToolboxNode::fuseAndPublish, this));
    }

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg,std::string source_name) {
        std::stringstream debug_ss;
        bool ready = false;
        debug_ss << "\n\n" << source_name << ": Scan received" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
        sources_var[source_name]["last_timestamp"] = std::to_string(TimeToDouble(msg->header.stamp));
        //we get frame and transformation of the sensor before to use the data
        if (sources_var[source_name]["frame"] == "" || sources_var[source_name]["frame_trans_vector"] == ""){
            sources_var[source_name]["frame"] = msg->header.frame_id;
            try {
                //get translation vector new_frame=>sensor
                geometry_msgs::msg::TransformStamped transf =tf_buffer_->lookupTransform(new_frame, sources_var[source_name]["frame"], tf2::TimePointZero);
                geometry_msgs::msg::Vector3 vector_newframe_sensor = transf.transform.translation;  //translation vector from new_frame to frame_sensor
                geometry_msgs::msg::Vector3 rotate_newframe_sensor = quaternion_to_euler(transf.transform.rotation);
                //sav datas
                sources_var[source_name]["frame_trans_vector"] = vector3ToString(vector_newframe_sensor);
                sources_var[source_name]["frame_rot_vector"] = vector3ToString(rotate_newframe_sensor);
                debug_ss << source_name << ": Got transformation"
                         <<"\n  Translation: "<< sources_var[source_name]["frame_trans_vector"]
                         <<"\n  Euler rotation: "<< sources_var[source_name]["frame_rot_vector"] << std::endl;
                ready = true;
            }
            catch (const std::exception& e) {
                debug_ss  << source_name << ": Couldn't get transformation " << new_frame << " --> " << sources_var[source_name]["frame"] << std::endl;
            }
        }
        else{
            ready = true;
        }

        if(ready){
            raw_scans[source_name] = msg;
            debug_ss << source_name << ": Raw scan updated" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            debug_ss << "  Raw scan timestamp: " << sources_var[source_name]["last_timestamp"] << " s)" << std::endl;
        }

        //debug
        if(debug){
            std::string debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
        }
    }

    void ClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {
        // Update the current timestamp when the clock message is received
        simu_timestamp = msg->clock;
    }

    void fuseAndPublish() {
        std::stringstream debug_ss;
        if(raw_scans.size() > 0){
            debug_ss << "\n\nStarting received scan transformations: Other sources may arrive later" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
        
            //Scan transformations and processing according to configurations
            for (const auto& pair1 : raw_scans) {
                std::string source_name = pair1.first;
                transformed_scans[source_name] = convert_raw_data(raw_scans[source_name],new_frame,std::stod(sources_config[source_name]["start_angle"]), std::stod(sources_config[source_name]["end_angle"]),std::stod(sources_config[source_name]["scan_angle_offset"]), stringToVector3(sources_var[source_name]["frame_trans_vector"]), stringToVector3(sources_var[source_name]["frame_rot_vector"]), debug_ss);
                debug_ss << source_name << ": Scan transformed, ready for fusion..." << "(last data stamp: " << sources_var[source_name]["last_timestamp"] << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            }

            debug_ss <<"    |\n    |\n    |\n    V" << "\nStarting FUSION" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            //Fusion
            fused_scan = new_clean_scan();
            fused_scan.header.frame_id=new_frame;
            int resolution = fused_scan.ranges.size();
            for (const auto& pair1 : transformed_scans) {
                std::string source_name = pair1.first;
                fuseScans(resolution, fused_scan, *transformed_scans[source_name]);
                debug_ss << "fused_scan <-- " << source_name << " (last data stamp: " << sources_var[source_name]["last_timestamp"] << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            }

            //put clock
            if(use_sim_time){ //if use_sim_time = true
                fused_scan.header.stamp = simu_timestamp;
            }
            else{
                fused_scan.header.stamp = clock->now();
            }

            // Publish the fused laser scan
            publisher_->publish(fused_scan);

            //debug data
            int non_zero_vals = 0;
            int total_vals = fused_scan.ranges.size();
            if(debug){
                for(int i=0; i < total_vals; i++){
                    if(fused_scan.ranges[i] < INFINITY){
                        non_zero_vals ++;
                    }
                }
            }
            debug_ss << "    |\n    |\n    |\n    V"
                     << "\nfused_scan published" << " (node time: " << (this->now()).nanoseconds() << "ns)" 
                     << "\n  Frame: " <<  fused_scan.header.frame_id
                     << "\n  Timestamp: " <<  std::to_string(TimeToDouble(fused_scan.header.stamp))
                     << "\n  Number of rays: " << total_vals
                     << "\n  Number of hit: " << non_zero_vals
                     << std::endl;

            if(debug && show_ranges){
                debug_ss << "Ranges: ";
                for(int i=0; i < total_vals; i++){
                    debug_ss << fused_scan.ranges[i] << " ";
                }
            }

            //debug
            if(debug){
                std::string debug_msg = debug_ss.str();
                write_debug(debug_file_path, debug_msg);
            }
        }
    }

    void fuseScans(int resolution, sensor_msgs::msg::LaserScan &scan1,sensor_msgs::msg::LaserScan &scan2) {
        //merge scan2 in scan1
        //fuse datas
        for (int i = 0; i < resolution; ++i) { 
            //we take the closest signal
            if(scan2.ranges[i]<scan1.ranges[i]){
                scan1.ranges[i] = scan2.ranges[i];
                scan1.intensities[i] = scan2.intensities[i];
            }
        }
    }

    sensor_msgs::msg::LaserScan new_clean_scan() {
        sensor_msgs::msg::LaserScan clean_scan;
        clean_scan.header.frame_id=new_frame;
        clean_scan.angle_min=angle_min; 
        clean_scan.angle_max=angle_max; 
        clean_scan.angle_increment=angle_increment;
        clean_scan.range_min=range_min; 
        clean_scan.range_max=range_max;

        if(auto_set){
            sensor_msgs::msg::LaserScan::SharedPtr ref_scan = transformed_scans[transformed_scans.begin()->first];
            clean_scan.time_increment=time_increment; 
            clean_scan.scan_time=scan_time; 
        }
        else{
            clean_scan.time_increment=time_increment; 
            clean_scan.scan_time=scan_time; 
        }

        int resolution = static_cast<int>(round((angle_max-angle_min)/angle_increment));
        //init ranges and add inf values according to the resolution
        clean_scan.ranges = {};
        for (int i = 0; i < resolution; ++i){
            clean_scan.ranges.push_back(INFINITY);
            clean_scan.intensities.push_back(0.0);
        }

        return clean_scan;
    }

    void init_sources_variables(){                 
        std::istringstream stream(sources); //convert string to stream
        std::string source_name;
        while (std::getline(stream, source_name, ' ')) {
            sources_var[source_name]["frame"] = "";
            sources_var[source_name]["frame_trans_vector"] = "";
            sources_var[source_name]["frame_rot_vector"] = "";
            sources_var[source_name]["last_timestamp"] = "";
        }
    }

    void initialize_common_params(){
        //classic parameters gathering
        //this->declare_parameter("use_sim_time",false); //already declared by launch
        this->declare_parameter("rate",1.0); 
        this->declare_parameter("topic_out","not_set"); 
        this->declare_parameter("new_frame","not_set");
        this->declare_parameter("debug",false);
        this->declare_parameter("debug_file_path","lasertoolbox_debug.txt");
        this->declare_parameter("show_ranges",false);
        this->declare_parameter("sources","not_set"); 
        this->declare_parameter("angle_min",0.0);
        this->declare_parameter("angle_max",0.0);
        this->declare_parameter("angle_increment",0.0);
        this->declare_parameter("range_min",0.0);
        this->declare_parameter("range_max",0.0);
        this->declare_parameter("auto_set",true);
        this->declare_parameter("time_increment",0.0);
        this->declare_parameter("scan_time",0.0);
    }

    void initialize_sources_params(){
        std::istringstream stream(sources); //convert string to stream
        std::string source_name;
        while (std::getline(stream, source_name, ' ')) {
            this->declare_parameter(source_name+".topic","not_set");
            this->declare_parameter(source_name+".start_angle",0.0);
            this->declare_parameter(source_name+".end_angle",0.0);
            this->declare_parameter(source_name+".scan_angle_offset",0.0);
        }
    }

    void refresh_common_params(){
        this->get_parameter("use_sim_time",use_sim_time); //managed by launch file
        this->get_parameter("rate",rate);                  
        this->get_parameter("topic_out",topic_out); 
        this->get_parameter("new_frame",new_frame);
        this->get_parameter("debug",debug);
        this->get_parameter("debug_file_path",debug_file_path); 
        this->get_parameter("show_ranges",show_ranges); 
        this->get_parameter("angle_min",angle_min);
        this->get_parameter("angle_max",angle_max);
        this->get_parameter("angle_increment",angle_increment);
        this->get_parameter("range_min",range_min);
        this->get_parameter("range_max",range_max);
        this->get_parameter("auto_set",auto_set);
        this->get_parameter("time_increment",time_increment);
        this->get_parameter("scan_time",scan_time);
        this->get_parameter("sources",sources);
    }

    void refresh_sources_params(){                 
        std::istringstream stream(sources); //convert string to stream
        std::string source_name;
        while (std::getline(stream, source_name, ' ')) {
            sources_config[source_name]["topic"] = this->get_parameter(source_name+".topic").as_string();
            sources_config[source_name]["start_angle"] = this->get_parameter(source_name+".start_angle").value_to_string();
            sources_config[source_name]["end_angle"] = this->get_parameter(source_name+".end_angle").value_to_string();
            sources_config[source_name]["scan_angle_offset"] = this->get_parameter(source_name+".scan_angle_offset").value_to_string();
        }
    }

    void debug_params(){
        std::stringstream debug_ss;
        debug_ss << "\nPARAMETERS:"
                << "\nuse_sim_time: " << use_sim_time
                << "\nrate: " << rate
                << "\ntopic_out: " << topic_out
                << "\nnew_frame: " << new_frame
                << "\ndebug: " << debug
                << "\ndebug_file_path: " << debug_file_path
                << "\nshow_ranges: " << show_ranges
                << "\nangle_min: " << angle_min
                << "\nangle_max: " << angle_max
                << "\nangle_increment: " << angle_increment
                << "\nrange_min: " << range_min
                << "\nrange_max: " << range_max
                << "\nauto_set: " << auto_set
                << "\ntime_increment: " << time_increment
                << "\nscan_time: " << scan_time
                << "\nsources: " << sources;

        for (const auto& pair1 : sources_config) {
            debug_ss << "\nSource: " << pair1.first;
            for (const auto& pair2 : pair1.second) {
                debug_ss << "\n  " << pair2.first << ": " << pair2.second;
            }
        }
        std::string debug_msg = debug_ss.str();
        write_debug(debug_file_path, debug_msg, false);
        //RCLCPP_INFO(this->get_logger(), "%s",debug_msg.c_str());
    }

    void write_debug(std::string file, std::string text,  bool append = true){
        std::lock_guard<std::mutex> lock(mutex_debug_file);
        std::ofstream debug_file_ss(file, append ? std::ios::app : std::ios::trunc);
        if (debug_file_ss.is_open()){
            debug_file_ss << text;
            debug_file_ss.close();
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Could not open file for debugging: %s",file.c_str());
        }
    }

private:
    /*
    Fixed variables
    */

    //common params
    double rate;
    std::string topic_out;
    std::string new_frame;
    bool use_sim_time = false;
    bool debug;
    std::string debug_file_path;
    bool show_ranges;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    bool auto_set;
    double time_increment;
    double scan_time;
    std::string sources;
    //sources params
    std::map<std::string, std::map<std::string, std::string>> sources_config; // Map to store sources and there data
    

    //transformations listening
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //other fixed variables
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subscriptions_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    /*
    Dynamic variables
    */

    //sources variables
    std::map<std::string, std::map<std::string, std::string>> sources_var; //allow to store custom variables for each sources, see: init_sources_variables()

    //scan datas
    std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> raw_scans; //raw scans from subscriptions
    std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> transformed_scans; //scans after transformations, filters...
    sensor_msgs::msg::LaserScan fused_scan; //final merged scan

    //clock
    builtin_interfaces::msg::Time simu_timestamp; //used for simuation
    rclcpp::Clock::SharedPtr clock; //used if not a simulation

    //concurrence
    std::mutex mutex_debug_file;


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanToolboxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

std::string vector3ToString(geometry_msgs::msg::Vector3& vector) {
    std::string result = "(" +
        std::to_string(vector.x) + ", " +
        std::to_string(vector.y) + ", " +
        std::to_string(vector.z) + ")";
    return result;
}

geometry_msgs::msg::Vector3 stringToVector3(std::string& str) {
    geometry_msgs::msg::Vector3 vector;
    std::stringstream ss(str);
    // Temporary variables to store parsed values
    double x, y, z;
    // Read values from stringstream
    char delim;
    ss >> delim >> x >> delim >> y >> delim >> z >> delim;
    // Assign parsed values to the vector
    vector.x = x;
    vector.y = y;
    vector.z = z;
    return vector;
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

geometry_msgs::msg::Vector3 adapt_angle(geometry_msgs::msg::Vector3 vect){
    //avoid flip with euleur angles at gimbla lock, work only for z component in that case.
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

double TimeToDouble(builtin_interfaces::msg::Time& stamp){
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

sensor_msgs::msg::LaserScan::SharedPtr convert_raw_data(sensor_msgs::msg::LaserScan::SharedPtr laser_raw,std::string new_frame,double start_angle, double end_angle, double angle_origin_offset, geometry_msgs::msg::Vector3 vector_newframe, geometry_msgs::msg::Vector3 rotate_newframe, std::stringstream &debug_ss){
    if (laser_raw != nullptr){
        //RCLCPP_INFO(this->get_logger(), "Lid1_run");
        //debug_ss << "/nDEBUG: vectx_= " << vector_newframe.x << "  vecty_= " << vector_newframe.y << "angle_= " << rotate_newframe.z << std::endl;
        //transform data and configure new message
        sensor_msgs::msg::LaserScan tempo = filter_data(laser_raw,start_angle, end_angle, angle_origin_offset);
        sensor_msgs::msg::LaserScan tempo2 = transform_data(std::make_shared<sensor_msgs::msg::LaserScan>(tempo),vector_newframe.x,vector_newframe.y,rotate_newframe.z,debug_ss);
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
    int resolution = msg->ranges.size();
    //we shift all the values to bring back the origin angle to the x axis
    int index_shift = angle_to_index(angle_origin_offset,resolution); //index which is the first angle for the lidar
    sensor_msgs::msg::LaserScan shifted_scan;
    shifted_scan.ranges.clear();
    for (int i = 0; i < resolution; ++i) { 
        if(i+index_shift < resolution){ 
            shifted_scan.ranges.push_back(msg->ranges[i+index_shift]);
        }
        else{
            shifted_scan.ranges.push_back(msg->ranges[i+index_shift-resolution]);
        }
    }

    //we keep the wanted angles
    sensor_msgs::msg::LaserScan filtered_scan;
    filtered_scan.ranges.clear();

    //compute index to consider according to min/max angles wanted
    int start_index = angle_to_index(start_angle, resolution);
    int end_index = angle_to_index(end_angle, resolution);

    // LaserScan message filling
    for (int i = 0; i < resolution; ++i) { 
        if(consider_val(i, start_index, end_index)){ //if the value is autorized, we add it
            filtered_scan.ranges.push_back(shifted_scan.ranges[i]);
        }
        else{
            filtered_scan.ranges.push_back(INFINITY);
        }
    }

    return filtered_scan;
}

int angle_to_index(double alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 angles.
    // Normalize the angle to the range [0, 2*M_PI)
    alpha = std::fmod(alpha, 2 * M_PI);
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    // Calculate the index
    return static_cast<int>(round((alpha * resolution) / (2*M_PI)));
}

double index_to_angle(int ind, int resolution){
    return (ind*(2*M_PI))/resolution;
}

void get_pos(double &x, double &y,double alpha,double val,double x_off,double y_off){
    x = val*cos(alpha)+x_off;
    y = val*sin(alpha)+y_off;
}

sensor_msgs::msg::LaserScan transform_data(sensor_msgs::msg::LaserScan::SharedPtr msg, double off_vect_x,double off_vect_y, double off_tetha, std::stringstream &debug_ss){
    /*
    msg: current message to transform
    off_vect_x: vector new_origin=>lidar
    off_vect_y: vector new_origin=>lidar
    return transformed_scan: message in which datas will be stored
    Homogoneous transformations: https://www.fil.univ-lille.fr/portail/archive21-22/~aubert/m3d/m3d_transformation.pdf
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

    //let's be sure to manipulate angles from 0 to 2pi
    if (off_tetha<0){
        off_tetha=2*M_PI+off_tetha;
    }
    if (off_tetha == 2*M_PI){
        off_tetha=0;
    }
    //translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = off_vect_x;
    T(1,3) = off_vect_y;
    //rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    R(0,0) = cos(off_tetha);
    R(0,1) = -sin(off_tetha);
    R(1,0) = sin(off_tetha);
    R(1,1) = cos(off_tetha);
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to sensor_frame
    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    //debug_ss << "DEBUG: Pass Matrix:" << std::endl;
    //debug_ss << M1_2 << std::endl;

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

        //points transformation
        if (!isinf(init_val)){ //if it is infinity, we don't need to compute as the default values in the new scan will be INFINITY
            get_pos(init_x, init_y,init_angle,init_val,0.0,0.0); //position in inital frame, vector: lidar=>point
            Eigen::MatrixXd X2(4, 1); //pos in sensor frame
            X2(0,0) = init_x;
            X2(1,0) = init_y;
            X2(2,0) = 0.0;
            X2(3,0) = 1.0;
            Eigen::MatrixXd X1(4, 1); //pos in newframe
            X1 = M1_2*X2;
            //Unpack
            double new_x2 = X1(0,0);
            double new_y2 = X1(1,0);
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

    return transformed_scan;
}

////TRASH

/*bool save_dataset = false;//if true, the code will generate a dataset with the first values of the first lidar in home/scan_merger_dataset.txt
sensor_msgs::msg::LaserScan transform_data(sensor_msgs::msg::LaserScan::SharedPtr msg, double off_vect_x,double off_vect_y, double off_tetha){
    
    //msg: current message to transform
    //off_vect_x: vector new_origin=>lidar
    //off_vect_y: vector new_origin=>lidar
    //return transformed_scan: message in which datas will be stored
    
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
}*/
