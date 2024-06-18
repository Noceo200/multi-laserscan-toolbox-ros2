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
#include "tools.h"
#include <stdexcept> // Include the standard exception header
#include "nav_msgs/msg/odometry.hpp"

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
        //Odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, sensor_qos, std::bind(&LaserScanToolboxNode::odomCallback, this, std::placeholders::_1));

        /*
        Publisher
        */

        //timer for publisher (RPLidars are around 7Hz for example), it will follow sensors update rates if they are slower or extrapolate data if activated
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_out, default_qos);
        timer_ = create_wall_timer(std::chrono::milliseconds(int(1000/rate)), std::bind(&LaserScanToolboxNode::fuseAndPublish, this));
    
        //initialize times
        update_stamp();
        current_source_latest_stamp = TimeToDouble(current_global_stamp);
        prev_source_latest_stamp = TimeToDouble(current_global_stamp);

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //Update odometry
        mutex_odom.lock();
        //odom_msg_new = msg;
        sav_odom(odom_msg_list, *msg, TimeToDouble(msg->header.stamp), odom_delay_limit);
        mutex_odom.unlock();

        /*if(odom_msg_list.size() > 0){
            std::stringstream debug_ss;
            debug_ss << std::fixed << "\nNb odometry saved: " << odom_msg_list.size()
                << "\nFirst Odom: " << TimeToDouble(odom_msg_list[0].header.stamp)
                << "\nLast Odom: " << TimeToDouble(odom_msg_list[odom_msg_list.size()-1].header.stamp)
                << std::endl;
            std::string debug_msg = debug_ss.str();
            RCLCPP_INFO(this->get_logger(), "%s",debug_msg.c_str());
        }*/
    }

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg,std::string source_name) {
        std::stringstream debug_ss;
        bool ready = false;
        debug_ss << std::fixed << "\n\n" << source_name << ": Scan received" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
        //we get frame and transformation of the sensor before to use the data
        if (sources_var[source_name]["frame"] == "" || sources_var[source_name]["frame_trans_vector"] == ""){
            sources_var[source_name]["frame"] = msg->header.frame_id;
            try {
                //get translation vector new_frame=>sensor
                geometry_msgs::msg::TransformStamped transf =tf_buffer_->lookupTransform(new_frame, sources_var[source_name]["frame"], tf2::TimePointZero);
                geometry_msgs::msg::Vector3 vector_newframe_sensor = transf.transform.translation;  //translation vector from new_frame to frame_sensor
                geometry_msgs::msg::Vector3 rotate_newframe_sensor = adapt_angle(quaternion_to_euler3D(transf.transform.rotation));
                //sav datas
                sources_var[source_name]["frame_trans_vector"] = vector3ToString(vector_newframe_sensor);
                sources_var[source_name]["frame_rot_vector"] = vector3ToString(rotate_newframe_sensor);
                debug_ss << source_name << ": Got transformation"
                         <<"\n  Translation: "<< sources_var[source_name]["frame_trans_vector"]
                         <<"\n  Euler rotation: "<< sources_var[source_name]["frame_rot_vector"] << std::endl;
                ready = true;
                //initialize the masks for persistence that will be updated after
                init_scan_masks(source_name,angle_increment); //generated for all sources, even if not used
            }
            catch (const std::exception& e) {
                debug_ss  << source_name << ": Couldn't get transformation " << new_frame << " --> " << sources_var[source_name]["frame"] << std::endl;
            }
        }
        else{
            ready = true;
        }

        //save scan
        if(ready){
            raw_scan_mutexes[source_name].lock();
            raw_scans[source_name] = msg;
            debug_ss << source_name << ": Raw scan updated" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            debug_ss << "  Raw scan timestamp: " << TimeToDouble(raw_scans[source_name]->header.stamp) << " s)" << std::endl;
            raw_scan_mutexes[source_name].unlock();
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

        //check if need to compute
        prev_source_latest_stamp = current_source_latest_stamp;
        for (const auto& pair1 : raw_scans) {
            std::string source_name = pair1.first;
            //getting last time stamp
            double last_stamp = TimeToDouble(raw_scans[source_name]->header.stamp); //we want last timestamp of current scan used
            if(last_stamp > current_source_latest_stamp){ //update latest source time
                current_source_latest_stamp = last_stamp; //we update time of latest raw_scan
            }
        }

        //compute classical sensors fusion
        if(raw_scans.size() > 0 && current_source_latest_stamp != prev_source_latest_stamp){
            //fusion variables created here because need 'resolution_360' wich is the futur final resolution of the scan considering it as a 360 deg one
            RCLCPP_INFO(this->get_logger(), "OK0");
            sensor_msgs::msg::LaserScan::SharedPtr fused_scan_360 = new_360_scan(); //the final fused scan can be a non 360 scan, but we first fuse evrything in a 360deg scan
            //the resolution should be same that every other transformed_scans from sources
            int resolution_360 = fused_scan_360->ranges.size();
            debug_ss << std::fixed << "\n\nStarting received scan transformations: Other sources may arrive later" << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            RCLCPP_INFO(this->get_logger(), "OK0.1: raw scans: %d",raw_scans.size());
            //Scan transformations and processing according to configurations
            for (const auto& pair1 : raw_scans) { //each raw scan need to be in same format (resolution and 360deg fov) of the fused_scan_360
                //Get new data
                RCLCPP_INFO(this->get_logger(), "OK0.2");
                std::string source_name = pair1.first;
                raw_scan_mutexes[source_name].lock();
                sensor_msgs::msg::LaserScan raw_scan = *raw_scans[source_name]; //copy the data
                raw_scan_mutexes[source_name].unlock();
                RCLCPP_INFO(this->get_logger(), "OK0.3");
                sensor_msgs::msg::LaserScan::SharedPtr raw_scan_ptr = std::make_shared<sensor_msgs::msg::LaserScan>(raw_scan);
                RCLCPP_INFO(this->get_logger(), "OK0.4");
                //get 360 equivalent filtered and transformed scan
                sensor_msgs::msg::LaserScan::SharedPtr transformed_scan_local = convert_raw_data_to_360_scan(raw_scan_ptr,scan_360_mask[source_name],stringToBool(sources_config[source_name]["persistence"]),raw_scans_persist[source_name],new_frame,std::stod(sources_config[source_name]["start_angle"]), std::stod(sources_config[source_name]["end_angle"]),std::stod(sources_config[source_name]["scan_angle_offset"]), std::stod(sources_config[source_name]["range_min"]), std::stod(sources_config[source_name]["range_max"]), stringToVector3(sources_var[source_name]["frame_trans_vector"]), stringToVector3(sources_var[source_name]["frame_rot_vector"]), debug_ss, source_name);
                transformed_scan_local->header.stamp = raw_scan_ptr->header.stamp;
                transformed_scans[source_name]=transformed_scan_local;
                debug_ss << source_name << ": Scan transformed, ready for global fusion..." << "(data stamp: " << TimeToDouble(transformed_scans[source_name]->header.stamp) << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
                RCLCPP_INFO(this->get_logger(), "OK7");
            }
            
            RCLCPP_INFO(this->get_logger(), "OK7.1");
            update_stamp();
            RCLCPP_INFO(this->get_logger(), "OK7.2");
            debug_ss <<"    |\n    |\n    |\n    V" << "\nStarting FUSION (Global timestamp: " << TimeToDouble(current_global_stamp) << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
            
            RCLCPP_INFO(this->get_logger(), "OK8");
            //Global Fusion
            nav_msgs::msg::Odometry ref_odom = get_estimated_odom(current_source_latest_stamp, odom_msg_list, extrapolate_odometry, false, debug_ss); //odom of newest LaserScan received
            double ref_heading = odom_to_heading(ref_odom);
            for (const auto& pair1 : transformed_scans) {
                std::string source_name = pair1.first;
                double dt_out = std::stod(sources_config[source_name]["timeout"]);
                double source_stamp = TimeToDouble(transformed_scans[source_name]->header.stamp);
                if(dt_out == 0.0 || source_stamp >= TimeToDouble(current_global_stamp)-dt_out){
                    //we correct the source scan to match how it should be at the time of the odometry of the newest source received
                    nav_msgs::msg::Odometry source_odom = get_estimated_odom(source_stamp, odom_msg_list, extrapolate_odometry, show_odometry_detail, debug_ss);
                    double x_off = ref_odom.pose.pose.position.x - source_odom.pose.pose.position.x;
                    double y_off = ref_odom.pose.pose.position.y - source_odom.pose.pose.position.y;
                    double source_heading = odom_to_heading(source_odom);
                    double tetha_off = sawtooth(ref_heading - source_heading);
                    debug_ss << source_name << ": Data late of (from newest scan): " << current_source_latest_stamp-source_stamp <<  " s, Odometry correction (x,y,tetha): (" << x_off << "," << y_off << "," << tetha_off << ")" << std::endl;
                    transform_360_data(transformed_scans[source_name],-x_off,-y_off,-tetha_off,debug_ss);
                    transformed_scans[source_name]->header.stamp = DoubleToTime(current_source_latest_stamp);
                    //global fusion
                    fuseScans(resolution_360, fused_scan_360, transformed_scans[source_name]);
                    debug_ss << "   fused_scan <-- " << source_name << " (raw data stamp: " << source_stamp << " s) (corrected data stamp: " << TimeToDouble(transformed_scans[source_name]->header.stamp) << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
                }
                else{
                    debug_ss << "fused_scan xxx " << source_name << " exceed its timeout value of " << dt_out << "s" << " (data stamp: " << TimeToDouble(transformed_scans[source_name]->header.stamp) << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
                }
            }
            RCLCPP_INFO(this->get_logger(), "OK9");

            //get last time
            update_stamp();
            //correct 360 fused scan to newest odometry
            /*nav_msgs::msg::Odometry newest_odom = get_estimated_odom(TimeToDouble(current_global_stamp), odom_msg_list, extrapolate_odometry, show_odometry_detail, debug_ss);
            double x_off = newest_odom.pose.pose.position.x - ref_odom.pose.pose.position.x;
            double y_off = newest_odom.pose.pose.position.y - ref_odom.pose.pose.position.y;
            double newest_heading = odom_to_heading(newest_odom);
            double tetha_off = sawtooth(newest_heading - ref_heading);
            debug_ss << "Final fused scan: Data late of (from current odometry): " << TimeToDouble(current_global_stamp)-current_source_latest_stamp <<  " s, Odometry correction (x,y,tetha): (" << x_off << "," << y_off << "," << tetha_off << ")" << std::endl;
            transform_360_data(fused_scan_360,-x_off,-y_off,-tetha_off,debug_ss);*/

            //we fill the final fused scan by keeping only the points that should be kept according to the angles selected, and the ranges
            fused_scan = new_clean_scan();
            int resolution = fused_scan->ranges.size();
            for(int i=0; i<resolution_360;i++){
                //By construction, Angle_incr will be the same,
                //So this remap will only be influenced if the wanted final scan have a different angle_start than 0.0. It just apply the offset in that case
                int new_i = remap_scan_index(i, fused_scan_360->angle_min, fused_scan_360->angle_max, resolution_360, fused_scan->angle_min, fused_scan->angle_max, resolution);
                double dist = fused_scan_360->ranges[i];
                //new_i  = index on a 360deg scan of angle_incr and start angle like fused_scan
                if(consider_val(new_i,0,resolution-1) && dist<=range_max && dist>=range_min){
                    if(dist < fused_scan->ranges[new_i]){
                        fused_scan->ranges[new_i] = dist; //new_i should naturally go from 0 to resolution because consider_val should prevent more or less values to go in
                        fused_scan->intensities[new_i] = fused_scan_360->intensities[i];
                    }
                }
            }

            //put clock
            //fused_scan->header.stamp = current_global_stamp; 
            fused_scan->header.stamp = DoubleToTime(current_source_latest_stamp); //based on last source stamp rather than global time

            // Publish the fused laser scan only if it have been update
            publisher_->publish(*fused_scan); 
        
            /*
            for (const auto& pair1 : transformed_scans) {
                std::string source_name = pair1.first;
                transformed_scans[source_name]->header.stamp = current_global_stamp;
                publisher_->publish(*transformed_scans[source_name]);
            }
            */
            
            //debug data
            int non_zero_vals = 0;
            int total_vals = fused_scan->ranges.size();
            if(debug){
                for(int i=0; i < total_vals; i++){
                    if(fused_scan->ranges[i] < INFINITY){
                        non_zero_vals ++;
                    }
                }
            }
            debug_ss << "    |\n    |\n    |\n    V"
                    << "\nfused_scan published" << " (node time: " << (this->now()).nanoseconds() << "ns)" 
                    << "\n  Frame: " <<  fused_scan->header.frame_id
                    << "\n  Timestamp: " <<  std::to_string(TimeToDouble(fused_scan->header.stamp))
                    << "\n  Number of rays: " << total_vals
                    << "\n  Number of hit: " << non_zero_vals
                    << std::endl;

            if(debug && show_ranges){
                debug_ss << "Ranges: ";
                for(int i=0; i < total_vals; i++){
                    debug_ss << fused_scan->ranges[i] << " ";
                }
            }

        }
        else if(extrapolate_scan && fused_scan!=nullptr){
            debug_ss << std::fixed << "\n\nNo source updated its former data but 'extrapolate_scan' is True: Checking if a new odometry is available to extrapolate the scan.";
        }
        else{
            debug_ss << "\n\nNo scan published. (No sources received yet or (no source updated its former data and 'extrapolate_scan' is False)), Activate 'extrapolate_scan' allows to update and publish the scan using the odometry even when the sensors did not update their datas." << " (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
        }

        //debug
        if(debug){
            std::string debug_msg = debug_ss.str();
            write_debug(debug_file_path, debug_msg);
        }

        /*
        catch(const std::exception& e){
            debug_ss << "Error fuseAndPublish: " << e.what() << "\n" << std::endl;
            std::string debug_msg = debug_ss.str();
            //RCLCPP_INFO(this->get_logger(), "ERROR fuseAndPublish: %s",debug_msg.c_str());
            if(debug){
                write_debug(debug_file_path, debug_msg);
            }
        }
        */
    }

    void fuseScans(int resolution, sensor_msgs::msg::LaserScan::SharedPtr scan1,sensor_msgs::msg::LaserScan::SharedPtr scan2, bool persistent_mode = false, std::string source_name = "") {
        //merge scan2 in scan1, they have to be same size.
        //if persistent_mode: the scan2 will override scan1 values when angles are between angle_min and angle_max. scan2 will update scan1 on its visible area.
        //fuse datas
        //RCLCPP_INFO(this->get_logger(), "size1: %d",scan1->ranges.size());
        //RCLCPP_INFO(this->get_logger(), "size2: %d",scan2->ranges.size());
        if(!persistent_mode){
            for (int i = 0; i < resolution; ++i) { 
                //we take the closest signal
                if(scan2->ranges[i]<scan1->ranges[i]){
                    scan1->ranges[i] = scan2->ranges[i];
                    scan1->intensities[i] = scan2->intensities[i];
                }
            }
        }
        else{
            for (int i = 0; i < resolution; ++i) { 
                //we use the masks to fuse with the former saved scan and we check for old value to delete
                double dt_out = std::stod(sources_config[source_name]["timeout_persistence"]);
                //we should be on 360 scan 
                if(scan_360_mask[source_name][i] == 1.0){ //if we should override the value, we override it
                    scan1->ranges[i] = scan2->ranges[i];
                    scan1->intensities[i] = scan2->intensities[i];
                    scan_360_times[source_name][i] = TimeToDouble(scan2->header.stamp);
                }
                else if((scan_360_times[source_name][i] < TimeToDouble(scan2->header.stamp)-dt_out) && dt_out != 0.0){ //time of scan2 should be time of the raw scan received in the case if persistence
                    scan1->ranges[i] = INFINITY;
                    scan1->intensities[i] = 0.0;
                }
                //otherwise we don't change scan1[i]
            }
        }
    }

    sensor_msgs::msg::LaserScan::SharedPtr new_clean_scan() {
        sensor_msgs::msg::LaserScan::SharedPtr clean_scan = std::make_shared<sensor_msgs::msg::LaserScan>();
        clean_scan->header.frame_id=new_frame;
        clean_scan->angle_min=angle_min; 
        clean_scan->angle_max=angle_max; 
        clean_scan->angle_increment=angle_increment;
        clean_scan->range_min=range_min; 
        clean_scan->range_max=range_max;

        if(auto_set){
            sensor_msgs::msg::LaserScan::SharedPtr ref_scan = transformed_scans[transformed_scans.begin()->first];
            clean_scan->time_increment=ref_scan->time_increment; 
            clean_scan->scan_time=ref_scan->scan_time; 
        }
        else{
            clean_scan->time_increment=time_increment; 
            clean_scan->scan_time=scan_time; 
        }

        int resolution = static_cast<int>(round((angle_max-angle_min)/angle_increment));
        //init ranges and add inf values according to the resolution
        clean_scan->ranges = {};
        for (int i = 0; i < resolution; ++i){
            clean_scan->ranges.push_back(INFINITY);
            clean_scan->intensities.push_back(0.0);
        }

        return clean_scan;
    }

    sensor_msgs::msg::LaserScan::SharedPtr new_360_scan() {
        try
        {
            sensor_msgs::msg::LaserScan::SharedPtr scan_360 = std::make_shared<sensor_msgs::msg::LaserScan>();
            scan_360->header.frame_id=new_frame;
            scan_360->angle_min=0.0; 
            scan_360->angle_max=2*M_PI; 
            scan_360->angle_increment=angle_increment;
            scan_360->range_min=range_min; 
            scan_360->range_max=range_max;

            if(auto_set){
                sensor_msgs::msg::LaserScan::SharedPtr ref_scan = raw_scans[raw_scans.begin()->first];
                scan_360->time_increment=ref_scan->time_increment; 
                scan_360->scan_time=ref_scan->scan_time; 
            }
            else{
                scan_360->time_increment=time_increment; 
                scan_360->scan_time=scan_time; 
            }

            int resolution = static_cast<int>(round((scan_360->angle_max-scan_360->angle_min)/angle_increment));
            //init ranges and add inf values according to the resolution
            scan_360->ranges = {};
            for (int i = 0; i < resolution; ++i){
                scan_360->ranges.push_back(INFINITY);
                scan_360->intensities.push_back(0.0);
            }
            return scan_360;
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "error catched");
            std::stringstream tempo;
            tempo << e.what();
            std::string tempo_str = tempo.str();
            RCLCPP_INFO(this->get_logger(), "Error: %s", tempo_str.c_str());
        }
        
    }

    void init_sources_variables(){                 
        std::istringstream stream(sources); //convert string to stream
        std::string source_name;
        while (std::getline(stream, source_name, ' ')) {
            sources_var[source_name]["frame"] = "";
            sources_var[source_name]["frame_trans_vector"] = "";
            sources_var[source_name]["frame_rot_vector"] = "";
            raw_scan_mutexes[source_name]; //this is enough to init the mutex
        }
    }

    void initialize_common_params(){
        //classic parameters gathering
        //this->declare_parameter("use_sim_time",false); //already declared by launch
        this->declare_parameter("rate",1.0); 
        this->declare_parameter("topic_out","not_set"); 
        this->declare_parameter("new_frame","not_set");
        this->declare_parameter("odom_topic", "not_set");
        this->declare_parameter("odom_delay_limit", 0.0);
        this->declare_parameter("extrapolate_odometry", false);
        this->declare_parameter("debug",false);
        this->declare_parameter("debug_file_path","lasertoolbox_debug.txt");
        this->declare_parameter("show_ranges",false);
        this->declare_parameter("show_odometry_detail",false);
        this->declare_parameter("sources","not_set"); 
        this->declare_parameter("angle_min",0.0);
        this->declare_parameter("angle_max",0.0);
        this->declare_parameter("angle_increment",0.0);
        this->declare_parameter("range_min",0.0);
        this->declare_parameter("range_max",0.0);
        this->declare_parameter("extrapolate_scan",false);
        this->declare_parameter("auto_set",true);
        this->declare_parameter("time_increment",0.0);
        this->declare_parameter("scan_time",0.0);
    }

    void initialize_sources_params(){
        std::istringstream stream(sources); //convert string to stream
        std::string source_name;
        while (std::getline(stream, source_name, ' ')) {
            this->declare_parameter(source_name+".topic","not_set");
            this->declare_parameter(source_name+".timeout",0.0);
            this->declare_parameter(source_name+".start_angle",0.0);
            this->declare_parameter(source_name+".end_angle",0.0);
            this->declare_parameter(source_name+".scan_angle_offset",0.0);
            this->declare_parameter(source_name+".range_min",0.0);
            this->declare_parameter(source_name+".range_max",0.0);
            this->declare_parameter(source_name+".persistence",true);
            this->declare_parameter(source_name+".timeout_persistence",0.0);
        }
    }

    void refresh_common_params(){
        this->get_parameter("use_sim_time",use_sim_time); //managed by launch file
        this->get_parameter("rate",rate);                  
        this->get_parameter("topic_out",topic_out); 
        this->get_parameter("new_frame",new_frame);
        this->get_parameter("odom_topic", odom_topic);
        this->get_parameter("odom_delay_limit", odom_delay_limit);
        this->get_parameter("extrapolate_odometry", extrapolate_odometry);
        this->get_parameter("debug",debug);
        this->get_parameter("debug_file_path",debug_file_path); 
        this->get_parameter("show_ranges",show_ranges); 
        this->get_parameter("show_odometry_detail",show_odometry_detail); 
        this->get_parameter("angle_min",angle_min);
        this->get_parameter("angle_max",angle_max);
        this->get_parameter("angle_increment",angle_increment);
        this->get_parameter("range_min",range_min);
        this->get_parameter("range_max",range_max);
        this->get_parameter("extrapolate_scan",extrapolate_scan);
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
            sources_config[source_name]["timeout"] = this->get_parameter(source_name+".timeout").value_to_string();
            sources_config[source_name]["start_angle"] = this->get_parameter(source_name+".start_angle").value_to_string();
            sources_config[source_name]["end_angle"] = this->get_parameter(source_name+".end_angle").value_to_string();
            sources_config[source_name]["scan_angle_offset"] = this->get_parameter(source_name+".scan_angle_offset").value_to_string();
            sources_config[source_name]["range_min"] = this->get_parameter(source_name+".range_min").value_to_string();
            sources_config[source_name]["range_max"] = this->get_parameter(source_name+".range_max").value_to_string();
            sources_config[source_name]["persistence"] = this->get_parameter(source_name+".persistence").value_to_string();
            sources_config[source_name]["timeout_persistence"] = this->get_parameter(source_name+".timeout_persistence").value_to_string();
        }
    }

    void debug_params(){
        std::stringstream debug_ss;
        debug_ss << "\nPARAMETERS:"
                << "\nuse_sim_time: " << use_sim_time
                << "\nrate: " << rate
                << "\ntopic_out: " << topic_out
                << "\nnew_frame: " << new_frame
                << "\nodom_topic: " << odom_topic
                << "\nodom_delay_limit: " << odom_delay_limit
                << "\nextrapolate_odometry: " << extrapolate_odometry
                << "\ndebug: " << debug
                << "\ndebug_file_path: " << debug_file_path
                << "\nshow_ranges: " << show_ranges
                << "\nshow_odometry_detail: " << show_odometry_detail
                << "\nangle_min: " << angle_min
                << "\nangle_max: " << angle_max
                << "\nangle_increment: " << angle_increment
                << "\nrange_min: " << range_min
                << "\nrange_max: " << range_max
                << "\nextrapolate_scan: " << extrapolate_scan
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

    sensor_msgs::msg::LaserScan::SharedPtr convert_raw_data_to_360_scan(sensor_msgs::msg::LaserScan::SharedPtr laser_raw, std::vector<double> &mask_raw_filtered_360, bool persistence, sensor_msgs::msg::LaserScan::SharedPtr laser_raw_filtered_persist, std::string new_frame,double start_angle, double end_angle, double angle_origin_offset, double min_range, double max_range, geometry_msgs::msg::Vector3 vector_newframe, geometry_msgs::msg::Vector3 rotate_newframe, std::stringstream &debug_ss, std::string source_name){
        if (laser_raw != nullptr){
            //Configure scan into same format that the final one
            sensor_msgs::msg::LaserScan::SharedPtr transformed_scan = new_360_scan();
            //remap points to match the new resolutions and have a 360 circle
            //this remap will only be influenced if the source as not an origin of 0.0 and if the angle_increment is different from the wanted final scan

            remap_scan(laser_raw,transformed_scan,mask_raw_filtered_360);

            //filter data we want to keep
            filter_360_data(transformed_scan, mask_raw_filtered_360, start_angle, end_angle, angle_origin_offset, min_range, max_range, debug_ss); //cut, crop data according to wanted angle for this source + add offset

            //DEBUG Mask START
            int count1 = 0;
            debug_ss << "\nMASK DEBUG after filter: [";
            for(int i =0; i<mask_raw_filtered_360.size() ;i++){
                debug_ss << int(round(mask_raw_filtered_360[i])) << ";"; 
                if(mask_raw_filtered_360[i]==1.0){
                    count1 ++;
                }
            }
            debug_ss << "]\n " << count1 << std::endl;
            //DEBUG Mask END

            RCLCPP_INFO(this->get_logger(), "OK1");

            RCLCPP_INFO(this->get_logger(), "OK2");
            //If persistence, we fuse the not transformed datas
            if(persistence){
                builtin_interfaces::msg::Time new_raw_time = laser_raw->header.stamp; //we can not use transformed_scan header yet as its time is not updated here yet
                //local fusion
                if(laser_raw_filtered_persist == nullptr){ 
                    //We initialize the saved scan for the first time
                    RCLCPP_INFO(this->get_logger(), "OK3");
                    raw_scans_persist[source_name] = new_360_scan(); //we initialize directly with source_name, otherwise it wouldn't works
                    RCLCPP_INFO(this->get_logger(), "OK3.1");
                    copy_ranges(raw_scans_persist[source_name], transformed_scan);
                    RCLCPP_INFO(this->get_logger(), "OK3.2: len raw_scans_persist[source_name]: %d",raw_scans_persist[source_name]->ranges.size());
                    raw_scans_persist[source_name]->header.stamp = new_raw_time;
                    RCLCPP_INFO(this->get_logger(), "OK3.3");
                    debug_ss << source_name << ": Persistence ON, raw filtered scan saved in 360 persistent raw scan for fusion with future raw filtered values, ready for transformation..." << "(raw filtered data stamp: " << TimeToDouble(raw_scans_persist[source_name]->header.stamp) << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
                    RCLCPP_INFO(this->get_logger(), "OK3.4");
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "OK4");
                    //we transform the former scan values according to how the bot moved between the 2 measures
                    //we get odometry difference
                    debug_ss << source_name << ": Persistence ON, getting odometries for older 360 persistent raw scan and new raw filtered values..." << std::endl;
                    nav_msgs::msg::Odometry last_odom = get_estimated_odom(TimeToDouble(laser_raw_filtered_persist->header.stamp), odom_msg_list, extrapolate_odometry, show_odometry_detail, debug_ss);
                    nav_msgs::msg::Odometry new_odom = get_estimated_odom(TimeToDouble(new_raw_time), odom_msg_list, extrapolate_odometry, show_odometry_detail, debug_ss);
                    double x_off = new_odom.pose.pose.position.x - last_odom.pose.pose.position.x;
                    double y_off = new_odom.pose.pose.position.y - last_odom.pose.pose.position.y;
                    double last_heading = odom_to_heading(last_odom);
                    double new_heading = odom_to_heading(new_odom);
                    double tetha_off = sawtooth(new_heading - last_heading);
                    RCLCPP_INFO(this->get_logger(), "OK4.1");
                    if(x_off != 0.0 || y_off != 0.0 || tetha_off != 0.0 ){
                        RCLCPP_INFO(this->get_logger(), "OK4.1.2");
                        transform_360_data(laser_raw_filtered_persist,-x_off,-y_off,-tetha_off,debug_ss);
                        RCLCPP_INFO(this->get_logger(), "OK4.1.3");
                        debug_ss << source_name << ": Persistence ON, motion detected, Former 360 persistent raw scan adjusted. (x,y,tetha) offset: (" << x_off << "," << y_off << "," << tetha_off << ") m, rad " << std::endl;
                    }
                    //the data is now at the time of the new received one
                    laser_raw_filtered_persist->header.stamp = new_raw_time;
                    //we update the masks according to the scan, only the time mask need to be updated
                    //TDM update_time_mask(scan_360_mask[source_name],resolution_360,scan_360_times[source_name],TimeToDouble(transformed_scans[source_name]->header.stamp)); 
                    //we fuse the scan with the former values, the new scan will update the former values only on its FOV equivalence on a 360 deg scan.
                    //local fusion
                    fuseScans(laser_raw_filtered_persist->ranges.size(),laser_raw_filtered_persist, transformed_scan, true, source_name);
                    RCLCPP_INFO(this->get_logger(), "OK4.2");
                    //we copy data in transformed_scan
                    copy_ranges(transformed_scan, laser_raw_filtered_persist);
                    RCLCPP_INFO(this->get_logger(), "OK4.3");
                    debug_ss << source_name << ": Raw filtered Scan fused with 360 persistent raw scan, ready for transformation..." << "(data stamp: " << TimeToDouble(transformed_scans[source_name]->header.stamp) << " s) (node time: " << (this->now()).nanoseconds() << "ns)" << std::endl;
                }
            }

            RCLCPP_INFO(this->get_logger(), "OK5");

            //compute new points in the wanted ouput frame
            transform_360_data(transformed_scan,vector_newframe.x,vector_newframe.y,rotate_newframe.z,debug_ss); //compute new points in output frame
            
            RCLCPP_INFO(this->get_logger(), "OK6");

            //compute the mask in a 360 final scan, using existing other function is too anoying as they are in tools.cpp 
            return transformed_scan;
        }
        else{
            return nullptr;
        }
    }

    void update_stamp(){
        if(use_sim_time){ //if use_sim_time = true
            current_global_stamp = simu_timestamp;
        }
        else{
            current_global_stamp = clock->now();
        }
    }

    void init_scan_masks(std::string source_name, double increment){
        int n = static_cast<int>(round((2*M_PI)/increment)); //size that would have a 360 scan of this source
        scan_360_mask[source_name] = std::vector<double>(n, 0.0);
        scan_360_times[source_name] = std::vector<double>(n, 0.0);
    }

    void update_time_mask(std::vector<double> mask_360_apply, int reso_360, std::vector<double> &mask_360_time_to_update, double data_time){
        for(int i =0; i<reso_360; i++){
            if(mask_360_apply[i] == 1.0){
                mask_360_time_to_update[i] = data_time;
            }
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
    std::string odom_topic;
    double odom_delay_limit;
    bool extrapolate_odometry;
    bool use_sim_time = false;
    bool debug;
    std::string debug_file_path;
    bool show_ranges;
    bool show_odometry_detail;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    bool extrapolate_scan;
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_; 
    /*
    Dynamic variables
    */
    //sources variables
    std::map<std::string, std::map<std::string, std::string>> sources_var; //allow to store custom variables for each sources, see: init_sources_variables()
    //scan datas
    std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> raw_scans; //raw scans from subscriptions
    std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> transformed_scans; //scans after transformations, filters...
    std::map<std::string, sensor_msgs::msg::LaserScan::SharedPtr> raw_scans_persist; //scans for persistence
    std::map<std::string, std::vector<double>> scan_360_mask; //0 or 1, used to know what values we should keep from a raw scan on a 360 scan with final resolution, used for the persistence
    std::map<std::string, std::vector<double>> scan_360_times; //used to know the last times a value have been updated in a final 360 scan, allow to delete old ones, used for persistence aging
    sensor_msgs::msg::LaserScan::SharedPtr fused_scan; //final merged scan
    //clock
    builtin_interfaces::msg::Time simu_timestamp; //used for simuation
    rclcpp::Clock::SharedPtr clock; //used if not a simulation
    builtin_interfaces::msg::Time current_global_stamp; //store current time
    double current_source_latest_stamp;
    double prev_source_latest_stamp;
    //odometry
    std::vector<nav_msgs::msg::Odometry> odom_msg_list;
    //nav_msgs::msg::Odometry::SharedPtr odom_msg_former;
    //nav_msgs::msg::Odometry::SharedPtr odom_msg_new;
    //concurrence
    std::mutex mutex_debug_file;
    std::map<std::string, std::mutex> raw_scan_mutexes;
    std::mutex mutex_odom;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanToolboxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


////TRASH

/*bool save_dataset = false;//if true, the code will generate a dataset with the first values of the first lidar in home/scan_merger_dataset.txt
sensor_msgs::msg::LaserScan transform_360_data(sensor_msgs::msg::LaserScan::SharedPtr msg, double off_vect_x,double off_vect_y, double off_tetha){
    
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


/*
    std::string print_exception(const std::exception& e, int level = 0) {
        std::stringstream err_ss;
        // Print indentation for nested exceptions
        for (int i = 0; i < level; ++i) {
            err_ss << "  ";
        }
        // Print the exception message
        err_ss << "Exception: " << e.what() << std::endl;
        try {
            // Try to re-throw the nested exception
            std::rethrow_if_nested(e);
        } catch(const std::exception& nested) {
            // Recursively print nested exceptions
            print_exception(nested, level + 1);
        } catch(...) {
            // Ignore non-standard exceptions
        }

        std::string err_msg = err_ss.str();
        return err_msg;
    }



    if(odom_msg_new != nullptr){
        mutex_odom.lock();
        nav_msgs::msg::Odometry::SharedPtr current_odom(new nav_msgs::msg::Odometry(*odom_msg_new)); //copy data
        mutex_odom.unlock();
        if(odom_msg_former != nullptr){ //if we have former odometry, we compute the offset
            geometry_msgs::msg::Vector3 euler_heading_former = adapt_angle(quaternion_to_euler3D(odom_msg_former->pose.pose.orientation)); //previous heading
            geometry_msgs::msg::Vector3 euler_heading_new = adapt_angle(quaternion_to_euler3D(current_odom->pose.pose.orientation)); //new heading
            x_off = current_odom->pose.pose.position.x - odom_msg_former->pose.pose.position.x;
            y_off = current_odom->pose.pose.position.y - odom_msg_former->pose.pose.position.y;
            tetha_off = sawtooth(euler_heading_new.z - euler_heading_former.z);
            //off_set_debug = euler_heading_new.z; //debuging persistence 
            odom_msg_former = current_odom;
        }
        else{ //otherwise we just save odometry for next frame
            odom_msg_former = current_odom;
        }
    }
    else{
        debug_ss << source_name << ":Persistence ON, but no Odometry messages received yet on topic '" << odom_topic << "'. Persistence will not work until odometry is received." << std::endl;
    }

    //START for debuging persistence areas
    double start_angle_debug = degrees_to_rads(170)-off_set_debug; //angles in map frame
    double end_angle_debug = degrees_to_rads(190)-off_set_debug;
    int start_ind_debug = angle_to_index(start_angle_debug,resolution_360); //indexes in scan, so in robot frame
    int end_ind_debug = angle_to_index(end_angle_debug,resolution_360);
    debug_ss << "Persitence area: (" << rads_to_degrees(start_angle_debug) << ";" 
                                    << rads_to_degrees(end_angle_debug) << ";"
                                    << rads_to_degrees(off_set_debug) << ";"
                                    << start_ind_debug << ";"
                                    << end_ind_debug << ")\n[";
    int hits = 0;
    for(int y=0;y<transformed_scans[source_name]->ranges.size();y++){
        if(consider_val(y,start_ind_debug,end_ind_debug)){ 
            debug_ss << transformed_scans[source_name]->ranges[y] << ";";
            if(transformed_scans[source_name]->ranges[y] != INFINITY){
                hits ++;
            }
        }
    }
    debug_ss << "] \nHITS: "<< hits << std::endl;
    // debugging END
*/