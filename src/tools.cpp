/*
LAST MODIF(DD/MM/YYYY): 09/04/2024
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <iostream> 
#include "rosgraph_msgs/msg/clock.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <cmath>
#include <Eigen/Dense>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tools.h"

int copy_ranges(sensor_msgs::msg::LaserScan::SharedPtr host_scan, sensor_msgs::msg::LaserScan::SharedPtr target_scan){
    try
    {
        int reso = host_scan->ranges.size();
        for (int i = 0; i < reso; ++i) { 
            host_scan->ranges[i] = target_scan->ranges[i];
        }
        return 1;
    }
    catch(const std::exception& e)
    {
        return 0;
    }
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

geometry_msgs::msg::TransformStamped tf_offset(geometry_msgs::msg::TransformStamped &relative_transform, geometry_msgs::msg::TransformStamped tf1, geometry_msgs::msg::TransformStamped tf2){
    //give transformation from tf1 to tf2

    Eigen::Quaterniond rot_1(tf1.transform.rotation.w,
                            tf1.transform.rotation.x,
                            tf1.transform.rotation.y,
                            tf1.transform.rotation.z);

    Eigen::Quaterniond rot_2(tf2.transform.rotation.w,
                            tf2.transform.rotation.x,
                            tf2.transform.rotation.y,
                            tf2.transform.rotation.z);

    // Perform quaternion subtraction
    Eigen::Quaterniond relative_rotation = rot_2 * rot_1.inverse();

    // Subtract translation components
    double off_x = tf2.transform.translation.x - tf1.transform.translation.x;
    double off_y = tf2.transform.translation.y - tf1.transform.translation.y;
    double off_z = tf2.transform.translation.z - tf1.transform.translation.z;

    // relative_transform represents the difference between the two original transforms
    relative_transform.header = tf2.header;
    relative_transform.transform.rotation.w = relative_rotation.w();
    relative_transform.transform.rotation.x = relative_rotation.x();
    relative_transform.transform.rotation.y = relative_rotation.y();
    relative_transform.transform.rotation.z = relative_rotation.z();
    relative_transform.transform.translation.x = off_x;
    relative_transform.transform.translation.y = off_y;
    relative_transform.transform.translation.z = off_z;

}


void shift_cloud_from_tf(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, geometry_msgs::msg::TransformStamped transf){
    //transf need to be the transformation from new_frame to old_frame
    std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> Pass_matrixes_new_old = get_4Dmatrix_from_transform(transf);
    //transform each points
    for(int i=0; i<cloud->size(); i++){
        Eigen::MatrixXd Point_old = Eigen::MatrixXd::Identity(4, 1);
        Point_old(0,0) = cloud->points[i].x;
        Point_old(1,0) = cloud->points[i].y;
        Point_old(2,0) = cloud->points[i].z;
        Point_old(3,0) = 1.0;
        
        Eigen::MatrixXd Point_new = std::get<2>(Pass_matrixes_new_old)*Point_old;

        cloud->points[i].x = Point_new(0,0);
        cloud->points[i].y = Point_new(1,0);
        cloud->points[i].z = Point_new(2,0);
    }
}

void MatProd_fast4_4(double (&M)[4][4],double (&A)[4][4],double (&B)[4][4]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 4; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 4; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void MatProd_fast3_3(double (&M)[3][3],double (&A)[3][3],double (&B)[3][3]){
    for(int i =0; i< 3; i++){
        for(int j =0; j< 3; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 3; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void MatProd_fast4_Vect(double (&M)[4][1],double (&A)[4][4],double (&B)[4][1]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 1; j++){
            M[i][j] = 0.0;
            for(int s =0; s< 4; s++){
                M[i][j] += A[i][s]*B[s][j];
            }
        }
    }
}

void to_identity3_3(double (&mat)[3][3]){
    for(int i =0; i< 3; i++){
        for(int j =0; j< 3; j++){
            if(j==i){
                mat[i][j]=1.0;
            }
            else{
                mat[i][j]=0.0;
            }
        }
    }
}

void to_identity4_4(double (&mat)[4][4]){
    for(int i =0; i< 4; i++){
        for(int j =0; j< 4; j++){
            if(j==i){
                mat[i][j]=1.0;
            }
            else{
                mat[i][j]=0.0;
            }
        }
    }
}

double scalar_projection_fast(double (&a)[3], double (&b)[3]) {
    //projection of 'a' into 'b': https://en.wikipedia.org/wiki/Vector_projection
    // Calculate dot product of a and b
    double dot_product = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    
    // Calculate squared norm of vector b
    double norm_b_squared = pow(b[0],2) + pow(b[1],2) + pow(b[2],2);
    
    // Calculate the projection
    double projection = (dot_product / norm_b_squared);
    
    return projection;
}

double scalar_projection(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    //projection of 'a' into 'b': https://en.wikipedia.org/wiki/Vector_projection
    // Calculate dot product of a and b
    double dot_product = a.dot(b);
    
    // Calculate squared norm of vector b
    double norm_b_squared = b.squaredNorm();
    
    // Calculate the projection
    double projection = (dot_product / norm_b_squared);
    
    return projection;
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> get_4Dmatrix_from_transform(geometry_msgs::msg::TransformStamped transf){
    geometry_msgs::msg::Vector3 translation_ = transf.transform.translation;  //translation vector from new_frame to frame_sensor
    geometry_msgs::msg::Vector3 rotation_ = quaternion_to_euler3D(transf.transform.rotation);
    //Translation in homogeneous coordinates
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    T(0,3) = translation_.x;
    T(1,3) = translation_.y;
    T(2,3) = translation_.z;
    //3D rotation matrix
    Eigen::MatrixXd R_temp = rot_matrix_from_euler(rotation_);
    //3D rotation in homogeneous coordinates
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4, 4);
    for(int i = 0; i<3; i++){
        for(int y = 0; y<3; y++){
            R(i,y) = R_temp(i,y);
        }
    }
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to init_frame

    Eigen::MatrixXd M1_2(4, 4);
    M1_2 = T*R;

    return std::make_tuple(T, R, M1_2);

}

void get_4Dmatrix_from_transform_fast(double (&M)[4][4],geometry_msgs::msg::TransformStamped transf){
    geometry_msgs::msg::Vector3 translation_ = transf.transform.translation;  //translation vector from new_frame to frame_sensor
    geometry_msgs::msg::Vector3 rotation_ = quaternion_to_euler3D(transf.transform.rotation);
    //Translation in homogeneous coordinates
    double T[4][4] = {
                    {1.0,0.0,0.0,translation_.x},
                    {0.0,1.0,0.0,translation_.y},
                    {0.0,0.0,1.0,translation_.z},
                    {0.0,0.0,0.0,1.0}
                    };
    //3D rotation matrix
    double R_temp[3][3];
    rot_matrix_from_euler_fast(R_temp,rotation_);
    //3D rotation in homogeneous coordinates
    double R[4][4];
    to_identity4_4(R);
    for(int i = 0; i<3; i++){
        for(int y = 0; y<3; y++){
            R[i][y] = R_temp[i][y];
        }
    }
    //final homogeneous transformation, M1_2 = pass matrix, displacement from new_frame to init_frame
    double M1_2[4][4];
    MatProd_fast4_4(M1_2,T,R);
}

Eigen::MatrixXd rot_matrix_from_euler(geometry_msgs::msg::Vector3 euler_angles){
    Eigen::MatrixXd Rx = Eigen::MatrixXd::Identity(3, 3);
    Rx(1,1) = cos(euler_angles.x);
    Rx(1,2) = -sin(euler_angles.x);
    Rx(2,1) = sin(euler_angles.x);
    Rx(2,2) = cos(euler_angles.x);
    Eigen::MatrixXd Ry = Eigen::MatrixXd::Identity(3, 3);
    Ry(0,0) = cos(euler_angles.y);
    Ry(0,2) = sin(euler_angles.y);
    Ry(2,0) = -sin(euler_angles.y);
    Ry(2,2) = cos(euler_angles.y);
    Eigen::MatrixXd Rz = Eigen::MatrixXd::Identity(3, 3);
    Rz(0,0) = cos(euler_angles.z);
    Rz(0,1) = -sin(euler_angles.z);
    Rz(1,0) = sin(euler_angles.z);
    Rz(1,1) = cos(euler_angles.z);
    return Rx*Ry*Rz;
}

void rot_matrix_from_euler_fast(double (&R)[3][3], geometry_msgs::msg::Vector3 euler_angles){
    double Rx[3][3];
    to_identity3_3(Rx);
    Rx[1][1] = cos(euler_angles.x);
    Rx[1][2] = -sin(euler_angles.x);
    Rx[2][1] = sin(euler_angles.x);
    Rx[2][2] = cos(euler_angles.x);
    double Ry[3][3];
    to_identity3_3(Ry);
    Ry[0][0] = cos(euler_angles.y);
    Ry[0][2] = sin(euler_angles.y);
    Ry[2][0] = -sin(euler_angles.y);
    Ry[2][2] = cos(euler_angles.y);
    double Rz[3][3];
    to_identity3_3(Rz);
    Rz[0][0] = cos(euler_angles.z);
    Rz[0][1] = -sin(euler_angles.z);
    Rz[1][0] = sin(euler_angles.z);
    Rz[1][1] = cos(euler_angles.z);
    double R_temp[3][3];
    MatProd_fast3_3(R_temp,Ry,Rz);
    MatProd_fast3_3(R,Rx,R_temp);
}

geometry_msgs::msg::Vector3 quaternion_to_euler3D(geometry_msgs::msg::Quaternion quat){
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
    return rot_vect;
}

double TimeToDouble(builtin_interfaces::msg::Time& stamp){
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
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

int angle_to_index(double alpha, int resolution){
    //return index of angle alpha, in a table with 'resolution' values placed from 0 to 360 degree.
    // Normalize the angle to the range [0, 2*M_PI)
    alpha = std::fmod(alpha, 2 * M_PI);
    //debug_ss << "\nAngle_to_index: "<< " mod: " << alpha;
    if (alpha < 0) {
        alpha += 2 * M_PI;
    }
    // Calculate the index
    int ind = static_cast<int>(round((alpha * resolution)/(2*M_PI)));
    //debug_ss <<  " alpha*reso: " << alpha*resolution << " ind: " << ind;
    return ind;
}

double index_to_angle(int ind, int resolution){
    return (ind*(2*M_PI))/resolution;
}

int remap_scan_index(int prev_ind, double prev_angle_start, double prev_angle_end, double prev_reso, double new_angle_start, double new_angle_end, double new_reso){
    int new_ind;
    /*
    return the index in a new scan list.
    */
    double prev_elong = prev_angle_end - prev_angle_start;  
    double new_elong = new_angle_end - new_angle_start;
    double prev_angle_incr = prev_elong/prev_reso;
    double new_angle_incr = new_elong/new_reso;
    int reso_360 = static_cast<int>(round(2*M_PI/prev_angle_incr)); //resoluton that would have prev_list if on a 360deg circle list

   //offset gestion
    double angle_offset = sawtooth(prev_angle_start-new_angle_start);
    if(angle_offset<0){
        angle_offset += 2*M_PI;
    }
    //should have offset between [0,2Pi]
    int ind_offset = angle_to_index(angle_offset,reso_360); //should return index between [0,reso_360]
    new_ind = static_cast<int>(round(fmod(prev_ind + ind_offset,reso_360)));
    //different reso gestion 
    new_ind = static_cast<int>(round((prev_angle_incr*new_ind)/new_angle_incr));
    
    return new_ind;
}

double sawtooth(double x) {
    return std::fmod(x+M_PI,2*M_PI)-M_PI;
}

