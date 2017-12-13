#include "controller/latpp.hpp"

PurePursuit::PurePursuit(void)
{

}

PurePursuit::~PurePursuit(void)
{
  ofs.close();
  ifs.close();
}

MatrixXf PurePursuit::g2b(double theta){
  MatrixXf Rotate_Matrix = MatrixXf::Zero(2,2);  //rotate matrix;

  Rotate_Matrix(0,0) = cos(theta);
  Rotate_Matrix(0,1) = sin(theta);
  Rotate_Matrix(1,0) = -sin(theta);
  Rotate_Matrix(1,1) = cos(theta);

  return Rotate_Matrix;
}

bool PurePursuit::init(float ts, float rate){
  this->ts = ts;
  this->rate = rate;
  this->wheelbase = 2.8498; // 112.2 inches
  this->steering_ratio = 14.8;

  string logfilepath = "/auto_ws/data/controller/debug_pp/";
  string current_date = get_current_date();
  ofs.open(logfilepath + current_date + "_pp_debug.txt");

  return true;
}

tuple<geometry_msgs::Pose2D, float> PurePursuit::update(vector<geometry_msgs::Pose2D> &pose_array, geometry_msgs::Pose2D car_pose, float v_car_, float steer_cur){
  v_car = v_car_;
  //step1:nearest point;
  int nearest_index = get_nearest_index(car_pose, pose_array);

  //step2:way point index;
  //float yaw_dis_preview = 4 + 1.7*v_car;
  float yaw_dis_preview = 1 + 7*sqrt(v_car);
  int yaw_index_preview = get_preview_index(pose_array, car_pose, yaw_dis_preview);
  if(yaw_index_preview == -1) {
    ROS_ERROR("no preview pt!!!");
  }
  geometry_msgs::Pose2D preview_pose = pose_array[yaw_index_preview];
  int waypoint_index = yaw_index_preview; //can change the second term ;

  //step3:Coordinate Transform;
  MatrixXf WayPoint_In_Vehicle = MatrixXf::Zero(2,1);
  MatrixXf Rotate_Matrix = g2b(M_PI/2-car_pose.theta);

  //step4:WayPoint in Vehicle;
  WayPoint_In_Vehicle(0,0) = pose_array[waypoint_index].x - car_pose.x;
  WayPoint_In_Vehicle(1,0) = pose_array[waypoint_index].y - car_pose.y;
  WayPoint_In_Vehicle = Rotate_Matrix*WayPoint_In_Vehicle;

  //step5:Calculate Curvature;
  double ld = sqrt(pow(WayPoint_In_Vehicle(0,0),2) + pow(WayPoint_In_Vehicle(1,0),2)); //distance to waypoint;
  double radius = pow(ld,2)/2.0/WayPoint_In_Vehicle(1,0); //radius;
  double kappa = 1/radius;

  float steer_angle = atan(wheelbase*kappa)*steering_ratio; //RAD;

  float d_lookahead = WayPoint_In_Vehicle(0,0);
  float delta_f = steer_cur/steering_ratio;
  float kappa2 = tan(delta_f)/wheelbase;
  float alpha = asin(clip(d_lookahead*kappa2,-0.999,0.999))/2;
  float y_actual = - d_lookahead * tan(alpha);
  float y_des = WayPoint_In_Vehicle(1,0);

  ROS_INFO("y_des: %.4f, y_actual: %.4f, steer: %f, kappa: %f",y_des, y_actual, steer_angle*180/M_PI,kappa);

  ofs << steer_angle << "\t" <<car_pose.x << "\t" << car_pose.y << "\t"
      << pose_array[nearest_index].x << "\t" << pose_array[nearest_index].y
      << "\t" << y_des << "\t" << y_actual
      << endl;

  //  steer_angle = 0;
  return make_tuple(preview_pose,steer_angle);
}
tuple<geometry_msgs::Pose2D, float> PurePursuit::update2(vector<geometry_msgs::Pose2D> &pose_array, geometry_msgs::Pose2D car_pose, float v_car_, float steer_cur){

    v_car = v_car_;
  //step1:nearest point;
  int nearest_index = get_nearest_index(car_pose, pose_array);

  PID_para_calculate(pose_array,car_pose,v_car,steer_cur );
  //step2:way point index;
  //float yaw_dis_preview = 4 + 1.7*v_car;
  float yaw_dis_preview = 1 + 7*sqrt(v_car);
  if(v_car < 3.6/3.6)
  {
      yaw_dis_preview = 8;
  }
  int yaw_index_preview = get_preview_index(pose_array, car_pose, yaw_dis_preview);
  if(yaw_index_preview == -1) {
    ROS_ERROR("no preview pt!!!");
  }
  geometry_msgs::Pose2D preview_pose = pose_array[yaw_index_preview];
  int waypoint_index = yaw_index_preview; //can change the second term ;

  //step3:Coordinate Transform;
  MatrixXf WayPoint_In_Vehicle = MatrixXf::Zero(2,1);
  MatrixXf Rotate_Matrix = g2b(M_PI/2-car_pose.theta);

  //step4:WayPoint in Vehicle;
  WayPoint_In_Vehicle(0,0) = pose_array[waypoint_index].x - car_pose.x;
  WayPoint_In_Vehicle(1,0) = pose_array[waypoint_index].y - car_pose.y;
  WayPoint_In_Vehicle = Rotate_Matrix*WayPoint_In_Vehicle;

  //step5:Calculate Curvature;
  double ld = sqrt(pow(WayPoint_In_Vehicle(0,0),2) + pow(WayPoint_In_Vehicle(1,0),2)); //distance to waypoint;
  double radius = pow(ld,2)/2.0/WayPoint_In_Vehicle(1,0); //radius;
  double kappa = 1/radius;
/********
 * add pid to decrease the lateral offset;
 ******************************/
  float kp = (- 0.01)*(v_car > 2);
  float ki = (- 0.05)*(v_car > 2);
  float kd = (- 0.01)*(v_car > 2);
  float u_s2 = min2(1,max2(-1,kp*latpp_para.lat_error))/max2(1,v_car)
          + min2(1,max2(-1,ki*latpp_para.lat_error_inter))/max2(1,v_car)
          + min2(1,max2(-1,kd*latpp_para.lat_error_rate))/max2(1,v_car);


  float steer_angle = (atan(wheelbase*kappa) + u_s2)*steering_ratio*1.1; //RAD;

  float d_lookahead = WayPoint_In_Vehicle(0,0);
  float delta_f = steer_cur/steering_ratio;
  float kappa2 = tan(delta_f)/wheelbase;
  float alpha = asin(clip(d_lookahead*kappa2,-0.999,0.999))/2;
  float y_actual = - d_lookahead * tan(alpha);
  float y_des = WayPoint_In_Vehicle(1,0);

 // ROS_INFO("y_des: %.4f, y_actual: %.4f, steer: %f, kappa: %f",y_des, y_actual, steer_angle*180/M_PI,kappa);

    ROS_INFO("lat_error:%.4f, lat_error_inter:%.4f,lat_error_deriv:%.4f,steer_cmd:%.4f,steer_cur:%.4f,kappa:%.4f",
             latpp_para.lat_error,latpp_para.lat_error_inter,latpp_para.lat_error_rate,steer_angle*180/M_PI,steer_cur*  180/M_PI,kappa);

/*  ofs << steer_angle << "\t" <<car_pose.x << "\t" << car_pose.y << "\t"
      << pose_array[nearest_index].x << "\t" << pose_array[nearest_index].y
      << "\t" << y_des << "\t" << y_actual
      << endl;
*/
   ofs<<car_pose.x<<"\t"<<car_pose.y<<"\t"
      <<latpp_para.lat_error<<"\t"<<latpp_para.lat_error_inter<<"\t"
      <<latpp_para.lat_error_rate<<"\t"<<steer_angle*180/M_PI<<"\t"
      <<steer_cur*180/M_PI<<"\t"
     <<endl;


  //  steer_angle = 0;
  return make_tuple(preview_pose,steer_angle);
}
bool PurePursuit::PID_para_calculate(vector<geometry_msgs::Pose2D> &pose_array,geometry_msgs::Pose2D car_pose,double v_car, float steer_cur ){

    int nearest_index = get_nearest_index(car_pose, pose_array);
    geometry_msgs::Pose2D nearest_poit = pose_array[nearest_index];

     //step1:calculate the current_lat_error;
     float x_vector_tmp[2];
     x_vector_tmp[0] = nearest_poit.x-car_pose.x;
     x_vector_tmp[1] = nearest_poit.y-car_pose.y;
     latpp_para.lat_error = x_vector_tmp[0]*cos(car_pose.theta) - x_vector_tmp[1]*sin(car_pose.theta);
     latpp_para.lat_error_rate = (latpp_para.lat_error - latpp_para.lat_error_last)/ts;
  //filtering ;
     float ratio_filtering = 0.1;
     latpp_para.lat_error_rate = ratio_filtering*latpp_para.lat_error_rate + (1-ratio_filtering)*latpp_para.lat_error_rate_last;
     latpp_para.lat_error_inter = min2(1,max2(-1,latpp_para.lat_error_inter_last + latpp_para.lat_error));

     latpp_para.lat_error_rate_last = latpp_para.lat_error_rate;
     latpp_para.lat_error_inter_last = latpp_para.lat_error_inter;
     latpp_para.lat_error_last = latpp_para.lat_error;

  return true;
}
