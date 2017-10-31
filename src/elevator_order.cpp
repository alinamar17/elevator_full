#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>

#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <dynamic_reconfigure/server.h>

void addCollisionObject();
void objectRecognitionCallback(const sensor_msgs::PointCloud2ConstPtr& input);
void coordinatesAccToBase(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
void liftTheTorso(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr);
void moveArmToPreposition();
void pushTheButton();
void moveArmToDrivingPosition();
void bringTorsoDown();
void setGripperCmd(double pos, double effort);

moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_ptr;
moveit_msgs::CollisionObject* collision_object_ptr;
std::vector<moveit_msgs::CollisionObject>* collision_objects_ptr;

actionlib::SimpleActionClient<control_msgs::GripperCommandAction>* gripperClientPtr;

ros::Subscriber depth_sub;
ros::Publisher hc_pub;
ros::Publisher bc_pub;
ros::Publisher torso_pub;
ros::Subscriber torso_sub;

geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped obj_pose;
tf::TransformListener *listener_ptr;

int in_preposition=0,got_coordinates=0;

int main(int argc, char** argv)
{
	ros::init(argc,argv, "elevator_order");
	ros::NodeHandle n;  

  ros::AsyncSpinner spinner(1);
    spinner.start();
    
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface_ptr=&planning_scene_interface;
  gripperClientPtr=new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>("gripper_controller/gripper_cmd");

  depth_sub=n.subscribe("/kinect2/qhd/points",10,objectRecognitionCallback); 
  hc_pub=n.advertise<geometry_msgs::PoseStamped>("head_relative_coordinates", 10, true);
  
  bc_pub=n.advertise<geometry_msgs::PoseStamped>("base_relative_coordinates", 10, true);

  tf::TransformListener listener(ros::Duration(200.0));
  listener_ptr=&listener;
    
  message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
  point_sub_.subscribe(n, "head_relative_coordinates", 10);

  tf::MessageFilter<geometry_msgs::PoseStamped> tf_filter(point_sub_, listener, "kinect2_rgb_optical_frame", 10);
  tf_filter.registerCallback( boost::bind(&coordinatesAccToBase, _1) );
  ROS_INFO("Subscribed to tf");

  torso_pub=n.advertise<std_msgs::Float64>("/torso_controller/command",10);
  torso_sub=n.subscribe("base_relative_coordinates",10,liftTheTorso);

  /*Waiting for robot to recognize the button, get coordinates according to base_footprint frame and lift the torso up to 0.3 in purpose to get to the button 
   * Important note: if the button is higher than 1.15 meter there is no way for armadillo to get to it*/
  while(!in_preposition);
  
  /*Shut down the subscribers 'cause there is no need of them anymore*/
  depth_sub.shutdown();
  torso_sub.shutdown();

  /*close the gripper*/
  setGripperCmd(0.04,0.3);
  
  /*Moving the arm against the button
   Important: don't move the arm right to the position 'cause: 
    1. a collision with a button is not allowed yet 
    2. there is a chance robot arm will scratch the wall near the button and get harmed*/
  moveArmToPreposition();
  
  /*Add the object to the robot's world and allowing the collision of robot with the button*/
  addCollisionObject();
  
  /*...*/
  pushTheButton();  
  
  /*Go back to the position against button. Have to do this before getting arm to drive position for the same reason (2) as previos call for this function*/
  moveArmToPreposition();
  
  /*Get the arm to position safe to drive*/
  moveArmToDrivingPosition();
  
  /*open the gripper*/  
  setGripperCmd(0.14,0.0);
  
  /*get torso to 0.1 cause it isn't safe to drive the robot when it lifted a lot*/
  bringTorsoDown();

	return 0;
}

void addCollisionObject(){
      collision_object_ptr=new moveit_msgs::CollisionObject();
      collision_object_ptr->header.frame_id = target_pose.header.frame_id;
      
      /* The id of the object is used to identify it. */
      collision_object_ptr->id = "button";
      /* Define a button to add to the world. */
      shape_msgs::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.05;
      primitive.dimensions[1] = 0.3;
      primitive.dimensions[2] = 0.3;

      /* A pose for the button (specified relative to frame_id) is the pose we got from recognition */
      
      collision_object_ptr->primitives.push_back(primitive);
      collision_object_ptr->primitive_poses.push_back(target_pose);
      collision_object_ptr->operation = collision_object_ptr->ADD;

      collision_objects_ptr=new std::vector<moveit_msgs::CollisionObject>();
      collision_objects_ptr->push_back(*collision_object_ptr);

      ROS_INFO("Add an object into the world");
      planning_scene_interface_ptr->addCollisionObjects(*collision_objects_ptr);


      /*Get the AllowedCollisionMatrix of our robot*/
      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
      planning_scene::PlanningScene planning_scene(kinematic_model);

      collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

      /*allow the collision of robot with the button*/
      acm.setEntry("button",true);

      /*In order to see the AllowedColisionMatrix uncomment the next line*/
      //acm.print(std::cout);
      
      /* Sleep so we have time to see the object in RViz */
      sleep(2);
}

void objectRecognitionCallback(const sensor_msgs::PointCloud2ConstPtr& input){
    
    //pcl - point clound library with lot of algorithms
    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    //convert ros point cloud msg to pcl point cloud 
    pcl::fromROSMsg (*input, cloud); 
    //create projection image from p.c.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {
        ROS_WARN("empty cloud");
      return;
    }
    //creating new ros sensor msg - picture is relative to depth camera tf
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;
    
    
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat result=cv_ptr->image;

    cv::Mat imgHSV;
    cv::Mat img_scaled_8u;
    cv::cvtColor(result,imgHSV,cv::COLOR_BGR2HSV);
    cv::waitKey(5);
    int iLowH = 170;
    int iHighH = 179;

    int iLowS = 50;
    int iHighS = 255;

    int iLowV = 60;
    int iHighV = 255;
    cv::Mat imgThresholded;

    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        
    //morphological opening (removes small objects from the foreground)
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) ); 

    //morphological closing (removes small holes from the foreground)
    cv::dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) ); 
    cv::erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

    //Calculate the moments of the thresholded image
    cv::Moments oMoments = cv::moments(imgThresholded);

    double dM01 = oMoments.m01;
    double dM10 = oMoments.m10;
    double dArea = oMoments.m00;

    // if the area <= 50, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if (dArea > 50)
    {
      //calculate the position of the ball
      int posX = dM10 / dArea;
      int posY = dM01 / dArea;   
    cv::line(result, cv::Point(posX, posY), cv::Point(posX, posY), cv::Scalar(0,0,255), 2);

    int pcl_index = ((int)(posY)*result.cols) + (int)(posX);
    
    obj_pose.header.frame_id=input->header.frame_id;
    obj_pose.header.stamp=ros::Time::now();
    obj_pose.pose.position.x =cloudp->points[pcl_index].x;
    obj_pose.pose.position.y = cloudp->points[pcl_index].y;
    obj_pose.pose.position.z = cloudp->points[pcl_index].z;
    obj_pose.pose.orientation.w=1;

        hc_pub.publish(obj_pose);

    cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
      cv::imshow("Original", result); //show the original image

        if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            std::cout << "esc key is pressed by user" << std::endl;
        }

   }
}

void coordinatesAccToBase(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr){
    geometry_msgs::PoseStamped base_object_pose;
    tf::StampedTransform transform;
    base_object_pose.header.stamp= ros::Time(0);
    ros::Time t = ros::Time(0);
    try
    {
       bool success =  listener_ptr->waitForTransform(point_ptr->header.frame_id,"base_footprint",t, ros::Duration(3.0));
       if(success){ 
           //std::cout<<std::endl<<"Waiting is done!"<<std::endl;
           try{
            listener_ptr->transformPose("base_footprint", *point_ptr, base_object_pose);

            base_object_pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(M_PI,0.0,0.0);
            
          std::cout<< std::endl<<"Got coordinates according to base:\nX: "<<base_object_pose.pose.position.x<<" Y: "<< base_object_pose.pose.position.y << " Z: " << base_object_pose.pose.position.z <<std::endl;
            
            if(!got_coordinates&&!isnan(base_object_pose.pose.position.x)){
                target_pose=base_object_pose;
                got_coordinates=1;
            }
             bc_pub.publish(base_object_pose);
           }
           catch(tf::TransformException &ex){
            ROS_INFO("Waiting for transformation. It may take a while...");
           }
         }
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
}

void liftTheTorso(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr){
    if(!isnan(point_ptr->pose.position.z)&&point_ptr->pose.position.z>0.9){
        std_msgs::Float64 msg;
        msg.data=0.3;
        //std::cout << std::endl << "Sending message to the torso" << std::endl;
        torso_pub.publish(msg);
    }
    else{
        in_preposition=1;
    }
}

void moveArmToPreposition(){
    
    moveit::planning_interface::MoveGroup group("arm");

    ROS_INFO("Setted group");
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(500);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
    group.setGoalTolerance(0.03);
    
    group.setStartStateToCurrentState();

    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped new_target_pose;
     new_target_pose.header.frame_id="base_footprint";
     new_target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);
     new_target_pose.pose.position.x = target_pose.pose.position.x-0.15;
     new_target_pose.pose.position.y = target_pose.pose.position.y+0.04;
     new_target_pose.pose.position.z = target_pose.pose.position.z-0.03;
     new_target_pose.pose.orientation = target_pose.pose.orientation; //horizontal
    
    group.setPoseTarget(new_target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.move();
    }
    sleep(5);
}

void pushTheButton(){
    moveit::planning_interface::MoveGroup group("arm");
    
    ROS_INFO("Setted group");
    group.setMaxVelocityScalingFactor(0.1);
    group.setMaxAccelerationScalingFactor(0.5);
    group.setPlanningTime(10.0);
    group.setNumPlanningAttempts(500);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("base_footprint");
   group.setGoalTolerance(0.03);
    
    group.setStartStateToCurrentState();

    ROS_INFO("End effector reference frame: %s", group.getEndEffectorLink().c_str());
    geometry_msgs::PoseStamped new_target_pose;
     new_target_pose.header.frame_id="base_footprint";
     new_target_pose.header.stamp=ros::Time::now()+ros::Duration(10.0);
     new_target_pose.pose.position.x = target_pose.pose.position.x-0.1;
     new_target_pose.pose.position.y = target_pose.pose.position.y+0.04;
     new_target_pose.pose.position.z = target_pose.pose.position.z-0.03;
     new_target_pose.pose.orientation = target_pose.pose.orientation; //horizontal
    
    group.setPoseTarget(new_target_pose);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.move();
    }
    sleep(4);
}

void bringTorsoDown(){
  for(int i=0;i<1000;i++){
      std_msgs::Float64 msg;
      msg.data=0.1;
      //std::cout << std::endl << "Sending message to the torso" << std::endl;
      torso_pub.publish(msg);
  }
  sleep(5);
}

void moveArmToDrivingPosition(){
   moveit::planning_interface::MoveGroup group("arm");
   group.setNamedTarget("driving");
   moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("plan: %s",success?"SUCCESS":"FAILED");
    if(success) {
        ROS_INFO("Moving...");
        group.move();
    }
    sleep(4);
}

void setGripperCmd(double pos, double effort = 0){
  control_msgs::GripperCommandGoal goal;
        goal.command.position = pos;
        goal.command.max_effort = effort;

        gripperClientPtr->sendGoal(goal);
        gripperClientPtr->waitForResult();
}
