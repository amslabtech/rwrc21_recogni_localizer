#include "map_matcher/map_matcher.h"

MapMatcher::MapMatcher() : 
	private_nh_("~"),
	map_pcl_(new pcl::PointCloud<pcl::PointXYZI>),
	current_pcl_(new pcl::PointCloud<pcl::PointXYZI>),
	is_reset_(true),
	has_received_ekf_pose_(false), has_received_pc_(false), has_read_map_(false)
{
	private_nh_.param("pcd_file_path",pcd_file_path_,{"/home/amsl/pcd/ikuta/ikuta_outdoor.pcd"});
	private_nh_.param("pc_topic_name",pc_topic_name_,{"pc_in"});
	private_nh_.param("orb_pc_topic_name",orb_pc_topic_name_,{"orb_pc_in"});
	private_nh_.param("ekf_pose_topic_name",ekf_pose_topic_name_,{"ekf_pose_in"});
	private_nh_.param("ndt_pose_topic_name",ndt_pose_topic_name_,{"ndt_pose_out"});
	private_nh_.param("map_topic_name",map_topic_name_,{"map_out"});
	private_nh_.param("ndt_pc_topic_name",ndt_pc_topic_name_,{"ndt_pc_out"});
	private_nh_.param("map_frame_id",map_frame_id_,{"map"});
	private_nh_.param("is_publish_map",is_publish_map_,{false});
	private_nh_.param("is_pcl_offset",is_pcl_offset_,{false});

	private_nh_.param("VOXEL_SIZE",VOXEL_SIZE_,{0.3});
	private_nh_.param("LIMIT_RANGE",LIMIT_RANGE_,{20.0});
	private_nh_.param("TRANS_EPSILON",TRANS_EPSILON_,{0.001});
	private_nh_.param("STEP_SIZE",STEP_SIZE_,{0.1});
	private_nh_.param("RESOLUTION",RESOLUTION_,{0.5});
	private_nh_.param("MAX_ITERATION",MAX_ITERATION_,{35});
	private_nh_.param("MATCHING_SCORE_TH",MATCHING_SCORE_TH_,{0.1});
	private_nh_.param("MAP_OFFSET_X",MAP_OFFSET_X_,{0.0});
	private_nh_.param("MAP_OFFSET_Y",MAP_OFFSET_Y_,{0.0});
	private_nh_.param("MAP_OFFSET_Z",MAP_OFFSET_Z_,{0.0});
	private_nh_.param("MAP_OFFSET_ROLL",MAP_OFFSET_ROLL_,{0.0});
	private_nh_.param("MAP_OFFSET_PITCH",MAP_OFFSET_PITCH_,{0.0});
	private_nh_.param("MAP_OFFSET_YAW",MAP_OFFSET_YAW_,{0.0});

	pc_sub_ = nh_.subscribe(pc_topic_name_,10,&MapMatcher::pc_callback,this);
	//orb_pc_sub_ = nh_.subscribe(orb_pc_topic_name_,1,&MapMatcher::orb_pc_callback,this);
	ekf_pose_sub_ = nh_.subscribe(ekf_pose_topic_name_,10,&MapMatcher::ekf_pose_callback,this);
	//map_sub_ = nh_.subscribe("map",10,&MapMatcher::map_callback,this);

	ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(ndt_pose_topic_name_,20);
	map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic_name_,10);
	ndt_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ndt_pc_topic_name_,10);

	broadcaster_.reset(new tf2_ros::TransformBroadcaster);
	buffer_.reset(new tf2_ros::Buffer);
	listener_.reset(new tf2_ros::TransformListener(*buffer_));
}

void MapMatcher::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pc_time_ = msg->header.stamp;
	pc_ = *msg;
	pcl::PointCloud<pcl::PointXYZI>::Ptr raw_current_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*msg,*raw_current_pcl);

	// downsampling
	if(VOXEL_SIZE_ > 0) downsample_pcl(raw_current_pcl,current_pcl_,VOXEL_SIZE_);
	else current_pcl_ = raw_current_pcl;

	current_pcl_->is_dense = false;
	current_pcl_->width = current_pcl_->size();


	// offset
	if(is_pcl_offset_){
		geometry_msgs::TransformStamped transform_stamped;
		try{
			transform_stamped = buffer_->lookupTransform("base_link",msg->header.frame_id,ros::Time(0));
		}
		catch(tf2::TransformException& ex){
			ROS_WARN("%s", ex.what());
			return;
		}	
		Eigen::Matrix4f transform = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
		pcl::transformPointCloud(*current_pcl_,*current_pcl_,transform);
	}

	has_received_pc_ = true;
}

/*
void MapMatcher::orb_pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	orb_pc_ =*msg;
}
*/

void MapMatcher::ekf_pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	ekf_pose_ = *msg;
	has_received_ekf_pose_ = true;
}

/*
void MapMatcher::map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

}
*/

void MapMatcher::init_map() { map_pcl_->clear(); }

void MapMatcher::set_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl,double x,double y,double z)
{
	output_pcl->clear();
	
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(input_pcl);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-LIMIT_RANGE_ + x,LIMIT_RANGE_ + x);
	pass.filter(*output_pcl);
	pass.setInputCloud(output_pcl);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-LIMIT_RANGE_ + y,LIMIT_RANGE_ + y);
	pass.filter(*output_pcl);

	pass.setInputCloud(output_pcl);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-15 + z,15 + z);
	pass.filter(*output_pcl);

}

void MapMatcher::read_map()
{
	// initialize
	if(is_reset_){
		init_map();
		is_reset_ = false;
	}

	// load map
	pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	if(pcd_file_path_ == ""){
		ROS_ERROR("No map entered");
		return;
	}
	if(pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file_path_,*raw_cloud) == -1){
		ROS_ERROR("Cloud not find a map");
		return;
	}
	else std::cout << "Load: " << pcd_file_path_ << std::endl;

	// downsampling
	if(VOXEL_SIZE_ > 0) downsample_pcl(raw_cloud,map_pcl_,VOXEL_SIZE_);
	else map_pcl_ = raw_cloud;

	// offset
	Eigen::Vector3f offset_position(MAP_OFFSET_X_,MAP_OFFSET_Y_,MAP_OFFSET_Z_);
	Eigen::Quaternionf offset_orientation = msg_to_quat_eigen(rpy_to_msg(MAP_OFFSET_ROLL_,MAP_OFFSET_PITCH_,MAP_OFFSET_YAW_));
	pcl::transformPointCloud(*map_pcl_,*map_pcl_,offset_position,offset_orientation);

	// publish map
	if(is_publish_map_){
		sensor_msgs::PointCloud2 map;
		pcl::toROSMsg(*map_pcl_,map);
		//map.header.stamp = ros::Time(0);
		map.header.frame_id = map_frame_id_;
		map_pub_.publish(map);
	}

	has_read_map_ = true;
}

void MapMatcher::downsample_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr& output_pcl,double voxel_size)
{
	pcl::VoxelGrid<pcl::PointXYZI> voxel_sampler;
	voxel_sampler.setLeafSize(voxel_size,voxel_size,voxel_size);
	voxel_sampler.setInputCloud(input_pcl);
	voxel_sampler.filter(*output_pcl);
}

void MapMatcher::matching(pcl::PointCloud<pcl::PointXYZI>::Ptr map_pcl,pcl::PointCloud<pcl::PointXYZI>::Ptr local_pcl)
{
	// passthrough
	pcl::PointCloud<pcl::PointXYZI>::Ptr map_local_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr current_local_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	set_pcl(map_pcl_,map_local_pcl,ekf_pose_.pose.position.x,ekf_pose_.pose.position.y,ekf_pose_.pose.position.z);
	set_pcl(current_pcl_,current_local_pcl,0.0,0.0,0.0);

	// initialize
	//Eigen::AngleAxisf init_rotation(msg_to_quat_eigen(ekf_pose_.pose.orientation));
	Eigen::AngleAxisf init_rotation((float)get_yaw_from_quat(ekf_pose_.pose.orientation),Eigen::Vector3f::UnitZ());
	Eigen::Translation3f init_translation((float)ekf_pose_.pose.position.x,(float)ekf_pose_.pose.position.y,(float)ekf_pose_.pose.position.z);
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();

	// align
	pcl::PointCloud<pcl::PointXYZI>::Ptr ndt_pcl(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI> ndt;
	ndt.setTransformationEpsilon(TRANS_EPSILON_);
	ndt.setStepSize(STEP_SIZE_);
	ndt.setResolution(RESOLUTION_);
	ndt.setMaximumIterations(MAX_ITERATION_);
	if(map_local_pcl->points.empty() || current_local_pcl->points.empty()){
		if(map_local_pcl->points.empty()) std::cout << "map_pcl is empty" << std::endl;
		if(current_local_pcl->points.empty()) std::cout << "local_pcl is empty" << std::endl;
		return;
	}
	ndt.setInputTarget(map_local_pcl);
	ndt.setInputSource(current_local_pcl);
	if(current_local_pcl->points.size() > map_local_pcl->points.size()){
		std::cout << "local clouds > map clouds" << std::endl;
		return;
	}
	ndt.align(*ndt_pcl,init_guess);
	//ndt.align(*ndt_pcl,Eigen::Matrix4f::Identity());
	if(!ndt.hasConverged()){
		std::cout << "Has converged" << std::endl;
		return;
	}

	std::cout << "FitnessScore: " << ndt.getFitnessScore() << std::endl;
	std::cout << std::endl;

	if(ndt.getFitnessScore() <= MATCHING_SCORE_TH_){
		Eigen::Matrix4f translation = ndt.getFinalTransformation();	
		Eigen::Quaternionf quaternion(Eigen::Matrix3f(translation.block(0,0,3,3)));
		quaternion.normalize();

		// publish_ndt_pose
		geometry_msgs::PoseStamped ndt_pose;
		ndt_pose.pose.position.x = translation(0,3);
		ndt_pose.pose.position.y = translation(1,3);
		ndt_pose.pose.position.z = translation(2,3);
		//ndt_pose.pose.position.z = 0.0;
		ndt_pose.pose.orientation = quat_eigen_to_msg(quaternion);
		ndt_pose.header.stamp = ekf_pose_.header.stamp;
		ndt_pose.header.frame_id = ekf_pose_.header.frame_id;
		ndt_pose_pub_.publish(ndt_pose);

		// publish ndt_pcl
		sensor_msgs::PointCloud2 ndt_msg;
		pcl::toROSMsg(*ndt_pcl,ndt_msg);
		ndt_msg.header.stamp = pc_time_;
		ndt_msg.header.frame_id = map_frame_id_;
		ndt_pc_pub_.publish(ndt_msg);

		//debug
		double roll, pitch, yaw;
		tf2::Quaternion q;
		tf2::fromMsg(ndt_pose.pose.orientation,q);
		tf2::Matrix3x3 r(q);
		r.getRPY(roll,pitch,yaw);
		std::cout << "NDT POSE: " << std::endl;
		std::cout << " X : " << ndt_pose.pose.position.x << std::endl;
		std::cout << " Y : " << ndt_pose.pose.position.y << std::endl;
		std::cout << "YAW: " << yaw << std::endl;
		std::cout << std::endl;
	}
	else{
		std::cout << "Cannot match due to high sum of squared distance between clouds" << std::endl;
	}
}

double MapMatcher::get_yaw_from_quat(geometry_msgs::Quaternion q)
{
	double r, p, y;
	tf2::Quaternion quaternion(q.x,q.y,q.z,q.w);
	tf2::Matrix3x3(quaternion).getRPY(r,p,y);

	return y;
}

geometry_msgs::Quaternion MapMatcher::rpy_to_msg(double roll,double pitch,double yaw)
{
	geometry_msgs::Quaternion msg;
	tf2::Quaternion quaternion;
	quaternion.setRPY(roll,pitch,yaw);
	msg.x = quaternion.x();
	msg.y = quaternion.y();
	msg.z = quaternion.z();
	msg.w = quaternion.w();

	return msg;
}

geometry_msgs::Quaternion MapMatcher::quat_eigen_to_msg(Eigen::Quaternionf q)
{
	geometry_msgs::Quaternion msg;
	msg.x = (double)q.x();
	msg.y = (double)q.y();
	msg.z = (double)q.z();
	msg.w = (double)q.w();

	return msg;
}

Eigen::Quaternionf MapMatcher::msg_to_quat_eigen(geometry_msgs::Quaternion q)
{
	Eigen::Quaternionf quaternion;
	quaternion.x() = (float)q.x;
	quaternion.y() = (float)q.y;
	quaternion.z() = (float)q.z;
	quaternion.w() = (float)q.w;
	quaternion.normalize();

	return quaternion;
}

void MapMatcher::process()
{
	read_map();
	ros::Rate rate(20.0);
	while(ros::ok()){
		if(has_read_map_ && has_received_ekf_pose_ && has_received_pc_){
			matching(map_pcl_,current_pcl_);
			has_received_pc_ = false;
			has_received_ekf_pose_ = false;
		}
		else if(has_read_map_) std::cout << "Waiting msg" << std::endl;
		ros::spinOnce();
		rate.sleep();
	}
}