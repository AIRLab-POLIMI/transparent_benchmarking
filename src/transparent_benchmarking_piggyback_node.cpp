#include "ros/ros.h"

#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>
#include <sstream>



using namespace std;

// standard topic names
#define STANDARD_MAP_TOPIC_NAME							"/ERL/map"
#define STANDARD_MAP_METADATA_TOPIC_NAME				"/ERL/map_metadata"
#define STANDARD_ROBOT_POSE_TOPIC_NAME					"/ERL/robot_pose"
#define STANDARD_ROBOT_POSE_WITH_COVARIANCE_TOPIC_NAME	"/ERL/robot_pose_with_covariance"
#define STANDARD_MARKERSET_POSE_TOPIC_NAME				"/ERL/markerset_pose"
#define STANDARD_TRAJECTORY_TOPIC_NAME					"/ERL/trajectory"
#define STANDARD_LASER_SCAN_TOPICS_NAME					"/ERL/laser_scan_"
#define STANDARD_CAMERA_IMAGE_TOPICS_NAME				"/ERL/image_"
#define STANDARD_CAMERA_INFO_TOPICS_NAME				"/ERL/camera_info_"
#define STANDARD_DEPTH_SENSOR_TOPICS_NAME				"/ERL/depth_sensor_pointcloud_"
#define STANDARD_AUDIO_TOPICS_NAME						"/ERL/audio_"

// standard farme names
#define WORLD_FRAME_NAME								"/world"
#define STANDARD_MAP_FRAME_NAME							"/ERL/map_frame"
#define STANDARD_BASE_FRAME_NAME						"/ERL/base_frame"
#define STANDARD_SCAN_FRAMES_NAME						"/ERL/laser_scan_frame_"
#define STANDARD_CAMERA_FRAMES_NAME						"/ERL/camera_frame_"
#define STANDARD_DEPTH_SENSOR_FRAMES_NAME				"/ERL/depth_sensor_frame_"

#define CLEAR_CONSOLE false
#define LOOP_RATE 1


// Global variables
tf::TransformListener* tf_listener_;
tf::TransformBroadcaster* tf_broadcaster_;


template <class T>
class TopicTranslator {
	TopicTranslator(){}
	
	public:
	
	string sourceTopicName, standardTopicName;
	long int sourceMessagesCount = 0;
	ros::Subscriber sub;
	ros::Publisher pub;
	
	void callback(const typename T::ConstPtr& msg) {
		sourceMessagesCount++;
		
		T outputMsg;
		outputMsg = *msg;
		pub.publish(outputMsg);
	}
	
	TopicTranslator<T>(string sourceTopicName_, string standardTopicName_) : sourceTopicName(sourceTopicName_), standardTopicName(standardTopicName_){
		ROS_INFO("TopicTranslator: from: [%s] to: [%s]", sourceTopicName.c_str(), standardTopicName.c_str());
	}
	
};


class FrameTranslator {
	FrameTranslator(){}
	
	public:
	
	ros::Timer tfTimer;
	
	string sourceOriginFrameName, sourceDestinationFrameName, standardOriginFrameName, standardDestinationFrameName;
	long int sourceFramesCount = 0;
	
	void tfTimerCallback(const ros::TimerEvent& msg){
		
		tf::StampedTransform transform;
		try {
			
			// TODO check transform is different? (timestamp)
			tf_listener_->lookupTransform(sourceOriginFrameName, sourceDestinationFrameName, ros::Time(0), transform);
			sourceFramesCount++;
			
			tf_broadcaster_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), standardOriginFrameName, standardDestinationFrameName));
			
			
		} catch (tf::TransformException ex) {
			ROS_INFO("FrameTranslator::tfTimerCallback : tf::TransformException: %s",ex.what());
		}
	}
	
	FrameTranslator(ros::NodeHandle n_, string sourceOriginFrameName_, string sourceDestinationFrameName_, string standardOriginFrameName_, string standardDestinationFrameName_)
			: sourceOriginFrameName(sourceOriginFrameName_), sourceDestinationFrameName(sourceDestinationFrameName_),
			  standardOriginFrameName(standardOriginFrameName_), standardDestinationFrameName(standardDestinationFrameName_){
		
		if(sourceOriginFrameName.size() == 0 || sourceDestinationFrameName.size() == 0) return;
		
		ROS_INFO("FrameTranslator: from: [%s -> %s] to: [%s -> %s]", sourceOriginFrameName.c_str(), sourceDestinationFrameName.c_str(), standardOriginFrameName.c_str(), standardDestinationFrameName.c_str());
		
		tfTimer = n_.createTimer(ros::Duration(0.01), &FrameTranslator::tfTimerCallback, this);
	}
	
};

struct TopicNames {
	string
		teamName,
		mapTopic,
		mapMetadataTopic,
		robotPoseTopic,
		robotPoseWithCovarianceTopic,
		trajectoryTopic,
		mapFrame,
		baseFrame
		;
	
	vector<string>
		laserScanTopics,
		laserScanFrames,
		cameraImageTopics,
		cameraInfoTopics,
		cameraFrames,
		depthSensorTopics,
		depthSensorFrames,
		audioTopics,
		additionalTopics
		;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "TB_piggyback");
	ros::NodeHandle n, np("~");
	ROS_INFO("TB_piggyback started");
	
	// Listeners and subscribers
//	tf_listener_ = new tf::TransformListener();
//	tf_broadcaster_ = new tf::TransformBroadcaster();
	
	TopicNames topicNames;
	np.getParam("/team_name", topicNames.teamName);
	np.getParam("/map_topic", topicNames.mapTopic);
	np.getParam("/map_metadata_topic", topicNames.mapMetadataTopic);
	np.getParam("/robot_pose_topic", topicNames.robotPoseTopic);
	np.getParam("/robot_pose_with_covariance_topic", topicNames.robotPoseWithCovarianceTopic);
	np.getParam("/trajectory_topic", topicNames.trajectoryTopic);
//	np.getParam("/map_frame", topicNames.mapFrame);
//	np.getParam("/base_frame", topicNames.baseFrame);
	np.getParam("/laser_scan_topics", topicNames.laserScanTopics);
//	np.getParam("/laser_scan_frames", topicNames.laserScanFrames);
	np.getParam("/camera_image_topics", topicNames.cameraImageTopics);
	np.getParam("/camera_info_topics", topicNames.cameraInfoTopics);
//	np.getParam("/camera_frames", topicNames.cameraFrames);
	np.getParam("/depth_sensor_topics", topicNames.depthSensorTopics);
//	np.getParam("/depth_sensor_frames", topicNames.depthSensorFrames);
	np.getParam("/audio_topics", topicNames.audioTopics);
	np.getParam("/additional_topics", topicNames.additionalTopics);
	
	// Instantiate map subscriber translator
	if(topicNames.mapTopic.size()){
		TopicTranslator<nav_msgs::OccupancyGrid>* s = new TopicTranslator<nav_msgs::OccupancyGrid>(topicNames.mapTopic, STANDARD_MAP_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.mapTopic, 1000, &TopicTranslator<nav_msgs::OccupancyGrid>::callback, s);
		s->pub = n.advertise<nav_msgs::OccupancyGrid>(s->standardTopicName, 10);
	}
	
	// Instantiate map metadata subscriber translator
	if(topicNames.mapMetadataTopic.size()){
		TopicTranslator<nav_msgs::MapMetaData>* s = new TopicTranslator<nav_msgs::MapMetaData>(topicNames.mapMetadataTopic, STANDARD_MAP_METADATA_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.mapMetadataTopic, 1000, &TopicTranslator<nav_msgs::MapMetaData>::callback, s);
		s->pub = n.advertise<nav_msgs::MapMetaData>(s->standardTopicName, 10);
	}
	
	// Instantiate robot pose subscriber translator
	if(topicNames.robotPoseTopic.size()){
		TopicTranslator<geometry_msgs::PoseStamped>* s = new TopicTranslator<geometry_msgs::PoseStamped>(topicNames.robotPoseTopic, STANDARD_ROBOT_POSE_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.robotPoseTopic, 1000, &TopicTranslator<geometry_msgs::PoseStamped>::callback, s);
		s->pub = n.advertise<geometry_msgs::PoseStamped>(s->standardTopicName, 10);
	}
	
	// Instantiate robot pose with covariance subscriber translator
	if(topicNames.robotPoseWithCovarianceTopic.size()){
		TopicTranslator<geometry_msgs::PoseWithCovarianceStamped>* s = new TopicTranslator<geometry_msgs::PoseWithCovarianceStamped>(topicNames.robotPoseWithCovarianceTopic, STANDARD_ROBOT_POSE_WITH_COVARIANCE_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.robotPoseWithCovarianceTopic, 1000, &TopicTranslator<geometry_msgs::PoseWithCovarianceStamped>::callback, s);
		s->pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(s->standardTopicName, 10);
	}
	
	
	// Instantiate trajectory subscriber translator
	TopicTranslator<nav_msgs::Path>* trajectorySubTranslator = (topicNames.robotPoseTopic.size() ? (new TopicTranslator<nav_msgs::Path>(topicNames.trajectoryTopic, STANDARD_TRAJECTORY_TOPIC_NAME)) : NULL);
	
	// Instantiate map and base frame translators
//	FrameTranslator* mapFrameTranslator = new FrameTranslator(n, WORLD_FRAME_NAME, topicNames.mapFrame, WORLD_FRAME_NAME, STANDARD_MAP_FRAME_NAME);
//	FrameTranslator* baseFrameTranslator = new FrameTranslator(n, topicNames.mapFrame, topicNames.baseFrame, STANDARD_MAP_FRAME_NAME, STANDARD_BASE_FRAME_NAME);
	
	// Instantiate laser scan subscriber translators
	for (int i = 0; i < topicNames.laserScanTopics.size(); i++) {
		stringstream standardTopicName;
		standardTopicName << STANDARD_LASER_SCAN_TOPICS_NAME << i;
		
		TopicTranslator<sensor_msgs::LaserScan>* s = new TopicTranslator<sensor_msgs::LaserScan>(topicNames.laserScanTopics[i], standardTopicName.str());
		s->sub = n.subscribe(topicNames.laserScanTopics[i], 1000, &TopicTranslator<sensor_msgs::LaserScan>::callback, s);
		s->pub = n.advertise<sensor_msgs::LaserScan>(s->standardTopicName, 10);
	}
	
	// Instantiate laser scan frame translators
//	for (int i = 0; i < topicNames.laserScanFrames.size(); i++) {
//		stringstream standardDestinationFrameName;
//		standardDestinationFrameName << STANDARD_SCAN_FRAMES_NAME << i;
//		new FrameTranslator(n, topicNames.baseFrame, topicNames.laserScanFrames[i], STANDARD_BASE_FRAME_NAME, standardDestinationFrameName.str());
//	}
	
	// Instantiate camera image subscriber translators
	for (int i = 0; i < topicNames.cameraImageTopics.size(); i++) {
		stringstream standardTopicName;
		standardTopicName << STANDARD_CAMERA_IMAGE_TOPICS_NAME << i;
		
		TopicTranslator<sensor_msgs::Image>* s = new TopicTranslator<sensor_msgs::Image>(topicNames.cameraImageTopics[i], standardTopicName.str());
		s->sub = n.subscribe(topicNames.cameraImageTopics[i], 1000, &TopicTranslator<sensor_msgs::Image>::callback, s);
		s->pub = n.advertise<sensor_msgs::Image>(s->standardTopicName, 10);
	}
	
	// Instantiate camera info subscriber translators
	for (int i = 0; i < topicNames.cameraInfoTopics.size(); i++) {
		stringstream standardTopicName;
		standardTopicName << STANDARD_CAMERA_INFO_TOPICS_NAME << i;
		
		TopicTranslator<sensor_msgs::CameraInfo>* s = new TopicTranslator<sensor_msgs::CameraInfo>(topicNames.cameraInfoTopics[i], standardTopicName.str());
		s->sub = n.subscribe(topicNames.cameraInfoTopics[i], 1000, &TopicTranslator<sensor_msgs::CameraInfo>::callback, s);
		s->pub = n.advertise<sensor_msgs::CameraInfo>(s->standardTopicName, 10);
	}
	
	// Instantiate camera frame translators
//	for (int i = 0; i < topicNames.cameraFrames.size(); i++) {
//		stringstream standardDestinationFrameName;
//		standardDestinationFrameName << STANDARD_CAMERA_FRAMES_NAME << i;
//		new FrameTranslator(n, topicNames.baseFrame, topicNames.cameraFrames[i], STANDARD_BASE_FRAME_NAME, standardDestinationFrameName.str());
//	}
	
	// Instantiate depth sensor subscriber translators
	for (int i = 0; i < topicNames.depthSensorTopics.size(); i++) {
		stringstream standardTopicName;
		standardTopicName << STANDARD_DEPTH_SENSOR_TOPICS_NAME << i;
		
		TopicTranslator<sensor_msgs::PointCloud2>* s = new TopicTranslator<sensor_msgs::PointCloud2>(topicNames.depthSensorTopics[i], standardTopicName.str());
		s->sub = n.subscribe(topicNames.depthSensorTopics[i], 1000, &TopicTranslator<sensor_msgs::PointCloud2>::callback, s);
		s->pub = n.advertise<sensor_msgs::PointCloud2>(s->standardTopicName, 10);
	}
	
	// Instantiate depth sensor frame translators
//	for (int i = 0; i < topicNames.depthSensorFrames.size(); i++) {
//		stringstream standardDestinationFrameName;
//		standardDestinationFrameName << STANDARD_DEPTH_SENSOR_FRAMES_NAME << i;
//		new FrameTranslator(n, topicNames.baseFrame, topicNames.depthSensorFrames[i], STANDARD_BASE_FRAME_NAME, standardDestinationFrameName.str());
//	}
	
	
	ros::spin();
	
	return 0;
}


/**	TODO:
  *		· republish audio
  *		· test poses
  *		· test trajectory
  *		· test images (and depthmaps?)
  *		· test pointclouds
  *		· test audio
  *		· publish markerset pose and frame
  *		· publish and record benchmark metadata (team_name, benchmark, configuration, etc)
  *		· use actual rulebook topic names
  *	
  */


