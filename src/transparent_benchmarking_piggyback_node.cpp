#include "ros/ros.h"

#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

//#include "tf/transform_listener.h"
//#include <tf/transform_broadcaster.h>

#include <vector>
#include <map>
#include <string>
#include <sstream>

using namespace std;


// metadata topics names
#define BENCHMARK_NAME_TOPIC_NAME						"/transparent_benchmarking/benchmark"
#define TEAM_NAME_TOPIC_NAME							"/transparent_benchmarking/team_name"
#define OVERALL_MESSAGES_COUNT_TOPIC_NAME				"/transparent_benchmarking/overall_messages_count"
#define MESSAGES_COUNT_TOPIC_NAME						"/transparent_benchmarking/messages_count"

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

// other stuff
#define CLEAR_CONSOLE false
#define LOOP_RATE 1



// Global variables
//tf::TransformListener* tf_listener_;
//tf::TransformBroadcaster* tf_broadcaster_;
std_msgs::Int64 overallMessagesCount;
ros::Publisher overallMessagesCountPublisher;
map<string, long int> messagesCount;
ros::Publisher messagesCountPublisher;

void publishMessagesCounters(){
	// publish the overall messages count
	overallMessagesCountPublisher.publish(overallMessagesCount);
	
	// make string with per-topic message count and publish it
	std_msgs::String s;
	stringstream ss;
	for(auto t_c : messagesCount) ss << t_c.first << ":\t" << t_c.second << endl;
	s.data = ss.str();
	messagesCountPublisher.publish(s);
}

template <class T>
class TopicTranslator {
	TopicTranslator() {}
	
	public:
	
	string sourceTopicName, standardTopicName;
	long int sourceMessagesCount = 0;
	ros::Subscriber sub;
	ros::Publisher pub;
	
	void callback(const typename T::ConstPtr& msg) {
		messagesCount[standardTopicName]++;
		sourceMessagesCount++;
		overallMessagesCount.data++;
		
		// publish the copy of the message with the standard name
		T outputMsg;
		outputMsg = *msg;
		pub.publish(outputMsg);
		
		publishMessagesCounters();
	}
	
	TopicTranslator<T>(string sourceTopicName_, string standardTopicName_) : sourceTopicName(sourceTopicName_), standardTopicName(standardTopicName_) {
		ROS_INFO("TopicTranslator from: [%s] to: [%s]", sourceTopicName.c_str(), standardTopicName.c_str());
		
		messagesCount[standardTopicName] = 0;
	}
	
};



struct TopicNames {
	string
		teamName,
		benchmarkName,
		mapTopic,
		mapMetadataTopic,
		robotPoseTopic,
		robotPoseWithCovarianceTopic,
		trajectoryTopic
		;
	
	vector<string>
		laserScanTopics,
		cameraImageTopics,
		cameraInfoTopics,
		depthSensorTopics,
		audioTopics,
		additionalTopics
		;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "TB_piggyback");
	ros::NodeHandle n, np("~");
	ROS_INFO("TB_piggyback started");
	
	TopicNames topicNames;
	np.getParam("/team_name", topicNames.teamName);
	np.getParam("/benchmark_name", topicNames.benchmarkName);
	np.getParam("/map_topic", topicNames.mapTopic);
	np.getParam("/map_metadata_topic", topicNames.mapMetadataTopic);
	np.getParam("/robot_pose_topic", topicNames.robotPoseTopic);
	np.getParam("/robot_pose_with_covariance_topic", topicNames.robotPoseWithCovarianceTopic);
	np.getParam("/trajectory_topic", topicNames.trajectoryTopic);
	np.getParam("/laser_scan_topics", topicNames.laserScanTopics);
	np.getParam("/camera_image_topics", topicNames.cameraImageTopics);
	np.getParam("/camera_info_topics", topicNames.cameraInfoTopics);
	np.getParam("/depth_sensor_topics", topicNames.depthSensorTopics);
	np.getParam("/audio_topics", topicNames.audioTopics);
	np.getParam("/additional_topics", topicNames.additionalTopics);
	
	// Instantiate map subscriber translator
	if(topicNames.mapTopic.size()) {
		TopicTranslator<nav_msgs::OccupancyGrid>* s = new TopicTranslator<nav_msgs::OccupancyGrid>(topicNames.mapTopic, STANDARD_MAP_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.mapTopic, 1000, &TopicTranslator<nav_msgs::OccupancyGrid>::callback, s);
		s->pub = n.advertise<nav_msgs::OccupancyGrid>(s->standardTopicName, 10);
	}
	
	// Instantiate map metadata subscriber translator
	if(topicNames.mapMetadataTopic.size()) {
		TopicTranslator<nav_msgs::MapMetaData>* s = new TopicTranslator<nav_msgs::MapMetaData>(topicNames.mapMetadataTopic, STANDARD_MAP_METADATA_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.mapMetadataTopic, 1000, &TopicTranslator<nav_msgs::MapMetaData>::callback, s);
		s->pub = n.advertise<nav_msgs::MapMetaData>(s->standardTopicName, 10);
	}
	
	// Instantiate robot pose subscriber translator
	if(topicNames.robotPoseTopic.size()) {
		TopicTranslator<geometry_msgs::PoseStamped>* s = new TopicTranslator<geometry_msgs::PoseStamped>(topicNames.robotPoseTopic, STANDARD_ROBOT_POSE_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.robotPoseTopic, 1000, &TopicTranslator<geometry_msgs::PoseStamped>::callback, s);
		s->pub = n.advertise<geometry_msgs::PoseStamped>(s->standardTopicName, 10);
	}
	
	// Instantiate robot pose with covariance subscriber translator
	if(topicNames.robotPoseWithCovarianceTopic.size()) {
		TopicTranslator<geometry_msgs::PoseWithCovarianceStamped>* s = new TopicTranslator<geometry_msgs::PoseWithCovarianceStamped>(topicNames.robotPoseWithCovarianceTopic, STANDARD_ROBOT_POSE_WITH_COVARIANCE_TOPIC_NAME);
		s->sub = n.subscribe(topicNames.robotPoseWithCovarianceTopic, 1000, &TopicTranslator<geometry_msgs::PoseWithCovarianceStamped>::callback, s);
		s->pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(s->standardTopicName, 10);
	}
	
	// Instantiate trajectory subscriber translator
	TopicTranslator<nav_msgs::Path>* trajectorySubTranslator = (topicNames.robotPoseTopic.size() ? (new TopicTranslator<nav_msgs::Path>(topicNames.trajectoryTopic, STANDARD_TRAJECTORY_TOPIC_NAME)) : NULL);
	
	// Instantiate laser scan subscriber translators
	for (int i = 0; i < topicNames.laserScanTopics.size(); i++) {
		stringstream standardTopicName;
		standardTopicName << STANDARD_LASER_SCAN_TOPICS_NAME << i;
		
		TopicTranslator<sensor_msgs::LaserScan>* s = new TopicTranslator<sensor_msgs::LaserScan>(topicNames.laserScanTopics[i], standardTopicName.str());
		s->sub = n.subscribe(topicNames.laserScanTopics[i], 1000, &TopicTranslator<sensor_msgs::LaserScan>::callback, s);
		s->pub = n.advertise<sensor_msgs::LaserScan>(s->standardTopicName, 10);
	}
	
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
	
	// Instantiate depth sensor subscriber translators
	for (int i = 0; i < topicNames.depthSensorTopics.size(); i++) {
		stringstream standardTopicName;
		standardTopicName << STANDARD_DEPTH_SENSOR_TOPICS_NAME << i;
		
		TopicTranslator<sensor_msgs::PointCloud2>* s = new TopicTranslator<sensor_msgs::PointCloud2>(topicNames.depthSensorTopics[i], standardTopicName.str());
		s->sub = n.subscribe(topicNames.depthSensorTopics[i], 1000, &TopicTranslator<sensor_msgs::PointCloud2>::callback, s);
		s->pub = n.advertise<sensor_msgs::PointCloud2>(s->standardTopicName, 10);
	}
	
	overallMessagesCountPublisher = n.advertise<std_msgs::Int64>(OVERALL_MESSAGES_COUNT_TOPIC_NAME, 10);
	messagesCountPublisher = n.advertise<std_msgs::String>(MESSAGES_COUNT_TOPIC_NAME, 10);
	
	ros::Publisher teamNamePublisher = n.advertise<std_msgs::String>(TEAM_NAME_TOPIC_NAME, 10, true);
	ros::Publisher benchmarkNamePublisher = n.advertise<std_msgs::String>(BENCHMARK_NAME_TOPIC_NAME, 10, true);
	
	std_msgs::String benchmarkNameMessage;
	benchmarkNameMessage.data = topicNames.benchmarkName;
	benchmarkNamePublisher.publish(benchmarkNameMessage);
	
	std_msgs::String teamNameMessage;
	teamNameMessage.data = topicNames.teamName;
	teamNamePublisher.publish(teamNameMessage);
	
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
  *		· publish markerset pose ?
  *		· publish and record benchmark metadata (team_name, benchmark, configuration, etc)
  *		· use actual rulebook topic names
  *	
  */


