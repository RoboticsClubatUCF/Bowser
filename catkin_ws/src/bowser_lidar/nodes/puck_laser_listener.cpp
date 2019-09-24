#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <iostream>

using namespace std;

void onScan(const sensor_msgs::LaserScan laser){
	int i=0;
	float onePoint = laser.ranges[i];
	float dangerZone = 2.0;
	if(onePoint >= laser.range_min && onePoint <= laser.range_max)
		if(onePoint < dangerZone)
			printf("Stay away plz.\n");
		else
			printf("\n");
}

int main(int argc, char** argv){
	ros::init(argc, argv, "laser_listener");
	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("scan", 1000, &onScan);

	while(node.ok()){	
		ros::spin();
	}

	return 0;
}