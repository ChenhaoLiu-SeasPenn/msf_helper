#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <imu_3dm_gx4/FilterOutput.h>

using namespace std;

void recoverCallback(const imu_3dm_gx4::FilterOutputConstPtr& fo, const sensor_msgs::ImuConstPtr& Imu, ros::Publisher& pubImu)/*ros::Publisher& pubImu*/{
	
	sensor_msgs::Imu sImu;
	
	// Info from filter output
	sImu.header = fo->header;
	sImu.orientation = fo->orientation;
	sImu.orientation_covariance = fo->orientation_covariance;
	
	// Info from raw Imu
	sImu.angular_velocity = Imu->angular_velocity;
	sImu.angular_velocity_covariance = Imu->angular_velocity_covariance;
	sImu.linear_acceleration = Imu->linear_acceleration;
	sImu.linear_acceleration_covariance = Imu->linear_acceleration_covariance;

	pubImu.publish(sImu);

}

int main(int argc, char **argv){
	ros::init(argc, argv, "recoverImu");
	ros::NodeHandle pnh("~");

	/*int requestedImuRate, requestedFilterRate;

	pnh.param<int>("imu_rate", requestedImuRate, 100);
	pnh.param<int>("filter_rate", requestedFilterRate, 100);

	if (requestedFilterRate < 0 || requestedImuRate < 0) {
		ROS_ERROR("imu_rate and filter_rate must be > 0");
		return -1;
	}

	message_filters::Subscriber<imu_3dm_gx4::FilterOutput> FiO_sub(pnh, "FilterOutput", 1);
  	message_filters::Subscriber<sensor_msgs::Imu> Imu_sub(pnh, "Imu", 1);
	ros::Publisher pubImu = pnh.advertise<sensor_msgs::Imu>("recImu", 1);

	typedef message_filters::sync_policies::ApproximateTime<imu_3dm_gx4::FilterOutput, sensor_msgs::Imu> MySyncPolicy;
*/

	message_filters::Subscriber<imu_3dm_gx4::FilterOutput> FiO_sub(pnh, "FilterOutput", 1);
  	message_filters::Subscriber<sensor_msgs::Imu> Imu_sub(pnh, "Imu", 1);
	ros::Publisher pubImu = pnh.advertise<sensor_msgs::Imu>("recImu", 1);

	typedef message_filters::sync_policies::ApproximateTime<imu_3dm_gx4::FilterOutput, sensor_msgs::Imu> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), FiO_sub, Imu_sub);

	sync.registerCallback(boost::bind(&recoverCallback, _1, _2, pubImu));

	//ros::Subscirber subIMUFilter = pnh.subscribe<imu_3dm_gx4::FilterOutput>("FilterOutput", 1, recoverCallback);

	ros::spin();

	return 0;
}
