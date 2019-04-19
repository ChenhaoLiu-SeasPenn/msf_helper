#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

namespace msf_helper {
	class msf_helper {
	private:
		ros::Subscriber subMagneticField_;
		ros::Publisher pubVector3_;
		ros::NodeHandle n_;

	public:
		double incl_, decl_;
		
		~msf_helper() {};
		msf_helper(ros::NodeHandle n, double incl, double decl) {
			n_ = n;
			pubVector3_ = n.advertise<geometry_msgs::Vector3>("attitude_vector", 1);
			subMagneticField_ = n.subscribe<sensor_msgs::MagneticField>("magnetic_field", 1, &msf_helper::mag_convert_callback, this);

		}
		void mag_convert_callback(const sensor_msgs::MagneticFieldConstPtr& msg);
	};

	void msf_helper::mag_convert_callback(const sensor_msgs::MagneticFieldConstPtr& msg) {
		this->pubVector3_.publish(msg->magnetic_field);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "recoverImu");
	ros::NodeHandle pnh("~");

	msf_helper::msf_helper helper(pnh, 0, 0);

	ros::spin();

	return 0;
}