#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace msf_helper {
	class msf_helper {
	private:
		message_filters::Subscriber subMagneticField_, subPose_;
		// ros::Publisher pubVector3_;
		ros::Publisher pubArrow_;
		ros::NodeHandle n_;

	public:
		double incl_, decl_;
		
		~msf_helper() {};
		msf_helper(ros::NodeHandle n, double incl, double decl) {
			n_ = n;
			// pubVector3_ = n.advertise<geometry_msgs::Vector3>("attitude_vector", 1);
			pubArrow_ = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			subMagneticField_ = message_filters::Subscriber<sensor_msgs::MagneticField>(n, "magnetic_field", 1);
			subPose_ = message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(n, "pose", 1);

			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::MagneticField, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
			message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subMagneticField_, subPose_);

			sync.registerCallback(boost::bind(&mag_convert_callback, _1, _2, pubImu));
		}
		void mag_convert_callback(const sensor_msgs::MagneticFieldConstPtr& msg,\
			const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg2);
	};

	void msf_helper::mag_convert_callback(const sensor_msgs::MagneticFieldConstPtr& msg,\
		const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg2) {
		Eigen::Matrix<double, 3, 1> m_;
		geometry_msgs::Point points[2];
		m_ = msg->magnetic_field;
		m_ = m_ / m_.norm();
		visualization_msgs::Marker marker;
		
		points[0].x = msg2->pose.pose.position.x;
		points[0].y = msg2->pose.pose.position.y;
		points[0].z = msg2->pose.pose.position.z;
		points[1].x = msg2->pose.pose.position.x + m_(0, 0);
		points[1].y = msg2->pose.pose.position.y + m_(1, 0);
		points[1].z = msg2->pose.pose.position.z + m_(2, 0);

		marker.header = msg2->header;
		marker.lifetime = 0;
		marker.type = 0;
		marker.points = points;

		this->pubArrow_.publish(marker);
		//this->pubVector3_.publish(msg->magnetic_field);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "msf_helper");
	ros::NodeHandle pnh("~");

	msf_helper::msf_helper helper(pnh, 0, 0);

	ros::spin();

	return 0;
}