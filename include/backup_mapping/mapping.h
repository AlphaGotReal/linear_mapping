#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/buffer.h>

namespace utils {

	union ieee {
		float val;
		unsigned char bytes[4];
	};

	class probs {
	public:
		probs();
		probs(uint64_t, float, float);
		int32_t get_occupancy(uint64_t);
		void declare_obstacle(uint64_t);
		void declare_clear(uint64_t);

		std::vector<float> is_obstacle; 
		float positive_del_confidence;
		float negative_del_confidence;
	};

	class probability_grid {
	public:
		probability_grid();
		probability_grid(const nav_msgs::OccupancyGrid&, float, float);
		probs occupancy_params;
		nav_msgs::OccupancyGrid map;
	};

	template <uint8_t>
	void update_map(probability_grid&, 
			const sensor_msgs::PointCloud2&, 
			const sensor_msgs::Image&, 
			const tf2_ros::Buffer&,
			const std::string&,
			const std::string&,
			float);

}

namespace mapping {

	class Mapping {
	public:
		Mapping(int, char **);
	private:
		
		std::string odom_topic;
		std::string lane_mask_topic;
		std::string zed_points_topic;
		std::string map_topic;

		std::string odom_frame;
		std::string zed_frame;

		float map_resolution;
		int32_t map_height;
		int32_t map_width;
		std::vector<float> map_origin;
		float threshold;

		ros::Subscriber lane_mask_sub;
		ros::Subscriber zed_points_sub;

		bool got_lane_mask;
		bool got_zed_points;

		sensor_msgs::PointCloud2 zed_points;
		sensor_msgs::Image lane_mask;
		utils::probability_grid map;

		tf2_ros::Buffer buffer;

		ros::Publisher map_pub; 

	public:

		void run();

	private:

		void lane_mask_callback(const sensor_msgs::Image::ConstPtr&);
		void zed_points_callback(const sensor_msgs::PointCloud2::ConstPtr&);

	};
}



