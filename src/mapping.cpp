#include "backup_mapping/mapping.h"

#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mapping {

	Mapping::Mapping(int argc, char **argv) {

		ros::init(argc, argv, "mapping");	
		ros::NodeHandle node;

		ros::param::get("/topic/odom", this->odom_topic);
		ros::param::get("/topic/image/lane_mask", this->lane_mask_topic);
		ros::param::get("/topic/pointcloud2/zed_points", this->zed_points_topic);
		ros::param::get("/topic/map/map", this->map_topic);

		ros::param::get("/frame/odom_frame", this->odom_frame);
		ros::param::get("/frame/zed_frame", this->zed_frame);

		this->map_origin.resize(3);

		ros::param::get("/map_params/resolution", this->map_resolution);
		ros::param::get("/map_params/height", this->map_height);
		ros::param::get("/map_params/width", this->map_width);
		ros::param::get("/map_params/origin/x", this->map_origin[0]);
		ros::param::get("/map_params/origin/y", this->map_origin[1]);
		ros::param::get("/map_params/origin/z", this->map_origin[2]);
		ros::param::get("/map_params/threshold", this->threshold);

		nav_msgs::OccupancyGrid temp_map;

		temp_map.header.frame_id = this->odom_frame;
		temp_map.info.resolution = this->map_resolution;
		temp_map.info.height = this->map_height;
		temp_map.info.width = this->map_width;
		temp_map.info.origin.position.x = this->map_origin[0];
		temp_map.info.origin.position.y = this->map_origin[1];
		temp_map.info.origin.position.z = this->map_origin[2];

		for (size_t t = 0; t < this->map_height * this->map_width; 
				++t, temp_map.data.push_back(-1));

		float positive_del_confidence, negative_del_confidence;
		ros::param::get("/map_params/del_confidence/positive", positive_del_confidence);
		ros::param::get("/map_params/del_confidence/negative", negative_del_confidence);

		this->map = utils::probability_grid(temp_map, positive_del_confidence, negative_del_confidence);
	
		this->lane_mask_sub = node.subscribe(this->lane_mask_topic, 1, 
				&Mapping::lane_mask_callback, this);
		this->zed_points_sub = node.subscribe(this->zed_points_topic, 1, 
				&Mapping::zed_points_callback, this);

		this->got_lane_mask = false;
		this->got_zed_points = false;

		this->map_pub = node.advertise<nav_msgs::OccupancyGrid>(this->map_topic, 1, true);
	}

	void Mapping::run() {

		ros::AsyncSpinner spinner(4);
		spinner.start();

		ros::Rate rate(60);

		tf2_ros::TransformListener listerner(this->buffer);

		while (ros::ok()) {
			if (!(this->got_lane_mask && this->got_zed_points)) {
				std::cerr << "[MASK|POINTCLOUD] :" << this->got_lane_mask << this->got_zed_points << std::endl;
				continue;
			}

			this->map_pub.publish(this->map.map);

			utils::update_map<1>(this->map, this->zed_points, 
					this->lane_mask, this->buffer, 
					this->odom_frame, this->zed_frame, 0.2);

			rate.sleep();
		}

	}

	void Mapping::lane_mask_callback(const sensor_msgs::Image::ConstPtr& mask_ptr) {
		this->lane_mask = *mask_ptr;
		this->got_lane_mask = true;
	}

	void Mapping::zed_points_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_ptr) {
		this->zed_points = *pc_ptr;
		this->got_zed_points = true;
	}

}

namespace utils {

	probs::probs() {}
	
	probs::probs(uint64_t size, float positive_del_confidence, float negative_del_confidence) {
		this->is_obstacle.resize(size);
		this->positive_del_confidence = positive_del_confidence;
		this->negative_del_confidence = negative_del_confidence;
	}

	int32_t probs::get_occupancy(uint64_t index) {
		return this->is_obstacle[index] * 100;
	}

	void probs::declare_obstacle(uint64_t index) {
		this->is_obstacle[index] += this->positive_del_confidence;
		if (this->is_obstacle[index] > 1.0) {
			this->is_obstacle[index] = 1.0;
		}
	}

	void probs::declare_clear(uint64_t index) {
		this->is_obstacle[index] -= this->negative_del_confidence;
		if (this->is_obstacle[index] < 0.0) {
			this->is_obstacle[index] = 0.0;
		}
	}

	probability_grid::probability_grid() {}
	probability_grid::probability_grid(const nav_msgs::OccupancyGrid& map,
			float positive_del_confidence,
			float negative_del_confidence) {
		this->map = map;
		this->occupancy_params = probs(map.info.height * map.info.width, 
				positive_del_confidence,
				negative_del_confidence);
	}
}

std::pair<uint64_t, uint64_t> world_to_map(geometry_msgs::Point point,
		nav_msgs::MapMetaData info) {

		uint64_t x_coor = (point.x - info.origin.position.x) / info.resolution;
		uint64_t y_coor = (point.y - info.origin.position.y) / info.resolution;

		return std::pair<uint64_t, uint64_t>(x_coor, y_coor);
}

template <uint8_t step>
void utils::update_map(utils::probability_grid& map,
	const sensor_msgs::PointCloud2& zed_points, 
	const sensor_msgs::Image& lane_mask,
	const tf2_ros::Buffer& buffer,
	const std::string& parent_frame,
	const std::string& child_frame,
	float max_height) {

	geometry_msgs::TransformStamped zed_to_odom;

	try {
		zed_to_odom = buffer.lookupTransform(
			parent_frame, child_frame,
			zed_points.header.stamp,
			ros::Duration(0.4)
		);
	} catch (tf2::LookupException& e) {
		std::cerr << e.what() << std::endl;
		return ;
	} catch (tf2::ConnectivityException& e) {
		std::cerr << e.what() << std::endl;
		return ;
	} catch (tf2::ExtrapolationException& e) {
		std::cerr << e.what() << std::endl;
		return ;
	}

	union utils::ieee x_bytes, y_bytes, z_bytes;
	geometry_msgs::Point point;


	for (int32_t t = 0; t < zed_points.height * zed_points.width; t += step) {

		x_bytes.bytes[0] = zed_points.data[t * zed_points.point_step + 0];
		x_bytes.bytes[1] = zed_points.data[t * zed_points.point_step + 1];
		x_bytes.bytes[2] = zed_points.data[t * zed_points.point_step + 2];
		x_bytes.bytes[3] = zed_points.data[t * zed_points.point_step + 3];

		y_bytes.bytes[0] = zed_points.data[t * zed_points.point_step + 4];
		y_bytes.bytes[1] = zed_points.data[t * zed_points.point_step + 5];
		y_bytes.bytes[2] = zed_points.data[t * zed_points.point_step + 6];
		y_bytes.bytes[3] = zed_points.data[t * zed_points.point_step + 7];

		z_bytes.bytes[0] = zed_points.data[t * zed_points.point_step + 8];
		z_bytes.bytes[1] = zed_points.data[t * zed_points.point_step + 9];
		z_bytes.bytes[2] = zed_points.data[t * zed_points.point_step + 10];
		z_bytes.bytes[3] = zed_points.data[t * zed_points.point_step + 11];

		point.x = x_bytes.val;
		point.y = y_bytes.val;
		point.z = z_bytes.val;


		if ((point.x != point.x) || (point.y != point.y) || (point.z != point.z)) {
			// std::cout << "nan" << std::endl;
			continue;
		}if ((point.x != 0.0 && 1/point.x == 0.0) || (point.y != 0.0 && 1/point.y == 0.0) || (point.z != 0.0 && 1/point.z == 0.0)) {
			// std::cout << "inf" << std::endl;
			continue;
		}


		tf2::doTransform(point, point, zed_to_odom);
	
		std::pair<uint64_t, uint64_t> coor = world_to_map(point, map.map.info);
		uint64_t index = coor.first + coor.second * map.map.info.width;

		if (lane_mask.data[t] == 255) {
			map.occupancy_params.declare_obstacle(index);
		} else {
			map.occupancy_params.declare_clear(index);
		}

		map.map.data[index] = map.occupancy_params.get_occupancy(index);
	}

}

int main(int argc, char **argv) {

	mapping::Mapping node(argc, argv);
	node.run();

	return 0;
}

