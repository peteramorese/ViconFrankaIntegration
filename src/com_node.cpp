#include <math.h>
#include "ros/ros.h"
#include "vicon_franka_integration/PlanningQuery_srv.h"
#include "vicon_franka_integration/Strategy.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
/*
struct EnvironmentSub {
        int num_bags;
        bool bags_found;
        bool& begin;
        bool& run;

        // This will hold the locations of the bags that are found, and will eventually hold
        // the hard-coded drop off locations (edited manually in the code)
        river_ros::BagConfigPoseArray_msg pose_array_copy;

        // This maps the location labels to the array index for easy lookup
        std::unordered_map<std::string, int> label_ind_map;

        EnvironmentSub(bool& begin_, bool& run_) : begin(begin_), run(run_) {
                run = true;
                bags_found = false;
        }

        void envSubCB(const river_ros::BagConfigPoseArray_msg::ConstPtr& env_pose_array){
                std::cout<<"recieved msg BagConfigPoseArray"<<std::endl;
                bags_found = env_pose_array->bags_found;
                if (bags_found && begin && run) {
                        run = false;
                        // Must copy over all data and store in local variable because msg is deallocated
                        pose_array_copy.domain_labels = env_pose_array->domain_labels;
                        pose_array_copy.pose_array.header.stamp = env_pose_array->pose_array.header.stamp;
                        num_bags = env_pose_array->pose_array.poses.size();
                        pose_array_copy.pose_array.poses.resize(num_bags);
                        for (int i=0; i<num_bags; ++i)  {
                                pose_array_copy.pose_array.poses[i].position.x = env_pose_array->pose_array.poses[i].position.x;
                                pose_array_copy.pose_array.poses[i].position.y = env_pose_array->pose_array.poses[i].position.y;
                                pose_array_copy.pose_array.poses[i].position.z = env_pose_array->pose_array.poses[i].position.z;
                                pose_array_copy.pose_array.poses[i].orientation.x = env_pose_array->pose_array.poses[i].orientation.x;
                                pose_array_copy.pose_array.poses[i].orientation.y = env_pose_array->pose_array.poses[i].orientation.y;
                                pose_array_copy.pose_array.poses[i].orientation.z = env_pose_array->pose_array.poses[i].orientation.z;
                                pose_array_copy.pose_array.poses[i].orientation.w = env_pose_array->pose_array.poses[i].orientation.w;
 //std::cout<<" MAPPING: "<<pose_array_copy.domain_labels[i]<<" to: "<<i<<std::endl;
                                //label_ind_map[pose_array_copy.domain_labels[i]] = i;
                        }
                        //std::cout<<" ME MAP L_0 AND L_1"<<std::endl;
                        //label_ind_map["L__0"] = 0;
                        //label_ind_map["L_1"] = 1;
                } else {
                        std::cout<<" --bags were not found or TP not initiated"<<std::endl;
                }
        }
        void mapSize(const std::string& label) {
                std::cout<<" MAPPING: "<<label<<" to: "<<pose_array_copy.pose_array.poses.size() - 1<<std::endl;
                label_ind_map[label] = pose_array_copy.pose_array.poses.size() - 1;
        }
};
*/

class RetrieveData {
        private:
                class callbackdata {
			private:
				float x_offset = -.021544;
				float y_offset = .19416;
				float z_offset = .038176 - .060;
                        public:
                                void sub_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg_ptr) {
                                        //std::cout<< "x: " << pose_msg_ptr->pose.position.x << std::endl;
                                        //std::cout<< "y: " << pose_msg_ptr->pose.position.y << std::endl;
                                        //std::cout<< "z: " << pose_msg_ptr->pose.position.z << std::endl;
                                        configptr->pose.position.x = pose_msg_ptr->pose.position.x + x_offset;
                                        configptr->pose.position.y = pose_msg_ptr->pose.position.y + y_offset;
                                        configptr->pose.position.z = pose_msg_ptr->pose.position.z + z_offset;
                                        configptr->pose.orientation.x = pose_msg_ptr->pose.orientation.x;
                                        configptr->pose.orientation.y = pose_msg_ptr->pose.orientation.y;
                                        configptr->pose.orientation.z = pose_msg_ptr->pose.orientation.z;
                                        configptr->pose.orientation.w = pose_msg_ptr->pose.orientation.w;
                                }
                                geometry_msgs::PoseStamped config;
                                geometry_msgs::PoseStamped* configptr = &config;
                };

                int Navg;
                bool hasdata;   
                //geometry_msgs::PoseStamped avgConfig;
                ros::NodeHandle* SUB_NH;
		std::vector<callbackdata> sub_data;
		ros::Subscriber sub_box_1, sub_box_2, sub_box_3;
		geometry_msgs::PoseArray sample_pose_avg;
        public:
                RetrieveData(int Navg_, ros::NodeHandle* SUB_NH_) : Navg(Navg_), SUB_NH(SUB_NH_){
                        hasdata = false;
			sub_data.resize(3);
			std::cout<<"sub data size: "<<sub_data.size()<<std::endl;
			sub_box_1 = SUB_NH->subscribe("/vrpn_client_node/smallBox1/pose", 10, &callbackdata::sub_callback, &sub_data[0]);
			sub_box_2 = SUB_NH->subscribe("/vrpn_client_node/smallBox2/pose", 10, &callbackdata::sub_callback, &sub_data[1]);
			sub_box_3 = SUB_NH->subscribe("/vrpn_client_node/smallBox3/pose", 10, &callbackdata::sub_callback, &sub_data[2]);
		}


		void retrieve() {
			//ros::Subscriber subscriber = SUB.subscribe("/vrpn_client_node/smallBox1/pose", 10, &callbackdata::sub_callback, &sub_data);

			sample_pose_avg.poses.clear();
			sample_pose_avg.poses.resize(3);
			for (int i=0; i<3; ++i) {
				geometry_msgs::Pose sample_pose;
				//sample_pose_avg.resize(7);
				sample_pose_avg.poses[i].position.x = 0;
				sample_pose_avg.poses[i].position.y = 0;
				sample_pose_avg.poses[i].position.z = 0;
				sample_pose_avg.poses[i].orientation.x = 0;
				sample_pose_avg.poses[i].orientation.y = 0;
				sample_pose_avg.poses[i].orientation.z = 0;
				sample_pose_avg.poses[i].orientation.w = 0;

				ros::Rate r(30);
				ros::spinOnce();
				int Navg_actual = 0;
				while (Navg_actual<Navg) {
					ros::spinOnce();
					if (sub_data[i].configptr->pose.position.x == 0.0){
						std::cout<<"Bad data"<<std::endl;
						r.sleep();
					} else {
						sample_pose.position.x += sub_data[i].configptr->pose.position.x;         
						sample_pose.position.y += sub_data[i].configptr->pose.position.y;
						sample_pose.position.z += sub_data[i].configptr->pose.position.z;
						sample_pose.orientation.x += sub_data[i].configptr->pose.orientation.x;
						sample_pose.orientation.y += sub_data[i].configptr->pose.orientation.y;
						sample_pose.orientation.z += sub_data[i].configptr->pose.orientation.z;
						sample_pose.orientation.w += sub_data[i].configptr->pose.orientation.w;
						Navg_actual++;
						r.sleep();
					}
					if (!ros::ok()){
						break;
					}
				}
				sample_pose_avg.poses[i].position.x = sample_pose.position.x/Navg_actual;
				sample_pose_avg.poses[i].position.y = sample_pose.position.y/Navg_actual;
				sample_pose_avg.poses[i].position.z = sample_pose.position.z/Navg_actual;
				sample_pose_avg.poses[i].orientation.x = sample_pose.orientation.x/Navg_actual;
				sample_pose_avg.poses[i].orientation.y = sample_pose.orientation.y/Navg_actual;
				sample_pose_avg.poses[i].orientation.z = sample_pose.orientation.z/Navg_actual;
				sample_pose_avg.poses[i].orientation.w = sample_pose.orientation.w/Navg_actual;
				/*
				   for (int i=0; i<7; i++) {
				   sample_pose_avg[i] = sample_pose[i]/Navg_actual;
				   }
				   */
			}
			std::cout<<"sample pose size: "<<sample_pose_avg.poses.size()<<std::endl;
                        hasdata = true;
                }

                geometry_msgs::Pose* returnConfigPtr (int ind) {
                        if (!hasdata) {
                                std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
                        } else {
                                return &sample_pose_avg.poses[ind];
                        }
		}

		geometry_msgs::PoseArray* returnConfigArrPtr() {
                        if (!hasdata) {
                                std::cout<< "Call retrieve() before calling returnConfig()" <<std::endl;
                        } else {
                                return &sample_pose_avg;
                        }
		}
};



class PredicateGenerator {
        private:
                struct coord {
                        double x, y, z;
                        std::string label;
                };
                std::vector<coord> locations;
                //std::vector<bool> is_occupied;
                double max_r;
                // Store quaternion for all hard-coded drop off locations, this assumes
                // all bags should be placed in the same orientation
                double qx, qy, qz, qw;
        public:
                PredicateGenerator(double max_r_) : max_r(max_r_) {}
                void addLocation(double x, double y, double z, const std::string& coord_label) {
                        coord temp_coord;
                        temp_coord.x = x;
                        temp_coord.y = y;
                        temp_coord.z = z;
                        temp_coord.label = coord_label;
                        locations.push_back(temp_coord);
                        //is_occupied.push_back(false);
                }
                geometry_msgs::Point getLocation(const std::string& coord_label) {
			geometry_msgs::Point ret_pose;
                        for (int i=0; i<locations.size(); ++i) {
                                if (locations[i].label == coord_label) {
                                        ret_pose.x = locations[i].x;
                                        ret_pose.y = locations[i].y;
                                        ret_pose.z = locations[i].z;
					/*
                                        ret_pose.orientation.x = qx;
                                        ret_pose.orientation.y = qy;
                                        ret_pose.orientation.z = qz;
                                        ret_pose.orientation.w = qw;
					*/
                                        break;
                                }
                        }
			return ret_pose;
                }
		/*
                void getUnoccupiedLocation(geometry_msgs::Pose& ret_pose, std::string& ret_label) {
                        // This will return the first unoccupied location, then set
                        // that location to 'occupied'
                        for (int i=0; i<locations.size(); ++i) {
                                if (!is_occupied[i]) {
                                        ret_pose.position.x = locations[i].x;
                                        ret_pose.position.y = locations[i].y;
                                        ret_pose.position.z = locations[i].z;
                                        ret_pose.orientation.x = qx;
                                        ret_pose.orientation.y = qy;
                                        ret_pose.orientation.z = qz;
                                        ret_pose.orientation.w = qw;
                                        ret_label = locations[i].label;
                                        is_occupied[i] = true;
                                        break;
                                }
                        }
                }
		*/
                void setOrientation(double qx_, double qy_, double qz_, double qw_) {
                        qx = qx_;
                        qy = qy_;
                        qz = qz_;
                        qw = qw_;
                }
                double cartDist(double x, double y, double z, const coord& coord) {
                        double sum = 0;
                        sum += (x - coord.x)*(x - coord.x);
                        sum += (y - coord.y)*(y - coord.y);
                        sum += (z - coord.z)*(z - coord.z);
                        sum = std::sqrt(sum);
                        return sum;
                }
                int getNumLocs() {
                        return locations.size();
                }
		/*
                bool getNearestLocLabel(double x, double y, double z, std::string& ret_coord_label) {
                        // This will return the nearest location label, then set that location
                        // to 'occupied', so that the same nearest location label cannot be 
                        // returned twice
                        double min_dist;
                        int locations_ind;
                        for (int i=0; i<locations.size(); ++i) {
                                if (!is_occupied[i]) {
                                        double temp_dist;
                                        temp_dist = cartDist(x, y, z, locations[i]);
                                        if (i == 0 || temp_dist < min_dist) {
                                                min_dist = temp_dist;
                                                locations_ind = i;
                                        }
                                }
                        }
                        if (min_dist < max_r) {
                                ret_coord_label = locations[locations_ind].label;
                                is_occupied[locations_ind] = true;
                                return true;
                        } else {
                                return false;
                                ROS_WARN("Did not find a location within the maximum radius");
                        }
                }
		*/
		bool getNearestLocLabel(geometry_msgs::Point loc, std::string& ret_coord_label) {
                        double min_dist;
                        int locations_ind;
                        for (int i=0; i<locations.size(); ++i) {
				double temp_dist;
				temp_dist = cartDist(loc.x, loc.y, loc.z, locations[i]);
				if (i == 0 || temp_dist < min_dist) {
					min_dist = temp_dist;
					locations_ind = i;
				}
                        }
                        if (min_dist < max_r) {
                                ret_coord_label = locations[locations_ind].label;
                                //is_occupied[locations_ind] = true;
                                return true;
                        } else {
                                return false;
                                ROS_WARN("Did not find a location within the maximum radius");
                        }
		}
		bool getPredicates(geometry_msgs::PoseArray* obj_locs, std::vector<std::string>& ret_state) {
			ret_state.clear();
			ret_state.resize(obj_locs->poses.size());
			std::cout<<"ret_state in get pred: "<<ret_state.size()<<std::endl;
			bool found = true;
			for (int i=0; i<obj_locs->poses.size(); ++i) {
				std::string temp_label;
				if (getNearestLocLabel(obj_locs->poses[i].position, temp_label)){
					ret_state[i] = temp_label;
				} else {
					ROS_WARN("Cannot get predicates");
					found = false;
					break;
				}
			}
			return found;
		}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "com_node");
	ros::NodeHandle com_NH;

	RetrieveData vicon_data(10, &com_NH);
	PredicateGenerator pred_gen(0.15); // set detection radius to 15 cm
	pred_gen.addLocation(.4, -.4, .09, "l0");
	pred_gen.addLocation(.4, .4, .09, "l1");
	pred_gen.addLocation(0, .4, .09, "l2");
	pred_gen.addLocation(-.4, -.4, .09, "l3");
	pred_gen.addLocation(-.4, .4, .09, "l4");

	ros::ServiceClient strategy_srv_client = com_NH.serviceClient<vicon_franka_integration::Strategy>("/com_node/strategy");
	vicon_franka_integration::Strategy strategy_srv;	
	ros::ServiceClient plan_query_client = com_NH.serviceClient<vicon_franka_integration::PlanningQuery_srv>("/com_node/planning_query");
	vicon_franka_integration::PlanningQuery_srv plan_query_srv;	

	geometry_msgs::PoseArray* data = vicon_data.returnConfigArrPtr();
	int j = 0;
	std::vector<std::string> bag_labels = {"box0", "box1", "box2"};
	std::vector<std::string> bag_domain_labels = {"domain", "domain", "domain"};
	std::string holding_state = "";
	geometry_msgs::Quaternion temp_orient;
	while (ros::ok()) {
		vicon_data.retrieve();
		std::vector<std::string> ret_state;
		bool found = pred_gen.getPredicates(data, ret_state);
		if (found) {
			ROS_INFO("Found predicates");	
		} else {
			ROS_WARN("Did not find predicates, breaking...");
			break;
		}
		std::cout<<"ret_state size: "<<ret_state.size()<<std::endl;
		for (int ii=0; ii<ret_state.size(); ii++) {
			std::cout<<"ret_state: "<<ret_state[ii]<<std::endl;
		}
		strategy_srv.request.world_config = ret_state;
		strategy_srv.request.prev_state = holding_state;
		if (strategy_srv_client.call(strategy_srv)) {
			std::string action = strategy_srv.response.action;
			int obj_ind = strategy_srv.response.obj;
			std::string to_loc = strategy_srv.response.to_loc;
			holding_state = strategy_srv.response.curr_state;
			std::cout<<"Action received: "<<action<<std::endl;
			if (action == "transit") {
				plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				temp_orient = plan_query_srv.request.manipulator_pose.orientation;
				std::cout<<"size: "<<data->poses.size()<<std::endl;
				for (int ii=0; ii<data->poses.size(); ii++) {
					std::cout<<"data stuff for box"<<ii<<": "<<data->poses[1].position.x<<std::endl;
				}
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = bag_labels;
				plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
			} else if (action == "transfer") {
				plan_query_srv.request.manipulator_pose.position = pred_gen.getLocation(to_loc);
				plan_query_srv.request.manipulator_pose.orientation = temp_orient;
				temp_orient = plan_query_srv.request.manipulator_pose.orientation;
				plan_query_srv.request.bag_poses = *data;
				plan_query_srv.request.setup_environment = true;
				plan_query_srv.request.bag_labels = bag_labels;
				plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;

			} else if (action == "grasp") {
				//plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				//plan_query_srv.request.bag_poses = data;
				plan_query_srv.request.setup_environment = false;
				//plan_query_srv.request.bag_labels = bag_labels;
				//plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = bag_labels[obj_ind];
				plan_query_srv.request.drop_object = "none";
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
			} else if (action == "release") {
				//plan_query_srv.request.manipulator_pose = data->poses[obj_ind];
				//plan_query_srv.request.bag_poses = data;
				plan_query_srv.request.setup_environment = false;
				//plan_query_srv.request.bag_labels = bag_labels;
				//plan_query_srv.request.bag_domain_labels = bag_domain_labels;
				plan_query_srv.request.pickup_object = "none";
				plan_query_srv.request.drop_object = bag_labels[obj_ind];
				plan_query_srv.request.planning_domain = "domain";
				plan_query_srv.request.safe_config = false;
			} else {
				ROS_WARN("Unrecognized action");
			}
			if (plan_query_client.call(plan_query_srv)) {
				ROS_INFO("Completed action service");
			} else {
				ROS_WARN("Did not find plan query service");
			}
		} else {
			ROS_WARN("Did not find strategy service");
		}
	}
	return 0;
}
