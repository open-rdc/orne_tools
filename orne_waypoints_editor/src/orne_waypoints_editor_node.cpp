#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <std_srvs/Trigger.h>

#include <yaml-cpp/yaml.h>

#include <string>
#include <fstream>
#include <math.h>

#ifdef NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
    i = node.as<T>();
}
#endif

using namespace visualization_msgs;

class WaypointsEditor{
public:
    WaypointsEditor() :
        filename_(""), fp_flag_(false), rate_(1)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("world_frame", world_frame_, std::string("map"));
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
        private_nh.param("save_joy_button", save_joy_button_, 0);

        server.reset(new interactive_markers::InteractiveMarkerServer("waypoints_marker_server", "", false));

        initMenu();

        ros::NodeHandle nh;
        waypoints_viz_sub_ = nh.subscribe("waypoints_viz", 1, &WaypointsEditor::waypointsVizCallback, this);
        waypoints_joy_sub_ = nh.subscribe("waypoints_joy", 1, &WaypointsEditor::waypointsJoyCallback, this);
        finish_pose_sub_ = nh.subscribe("finish_pose", 1, &WaypointsEditor::finishPoseCallback, this);
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker_descriptions",1);
        save_server_ = nh.advertiseService("save_waypoints", &WaypointsEditor::saveWaypointsCallback, this);

        private_nh.param("filename", filename_, filename_);
        if(filename_ != ""){
            ROS_INFO_STREAM("Read waypoints data from " << filename_);
            if(readFile(filename_)) {
                fp_flag_ = true;
                makeMarker();
            } else {
                ROS_ERROR("Failed loading waypoints file");
            }
        } else {
            ROS_ERROR("waypoints file doesn't have name");
        }
    }


    ~WaypointsEditor(){
        server.reset();
    }

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "'";
        // s << "Feedback from marker '" << feedback->marker_name << "'";
        //   << " /control '"<< feedback->control_name << "' ";
    
        std::ostringstream mouse_point_ss;
        // if(feedback->mouse_point_valid){
        //   mouse_point_ss << "at" << feedback->mouse_point.x
        //     << "," << feedback->mouse_point.y
        //     << "," << feedback->mouse_point.z
        //     << "in frame" << feedback->header.frame_id;
        // }
    
        switch(feedback->event_type){
          case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            ROS_INFO_STREAM(s.str() << ":button click" << mouse_point_ss.str() << ".");
            break;
    
          case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM(s.str() << ":menu item" << feedback->menu_entry_id << "clicked" << mouse_point_ss.str() << ".");
            break;
    
          case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            ROS_INFO_STREAM(s.str() << ":pose changed"
                << "\nposition = "
                << feedback->pose.position.x
                << "," << feedback->pose.position.y
                << "," << feedback->pose.position.z
                << "\norientation = "
                << "," << feedback->pose.orientation.x
                << "," << feedback->pose.orientation.y
                << "," << feedback->pose.orientation.z);
                // << "\nframe:" << feedback->header.frame_id
                // << "time" << feedback->header.stamp.sec << "sec,"
                // << feedback->header.stamp.nsec << "nsec");
            break;
    
          // case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
          //   ROS_INFO_STREAM(s.str() << ":mouse down" << mouse_point_ss.str() << ".");
          //   break;
    
          // case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
          //   ROS_INFO_STREAM(s.str() << ":mouse up" << mouse_point_ss.str() << ".");
          //   break;
        }

        server->applyChanges();

        if (feedback->marker_name == "finish_pose") {
            finish_pose_.pose = feedback->pose;
        } else {
          std::string str_wp_num = feedback->marker_name;
          waypoints_.at(std::stoi(str_wp_num.substr(8))) = feedback->pose.position;
        }
    }

    void initMenu(){
        interactive_markers::MenuHandler::EntryHandle wp_delete_menu_handler = wp_menu_handler_.insert("delete", boost::bind(&WaypointsEditor::wpDeleteCb, this, _1));
        interactive_markers::MenuHandler::EntryHandle wp_insert_menu_handler = wp_menu_handler_.insert("Insert");

        interactive_markers::MenuHandler::EntryHandle wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Prev", boost::bind(&WaypointsEditor::wpInsertCb, this, _1));
        wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Next", boost::bind(&WaypointsEditor::wpInsertCb, this, _1));

        interactive_markers::MenuHandler::EntryHandle fp_delete_menu_handler = fp_menu_handler_.insert("delete", boost::bind(&WaypointsEditor::fpDeleteCb, this, _1));
        
    }

    void wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        std::string str_wp_num = feedback->marker_name;
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
        resetMarkerDescription();
        waypoints_.erase(waypoints_.begin() + std::stoi(str_wp_num.substr(8)));
        makeMarker();
    }

    void wpInsertCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        std::string str_wp_num = feedback->marker_name;
        ROS_INFO_STREAM("insert : " << feedback->menu_entry_id);
        if (feedback->menu_entry_id == 3){
            geometry_msgs::Pose pose = feedback->pose;
            pose.position.x = pose.position.x - 1.0;
            waypoints_.insert(waypoints_.begin() + std::stoi(str_wp_num.substr(8)), pose.position);           
        } else if (feedback->menu_entry_id == 4) {
            geometry_msgs::Pose pose = feedback->pose;
            pose.position.x = pose.position.x + 1.0;
            waypoints_.insert(waypoints_.begin() + std::stoi(str_wp_num.substr(8)) + 1, pose.position);           
        }
        makeMarker();
    }

    void fpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        std::string str_wp_num = feedback->marker_name;
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
        resetMarkerDescription();
        fp_flag_ = false;
        makeMarker();
    }

    void makeMarker(){
        server->clear();
        server->applyChanges();

        makeWaypointsMarker();
        makeFinishPoseMarker();

        applyMenu();
        server->applyChanges();
    }

    void resetMarkerDescription(){
        for (int i=0; i!=waypoints_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(i);
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            std::stringstream name;
            name << "waypoint";
            marker.ns = name.str();
            marker.id = i;
            uint8_t DELETEALL = 3;
            marker.action = DELETEALL;
            marker_description_.markers.push_back(marker);
        }
        if (fp_flag_) {
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = "goal";
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            std::stringstream name;
            name << "finish pose";
            marker.ns = name.str();
            marker.id = 0;
            uint8_t DELETEALL = 3;
            marker.action = DELETEALL;
            marker_description_.markers.push_back(marker);
        }
        marker_description_pub_.publish(marker_description_);
    }

    void publishMarkerDescription(){

        for (int i=0; i!=waypoints_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(i);
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            std::stringstream name;
            name << "waypoint";
            marker.ns = name.str();
            marker.id = i;
            marker.pose.position = waypoints_.at(i);
            marker.pose.position.z = 3.0;
            marker.scale.z = 2.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }
        if (fp_flag_) {
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = "goal";
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time::now();
            std::stringstream name;
            name << "finish pose";
            marker.ns = name.str();
            marker.id = 0;
            marker.pose = finish_pose_.pose;
            marker.pose.position.z = 3.0;
            marker.scale.z = 2.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }
        marker_description_pub_.publish(marker_description_);
    }

    void makeWaypointsMarker(){

        for (int i=0; i!=waypoints_.size(); i++){
            InteractiveMarker int_marker;
            int_marker.header.frame_id = world_frame_;
            int_marker.pose.position = waypoints_.at(i);
            int_marker.scale = 1;
            int_marker.name = "waypoint"+std::to_string(i);
            int_marker.description = "waypoint"+std::to_string(i);

            InteractiveMarkerControl control;
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
            int_marker.controls.push_back(control);
    
            Marker marker;
            marker.type = Marker::SPHERE;
            marker.scale.x = 0.8;
            marker.scale.y = 0.8;
            marker.scale.z = 0.8;
            marker.color.r = 0.08;
            marker.color.g = 0.0;
            marker.color.b = 0.8;
            marker.color.a = 0.5;
            control.markers.push_back(marker);
            
            control.always_visible = true;
            int_marker.controls.push_back(control);
    
            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&WaypointsEditor::processFeedback, this, _1));
        }
        
    }

    void makeFinishPoseMarker(){

        if (fp_flag_) {
            InteractiveMarker int_marker;
            int_marker.header.frame_id = world_frame_;
            int_marker.pose = finish_pose_.pose;
            int_marker.scale = 1;
            int_marker.name = "finish_pose";
            int_marker.description = "finish_pose";

            InteractiveMarkerControl control;
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
            int_marker.controls.push_back(control);
    
            Marker marker;
            marker.type = Marker::ARROW;
            marker.scale.x = 1.0;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 0.08;
            marker.color.g = 0.0;
            marker.color.b = 0.8;
            marker.color.a = 0.8;
            control.markers.push_back(marker);
            
            control.always_visible = true;
            int_marker.controls.push_back(control);
    
            server->insert(int_marker);
            server->setCallback(int_marker.name, boost::bind(&WaypointsEditor::processFeedback, this, _1));
        }
    }

    void applyMenu(){
        for (int i=0; i!=waypoints_.size(); i++){
            wp_menu_handler_.apply(*server, "waypoint"+std::to_string(i));
        }
        if (fp_flag_){
            fp_menu_handler_.apply(*server, "finish_pose");
        }
        server->applyChanges();
    }

    bool readFile(const std::string &filename){
        waypoints_.clear();
        try{
            std::ifstream ifs(filename.c_str(), std::ifstream::in);
            if(ifs.good() == false){
                return false;
            }

            YAML::Node node;
            
            #ifdef NEW_YAMLCPP
                node = YAML::Load(ifs);
            #else
                YAML::Parser parser(ifs);
                parser.GetNextDocument(node);
            #endif

            #ifdef NEW_YAMLCPP
                const YAML::Node &wp_node_tmp = node["waypoints"];
                const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
            #else
                const YAML::Node *wp_node = node.FindValue("waypoints");
            #endif

            if(wp_node != NULL){
                for(int i=0; i < wp_node->size(); i++){
                    geometry_msgs::Point point;

                    (*wp_node)[i]["point"]["x"] >> point.x;
                    (*wp_node)[i]["point"]["y"] >> point.y;
                    (*wp_node)[i]["point"]["z"] >> point.z;

                    waypoints_.push_back(point);
                }
            }else{
                return false;
            }

            #ifdef NEW_YAMLCPP
                const YAML::Node &fp_node_tmp = node["finish_pose"];
                const YAML::Node *fp_node = fp_node_tmp ? &fp_node_tmp : NULL;
            #else
                const YAML::Node *fp_node = node.FindValue("finish_pose");
            #endif

            if(fp_node != NULL){
                (*fp_node)["pose"]["position"]["x"] >> finish_pose_.pose.position.x;
                (*fp_node)["pose"]["position"]["y"] >> finish_pose_.pose.position.y;
                (*fp_node)["pose"]["position"]["z"] >> finish_pose_.pose.position.z;

                (*fp_node)["pose"]["orientation"]["x"] >> finish_pose_.pose.orientation.x;
                (*fp_node)["pose"]["orientation"]["y"] >> finish_pose_.pose.orientation.y;
                (*fp_node)["pose"]["orientation"]["z"] >> finish_pose_.pose.orientation.z;
                (*fp_node)["pose"]["orientation"]["w"] >> finish_pose_.pose.orientation.w;
            }else{
                return false;
            }

        }catch(YAML::ParserException &e){
            return false;

        }catch(YAML::RepresentationException &e){
            return false;
        }

        return true;
    }

    void waypointsVizCallback(const geometry_msgs::PointStamped &msg){
        ROS_INFO_STREAM("point = " << msg);
        waypoints_.push_back(msg.point);
        makeMarker();
    }

    void waypointsJoyCallback(const sensor_msgs::Joy &msg) {
        static ros::Time saved_time(0.0);
        if(msg.buttons[save_joy_button_] == 1 && (ros::Time::now() - saved_time).toSec() > 3.0){
            tf::StampedTransform robot_gl;
            try{
                tf_listener_.lookupTransform(world_frame_, robot_frame_, ros::Time(0.0), robot_gl);
                geometry_msgs::Point point;
                point.x = robot_gl.getOrigin().x();
                point.y = robot_gl.getOrigin().y();
                point.z = robot_gl.getOrigin().z();
                waypoints_.push_back(point);
                saved_time = ros::Time::now();
                makeMarker();
            }catch(tf::TransformException &e){
                ROS_WARN_STREAM("tf::TransformException: " << e.what());
            }
        }
    }

    void finishPoseCallback(const geometry_msgs::PoseStamped &msg){
        ROS_INFO_STREAM("pose = " << msg);
        finish_pose_ = msg;
        fp_flag_ = true;
        makeMarker();
    }

    bool saveWaypointsCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
        if (save()) {
            response.success = true;
            return true;
        } else {
            response.success = false;
            return false;
        }
    }

    bool save(){
        if (fp_flag_) {
            std::ofstream ofs(filename_.c_str(), std::ios::out);
            
            ofs << "waypoints:" << std::endl;
            for(int i=0; i < waypoints_.size(); i++){
                ofs << "    " << "- point:" << std::endl;
                ofs << "        x: " << waypoints_[i].x << std::endl;
                ofs << "        y: " << waypoints_[i].y << std::endl;
                ofs << "        z: " << waypoints_[i].z << std::endl;
            }
            
            ofs << "finish_pose:"           << std::endl;
            ofs << "    header:"            << std::endl;
            ofs << "        seq: "          << finish_pose_.header.seq << std::endl;
            ofs << "        stamp: "        << finish_pose_.header.stamp << std::endl;
            ofs << "        frame_id: "     << finish_pose_.header.frame_id << std::endl;;
            ofs << "    pose:"              << std::endl;
            ofs << "        position:"      << std::endl;
            ofs << "            x: "        << finish_pose_.pose.position.x << std::endl;
            ofs << "            y: "        << finish_pose_.pose.position.y << std::endl;
            ofs << "            z: "        << finish_pose_.pose.position.z << std::endl;
            ofs << "        orientation:"   << std::endl;
            ofs << "            x: "        << finish_pose_.pose.orientation.x << std::endl;
            ofs << "            y: "        << finish_pose_.pose.orientation.y << std::endl;
            ofs << "            z: "        << finish_pose_.pose.orientation.z << std::endl;
            ofs << "            w: "        << finish_pose_.pose.orientation.w << std::endl;

            ofs.close();

            ROS_INFO_STREAM("write success");
            return true;
        } else {
            ROS_WARN_STREAM("Please set finish_pose");
            return false;
        }

    }

    void run() {
        while(ros::ok()){
            rate_.sleep();
            ros::spinOnce();
            publishMarkerDescription();
        }
    }

private:
    ros::Subscriber waypoints_viz_sub_;
    ros::Subscriber waypoints_joy_sub_;
    ros::Subscriber finish_pose_sub_;
    // ros::Subscriber syscommand_sub_;
    ros::Publisher marker_description_pub_;
    std::vector<geometry_msgs::Point> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    visualization_msgs::MarkerArray marker_description_;
    tf::TransformListener tf_listener_;
    int save_joy_button_;
    std::string filename_;
    std::string world_frame_;
    std::string robot_frame_;
    ros::ServiceServer save_server_;

    bool fp_flag_;
    ros::Rate rate_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    interactive_markers::MenuHandler wp_menu_handler_;
    interactive_markers::MenuHandler fp_menu_handler_;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "pose_marker");
  WaypointsEditor w_edit;
  w_edit.run();

  return 0;
}
