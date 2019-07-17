#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
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
        filename_(""), fp_flag_(false), rate_(2)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("world_frame", world_frame_, std::string("map"));
        private_nh.param("robot_frame", robot_frame_, std::string("base_link"));
       
        private_nh.param("save_joy_button", save_joy_button_, 0);
        //private_nh.param("suspend_size", suspend_size_, 2);
        suspend_size_ = 3;

        server.reset(new interactive_markers::InteractiveMarkerServer("waypoints_marker_server", "", false));

        initMenu();

        ros::NodeHandle nh;
        waypoints_viz_sub_ = nh.subscribe("waypoints_viz", 1, &WaypointsEditor::waypointsVizCallback, this);
        waypoints_joy_sub_ = nh.subscribe("waypoints_joy", 1, &WaypointsEditor::waypointsJoyCallback, this);
        finish_pose_sub_ = nh.subscribe("finish_pose", 1, &WaypointsEditor::finishPoseCallback, this);
        marker_description_pub_ = nh.advertise<visualization_msgs::MarkerArray>("marker_descriptions",1);
        save_server_ = nh.advertiseService("save_waypoints", &WaypointsEditor::saveWaypointsCallback, this);

        private_nh.param("filename", filename_, filename_);
        private_nh.param("suspend_filename", suspend_filename_, suspend_filename_);

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

        if(suspend_filename_ != ""){
            ROS_INFO_STREAM("Read suspned data from " << suspend_filename_);
            if(suspend_readFile(suspend_filename_)) {
                suspend_makeMarker();
                ROS_INFO("read suspend");
            } else {
                ROS_ERROR("Failed loading suspend file");
            }
        } else {
            ROS_ERROR("suspned file doesn't have name");
        }

    }


    ~WaypointsEditor(){
        server.reset();
    }

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "'";
    
        std::ostringstream mouse_point_ss;
    
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
            break;
        }

        server->applyChanges();

        if (feedback->marker_name == "finish_pose") {
            finish_pose_.pose = feedback->pose;
        } else {
          waypoints_.at(std::stoi(feedback->marker_name)) = feedback->pose.position;
        }
    }

    void suspend_processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
        std::ostringstream s;
        s << "Feedback from marker '" << feedback->marker_name << "'";
    
        std::ostringstream mouse_point_ss;
    
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
            break;
        }

        server->applyChanges();

        
        
        std::string name;
        name.clear();
        name = feedback->marker_name;
        
        std::cout << name << std::endl;
        
        if(name.at(0) == 's')
            suspends_.at(std::stoi((feedback->marker_name).substr(1))) = feedback->pose;
        if(name.at(0) == 'r')
            resumes_.at(std::stoi((feedback->marker_name).substr(1))) = feedback->pose;
    }


    void initMenu(){
        interactive_markers::MenuHandler::EntryHandle wp_delete_menu_handler = wp_menu_handler_.insert("delete", boost::bind(&WaypointsEditor::wpDeleteCb, this, _1));
        interactive_markers::MenuHandler::EntryHandle wp_insert_menu_handler = wp_menu_handler_.insert("Insert");

        interactive_markers::MenuHandler::EntryHandle wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Prev", boost::bind(&WaypointsEditor::wpInsertCb, this, _1));
        wp_mode = wp_menu_handler_.insert(wp_insert_menu_handler, "Next", boost::bind(&WaypointsEditor::wpInsertCb, this, _1));


        interactive_markers::MenuHandler::EntryHandle sp_delete_menu_handler = sp_menu_handler_.insert("sp_delete", boost::bind(&WaypointsEditor::spDeleteCb, this, _1));
        interactive_markers::MenuHandler::EntryHandle sp_insert_menu_handler = sp_menu_handler_.insert("sp_Insert");

        interactive_markers::MenuHandler::EntryHandle sp_mode = sp_menu_handler_.insert(sp_insert_menu_handler, "sp_Prev", boost::bind(&WaypointsEditor::spInsertCb, this, _1));
        sp_mode = sp_menu_handler_.insert(sp_insert_menu_handler, "sp_Next", boost::bind(&WaypointsEditor::spInsertCb, this, _1));
    }

    void wpDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
        int wp_num = std::stoi(feedback->marker_name);
        waypoints_.erase(waypoints_.begin() + wp_num);
        for (int i=wp_num; i<waypoints_.size(); i++) {
            geometry_msgs::Pose p;
            p.position = waypoints_.at(i);
            server->setPose(std::to_string(i), p);
        }
        server->erase(std::to_string((int)waypoints_.size()));
        server->applyChanges();
    }

    void wpInsertCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("menu_entry_id : " << feedback->menu_entry_id);
        int wp_num= std::stoi(feedback->marker_name);
        ROS_INFO_STREAM("insert : " << feedback->menu_entry_id);
        geometry_msgs::Pose p = feedback->pose;
        if (feedback->menu_entry_id == 3){
            p.position.x -= 1.0;
            waypoints_.insert(waypoints_.begin() + wp_num, p.position);

        } else if (feedback->menu_entry_id == 4) {
            p.position.x += + 1.0;
            waypoints_.insert(waypoints_.begin() + wp_num + 1, p.position);
        }

        for (int i=wp_num; i<waypoints_.size()-1; i++) {
            geometry_msgs::Pose p;
            p.position = waypoints_.at(i);
            server->setPose(std::to_string(i), p);
        }
        makeWpInteractiveMarker(std::to_string(waypoints_.size()-1), waypoints_.at(waypoints_.size()-1));
        server->applyChanges();
    }


    void spDeleteCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("delete : " << feedback->marker_name);
        std::cout << "delete suspend" << std::endl;
        int sp_num = std::stoi((feedback->marker_name).substr(1));
        std::cout << "sp_num" << sp_num << std::endl;
        suspends_.erase(suspends_.begin() + sp_num);
        resumes_.erase(resumes_.begin() + sp_num);
        for (int i=sp_num; i<suspends_.size(); i++) {
            geometry_msgs::Pose p;
            geometry_msgs::Pose rp;
            p = suspends_.at(i);
            rp = resumes_.at(i);
            server->setPose("s"+std::to_string(i), p);
            server->setPose("r"+std::to_string(i), rp);
        }

            
            
            std::cout << "server erase " << server->erase("s"+std::to_string((int)suspends_.size())) << std::endl;
            std::cout << "debug server->size()" << server->size();
            
            
            server->applyChanges();
            std::cout << "server erase2 " << server->erase("r"+std::to_string((int)resumes_.size())) << std::endl;
            std::cout << "debug2 server->size()" << server->size() << std::endl;
            server->applyChanges();
    }

    void spInsertCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
        ROS_INFO_STREAM("menu_entry_id : " << feedback->menu_entry_id);
        int sp_num= std::stoi((feedback->marker_name).substr(1));
        ROS_INFO_STREAM("insert : " << feedback->menu_entry_id);
        geometry_msgs::Pose p = feedback->pose;
        geometry_msgs::Pose rp = feedback->pose;

        if (feedback->menu_entry_id == 3){
            p.position.x -= 1.0;
            rp.position.x -= 2.0;
            suspends_.insert(suspends_.begin() + sp_num, p);
            resumes_.insert(resumes_.begin() + sp_num, rp);

        } else if (feedback->menu_entry_id == 4) {
            p.position.x +=  1.0;
            rp.position.x +=  2.0;
            suspends_.insert(suspends_.begin() + sp_num + 1, p);
            resumes_.insert(resumes_.begin() + sp_num + 1, rp);
        }

        for (int i=sp_num; i<suspends_.size()-1; i++) {
            geometry_msgs::Pose p;
            geometry_msgs::Pose rp;
            p = suspends_.at(i);
            rp = resumes_.at(i);
            server->setPose("s"+std::to_string(i), p);
            server->setPose("r"+std::to_string(i), rp);
        }
        makeSpInteractiveMarker("s"+std::to_string(suspends_.size()-1), suspends_.at(suspends_.size()-1));
        makeRpInteractiveMarker("r"+std::to_string(resumes_.size()-1), resumes_.at(suspends_.size()-1));
        server->applyChanges();
    }



    void makeMarker(){
        makeWpsInteractiveMarker();
        makeFinishPoseMarker();

        server->applyChanges();
    }

    void suspend_makeMarker(){
        makeSpsInteractiveMarker();
        makeRpsInteractiveMarker();

        server->applyChanges();
    }


    void publishMarkerDescription(){
        marker_description_.markers.clear();

        for(int i=0; i!=waypoints_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = std::to_string(i);
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time(0);
            std::stringstream name;
            name << "waypoint";
            marker.ns = name.str();
            marker.id = i;
            marker.lifetime = ros::Duration(0.5);
            marker.pose.position = waypoints_.at(i);
            marker.pose.position.z = 3.0;
            marker.scale.z = 2.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }if(fp_flag_){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = "goal";
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time(0);
            std::stringstream name;
            name << "finish pose";
            marker.ns = name.str();
            marker.id = 0;
            marker.lifetime = ros::Duration(0.5);
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

        for(int i=0; i!=suspends_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = "s"+std::to_string(i);
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time(0);
            std::stringstream name;
            name << "suspend";
            marker.ns = name.str();
            marker.id = i;
            marker.lifetime = ros::Duration(0.5);
            marker.pose = suspends_.at(i);
            marker.pose.position.z = 1.5;
            marker.scale.z = 2.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }
        std::cout << "sp size " << suspends_.size() << std::endl;//debug
        marker_description_pub_.publish(marker_description_);


        for(int i=0; i!=resumes_.size(); i++){
            Marker marker;
            marker.type = Marker::TEXT_VIEW_FACING;
            marker.text = "r"+std::to_string(i);
            marker.header.frame_id = world_frame_;
            marker.header.stamp = ros::Time(0);
            std::stringstream name;
            name << "resume";
            marker.ns = name.str();
            marker.id = i;
            marker.lifetime = ros::Duration(0.5);
            marker.pose = resumes_.at(i);
            marker.pose.position.z = 1.5;
            marker.scale.z = 2.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            marker_description_.markers.push_back(marker);
        }
        std::cout << "rp size " << resumes_.size() << std::endl;//debug
        marker_description_pub_.publish(marker_description_);
  
    }

    Marker makeWpMarker(){
        Marker marker;
        marker.type = Marker::SPHERE;
        marker.scale.x = 0.8;
        marker.scale.y = 0.8;
        marker.scale.z = 0.8;
        marker.color.r = 0.08;
        marker.color.g = 0.0;
        marker.color.b = 0.8;
        marker.color.a = 0.5;

        return marker;
    }
    
    Marker makeSpMarker(){
        Marker marker;
        marker.type = Marker::ARROW;
        marker.scale.x = 1.0;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.8;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        return marker;
    }

    Marker makeRpMarker(){
        Marker marker;
        marker.type = Marker::ARROW;
        marker.scale.x = 1.0;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        return marker;
    }
 
    InteractiveMarkerControl& makeWpControl(InteractiveMarker &msg) {
        InteractiveMarkerControl control;
        control.markers.clear();
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        control.always_visible = true;
        control.markers.push_back(makeWpMarker());
        msg.controls.push_back(control);

        return msg.controls.back();
    }

    InteractiveMarkerControl& makeSpControl(InteractiveMarker &msg) {
        InteractiveMarkerControl control;
        control.markers.clear();
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
        control.always_visible = true;
        control.markers.push_back(makeSpMarker());
        msg.controls.push_back(control);

        return msg.controls.back();
    }

    InteractiveMarkerControl& makeRpControl(InteractiveMarker &msg) {
        InteractiveMarkerControl control;
        control.markers.clear();
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
        control.always_visible = true;
        control.markers.push_back(makeRpMarker());
        msg.controls.push_back(control);

        return msg.controls.back();
    }

    void makeWpInteractiveMarker(std::string name, geometry_msgs::Point point){
        InteractiveMarker int_marker;
        int_marker.controls.clear();
        int_marker.header.frame_id = world_frame_;
        int_marker.pose.position = point;
        int_marker.scale = 1;
        int_marker.name = name;
        int_marker.description = name;

        int_marker.controls.push_back(makeWpControl(int_marker));

        server->insert(int_marker, boost::bind(&WaypointsEditor::processFeedback, this, _1));
        wp_menu_handler_.apply(*server, name);
    }

    void makeSpInteractiveMarker(std::string name, geometry_msgs::Pose pose){
        InteractiveMarker int_marker;
        int_marker.controls.clear();
        int_marker.header.frame_id = world_frame_;
        int_marker.pose = pose;
        int_marker.scale = 1;
        int_marker.name = name;
        int_marker.description = name;

        int_marker.controls.push_back(makeSpControl(int_marker));

        server->insert(int_marker, boost::bind(&WaypointsEditor::suspend_processFeedback, this, _1));
        sp_menu_handler_.apply(*server, name);
    }

    void makeRpInteractiveMarker(std::string name, geometry_msgs::Pose pose){
        InteractiveMarker int_marker;
        int_marker.controls.clear();
        int_marker.header.frame_id = world_frame_;
        int_marker.pose = pose;
        int_marker.scale = 1;
        int_marker.name = name;
        int_marker.description = name;

        int_marker.controls.push_back(makeRpControl(int_marker));

        server->insert(int_marker, boost::bind(&WaypointsEditor::suspend_processFeedback, this, _1));
    }


    void makeWpsInteractiveMarker(){
        for (int i=0; i!=waypoints_.size(); i++){
            makeWpInteractiveMarker(std::to_string(i), waypoints_.at(i));
        }
    }

    void makeSpsInteractiveMarker(){
        for (int i=0; i!=suspends_.size(); i++){
            makeSpInteractiveMarker("s"+std::to_string(i), suspends_.at(i));
        }
    }

    void makeRpsInteractiveMarker(){
        for (int i=0; i!=resumes_.size(); i++){
            makeRpInteractiveMarker("r"+std::to_string(i), resumes_.at(i));
        }
    }

    void makeFinishPoseMarker(){
        if (fp_flag_) {
            InteractiveMarker int_marker;
            int_marker.controls.clear();
            int_marker.header.frame_id = world_frame_;
            int_marker.pose = finish_pose_.pose;
            int_marker.scale = 1;
            int_marker.name = "finish_pose";
            int_marker.description = "finish_pose";

            InteractiveMarkerControl control;
            control.markers.clear();
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    
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
    
            server->insert(int_marker, boost::bind(&WaypointsEditor::processFeedback, this, _1));
        }
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

    bool suspend_readFile(const std::string &filename){
        suspends_.clear();
        resumes_.clear();
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


            std::string node_name = "size";
            #ifdef NEW_YAMLCPP
                const YAML::Node &rp_node_tmp = node[node_name];
                const YAML::Node *rp_node = rp_node_tmp ? &rp_node_tmp : NULL;
            #else
                const YAML::Node *rp_node = node.FindValue(node_name);
            #endif

            if(rp_node != NULL){
                (*rp_node) >> suspend_size_;
                std::cout << "read size " << suspend_size_ << std::endl;
            }else{
                return false;
            }

   
        std::cout << "suspend_size" << suspend_size_ << std::endl;
            for(int i=0;i<suspend_size_;i++){
                std::string node_name = "suspend_pose" + std::to_string(i+1);
                #ifdef NEW_YAMLCPP
                    const YAML::Node &sp_node_tmp = node[node_name];
                    const YAML::Node *sp_node = sp_node_tmp ? &sp_node_tmp : NULL;
                #else
                    const YAML::Node *sp_node = node.FindValue(node_name);
                #endif

                if(sp_node != NULL){
                    geometry_msgs::Pose pose;
                    (*sp_node)["position"]["x"] >> pose.position.x;
                    (*sp_node)["position"]["y"] >> pose.position.y;
                    (*sp_node)["position"]["z"] >> pose.position.z;
                    (*sp_node)["orientation"]["x"] >> pose.orientation.x;
                    (*sp_node)["orientation"]["y"] >> pose.orientation.y;
                    (*sp_node)["orientation"]["z"] >> pose.orientation.z;
                    (*sp_node)["orientation"]["w"] >> pose.orientation.w;

                    suspends_.push_back(pose);
                }else{
                    return false;
                }

                node_name = "resume_pose" + std::to_string(i+1);
                #ifdef NEW_YAMLCPP
                    const YAML::Node &rp_node_tmp = node[node_name];
                    const YAML::Node *rp_node = rp_node_tmp ? &rp_node_tmp : NULL;
                #else
                    const YAML::Node *rp_node = node.FindValue(node_name);
                #endif

                if(rp_node != NULL){

                    geometry_msgs::Pose pose;
                    (*rp_node)["position"]["x"] >> pose.position.x;
                    (*rp_node)["position"]["y"] >> pose.position.y;
                    (*rp_node)["position"]["z"] >> pose.position.z;
                    (*rp_node)["orientation"]["x"] >> pose.orientation.x;
                    (*rp_node)["orientation"]["y"] >> pose.orientation.y;
                    (*rp_node)["orientation"]["z"] >> pose.orientation.z;
                    (*rp_node)["orientation"]["w"] >> pose.orientation.w;

                    resumes_.push_back(pose);
                }else{
                    return false;
                }
 
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
        makeWpInteractiveMarker(std::to_string(waypoints_.size()), msg.point);
        server->applyChanges();

        waypoints_.push_back(msg.point);
    }

    void waypointsJoyCallback(const sensor_msgs::Joy &msg) {
        static ros::Time saved_time(0.0);
        if(msg.buttons[save_joy_button_] == 1 && (ros::Time(0) - saved_time).toSec() > 3.0){
            tf::StampedTransform robot_gl;
            try{
                tf_listener_.lookupTransform(world_frame_, robot_frame_, msg.header.stamp, robot_gl);
                geometry_msgs::Point point;
                point.x = robot_gl.getOrigin().x();
                point.y = robot_gl.getOrigin().y();
                point.z = robot_gl.getOrigin().z();
                makeWpInteractiveMarker(std::to_string(waypoints_.size()), point);
                waypoints_.push_back(point);
                server->applyChanges();
                saved_time = msg.header.stamp;
            }catch(tf::TransformException &e){
                ROS_WARN_STREAM("tf::TransformException: " << e.what());
            }
        }
    }

    void finishPoseCallback(const geometry_msgs::PoseStamped &msg){
        ROS_INFO_STREAM("pose = " << msg);
        finish_pose_ = msg;
        if (fp_flag_) {
            server->setPose("finish_pose", msg.pose);
        } else {
            fp_flag_ = true;
            makeFinishPoseMarker();
        }
        server->applyChanges();
    }

    bool saveWaypointsCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response) {
        if (save() && suspend_save()) {
            response.success = true;
            return true;
        } else {
            response.success = false;
            return false;
        }
    }

    bool save(){
        if (fp_flag_) {
            std::ofstream ofs(suspend_filename_.c_str(), std::ios::out);
            
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

    bool suspend_save(){

        std::ofstream ofs(suspend_filename_.c_str(), std::ios::out);
        

        ofs << "size" << ": " << suspends_.size() << std::endl;
        for(int i=0; i < suspends_.size(); i++){
            ofs << "suspend_pose" << std::to_string(i+1) << ":" << std::endl;
            ofs << "    " << "position:" << std::endl;
            ofs << "     x: " << suspends_[i].position.x << std::endl;
            ofs << "     y: " << suspends_[i].position.y << std::endl;
            ofs << "     z: " << suspends_[i].position.z << std::endl;
            ofs << "    " << "orientation:" << std::endl;
            ofs << "     x: " << suspends_[i].orientation.x << std::endl;
            ofs << "     y: " << suspends_[i].orientation.y << std::endl;
            ofs << "     z: " << suspends_[i].orientation.z << std::endl;
            ofs << "     w: " << suspends_[i].orientation.w << std::endl;

            ofs << "resume_pose" << std::to_string(i+1) << ":" << std::endl;
            ofs << "    " << "position:" << std::endl;
            ofs << "     x: " << resumes_[i].position.x << std::endl;
            ofs << "     y: " << resumes_[i].position.y << std::endl;
            ofs << "     z: " << resumes_[i].position.z << std::endl;
            ofs << "    " << "orientation:" << std::endl;
            ofs << "     x: " << resumes_[i].orientation.x << std::endl;
            ofs << "     y: " << resumes_[i].orientation.y << std::endl;
            ofs << "     z: " << resumes_[i].orientation.z << std::endl;
            ofs << "     w: " << resumes_[i].orientation.w << std::endl;
        }
        ofs.close();

        ROS_INFO_STREAM("write success");
        return true;
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
    ros::Publisher marker_description_pub_;
    std::vector<geometry_msgs::Point> waypoints_;
    geometry_msgs::PoseStamped finish_pose_;
    std::vector<geometry_msgs::Pose> suspends_;
    std::vector<geometry_msgs::Pose> resumes_;
 
    visualization_msgs::MarkerArray marker_description_;
    tf::TransformListener tf_listener_;
    int save_joy_button_;
    int suspend_size_;
    std::string filename_;
    std::string suspend_filename_;
    std::string world_frame_;
    std::string robot_frame_;
    ros::ServiceServer save_server_;

    bool fp_flag_;
    ros::Rate rate_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    interactive_markers::MenuHandler wp_menu_handler_;
    interactive_markers::MenuHandler sp_menu_handler_;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "pose_marker");
  WaypointsEditor w_edit;
  w_edit.run();

  return 0;
}
