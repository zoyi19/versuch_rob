#include "humanoid_interface/command/HumanoidHandTarget.h"

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <kuavo_msgs/endEffectorData.h>

namespace ocs2 {
namespace humanoid {

vector_t HandTargetTrajectoriesInteractiveMarker::lastEeState_ = vector_t::Zero(14);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HandTargetTrajectoriesInteractiveMarker::HandTargetTrajectoriesInteractiveMarker(::ros::NodeHandle& nodeHandle, uint8_t boxIndex, 
                                                                         const std::string& TopicPrefix, GoalPoseToTargetTrajectories goalPoseToTargetTrajectories)
    : server_("simple_marker_"+std::to_string(boxIndex)), goalPoseToTargetTrajectories_(std::move(goalPoseToTargetTrajectories)), boxIndex_(boxIndex) {

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(TopicPrefix + "_mpc_observation", 1, observationCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, TopicPrefix));

  auto eePoseCallback = [this](const kuavo_msgs::endEffectorData::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    kuavo_msgs::endEffectorData eeState = *msg;
    lastEeState_.resize(eeState.position.size());
    for(size_t i = 0; i < eeState.position.size(); i++){
      lastEeState_(i) = static_cast<scalar_t>(eeState.position[i]);
    }
    // std::cout << "lastEeState_:  " << lastEeState_.transpose() << "\n\n";
  };
  eePoseSub_ = nodeHandle.subscribe<kuavo_msgs::endEffectorData>("/humanoid_ee_State", 1, eePoseCallback);
  // create an interactive marker for our server
  menuHandler_.insert("Send target pose", boost::bind(&HandTargetTrajectoriesInteractiveMarker::processFeedback, this, _1));

  // create an interactive marker for our server
  auto interactiveMarker = createInteractiveMarker();

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server_.insert(interactiveMarker);  //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
  menuHandler_.apply(server_, interactiveMarker.name);

  // 'commit' changes and send to all clients
  server_.applyChanges();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
visualization_msgs::InteractiveMarker HandTargetTrajectoriesInteractiveMarker::createInteractiveMarker() const {
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = "odom";
  interactiveMarker.header.stamp = ros::Time::now();
  interactiveMarker.name = "Goal";
  interactiveMarker.scale = 0.2;
  interactiveMarker.description = "Right click to send command";
  interactiveMarker.pose.position.x = 0.52;
  interactiveMarker.pose.position.y = 0.2 - 0.4 * boxIndex_;
  interactiveMarker.pose.position.z = 1.0;

  // create a grey box marker
  const auto boxMarker = []() {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
  }();

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl boxControl;
  boxControl.always_visible = 1;
  boxControl.markers.push_back(boxMarker);
  boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  interactiveMarker.controls.push_back(boxControl);

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  interactiveMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(control);

  return interactiveMarker;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void HandTargetTrajectoriesInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  // Desired Base state trajectory
  const Eigen::Vector3d EePosition(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  const Eigen::Quaterniond EeOrientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                       feedback->pose.orientation.z);

  // get TargetTrajectories
  const auto targetTrajectories = goalPoseToTargetTrajectories_(EePosition, EeOrientation, 
                                                                latestObservation_, lastEeState_);

  // publish TargetTrajectories
  targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);

}

}   // namespace humanoid
}  // namespace ocs2
