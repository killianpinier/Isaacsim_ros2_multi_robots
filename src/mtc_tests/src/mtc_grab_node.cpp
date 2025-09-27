#include "mycobot_veggies_prep/mtc_grab_node.hpp"
#include "utils.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "globals.h"

//=============================================================================
// MTCStageFactory Implementation
//=============================================================================

MTCStageFactory::MTCStageFactory(const rclcpp::Node::SharedPtr& node,
                                 const RobotConfig& robot_config,
                                 const PlannerConfig& planner_config)
    : node_(node), robot_config_(robot_config), planner_config_(planner_config) {}

std::unique_ptr<mtc::stages::CurrentState> 
MTCStageFactory::createCurrentState(const std::string& name) const {
    return std::make_unique<mtc::stages::CurrentState>(name);
}

std::unique_ptr<mtc::stages::MoveTo> 
MTCStageFactory::createMoveTo(const std::string& name,
                              const std::shared_ptr<mtc::solvers::PlannerInterface>& planner,
                              const std::string& group,
                              const std::string& goal) const {
    auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
    stage->setGroup(group);
    stage->setGoal(goal);
    return stage;
}

std::unique_ptr<mtc::stages::Connect> 
MTCStageFactory::createConnect(const std::string& name,
                               const std::shared_ptr<mtc::solvers::PlannerInterface>& arm_planner,
                               const std::shared_ptr<mtc::solvers::PlannerInterface>& gripper_planner) const {
    mtc::stages::Connect::GroupPlannerVector planners;
    planners.push_back({robot_config_.arm_group_name, arm_planner});
    
    if (gripper_planner) {
        planners.push_back({robot_config_.gripper_group_name, gripper_planner});
    }
    
    auto stage = std::make_unique<mtc::stages::Connect>(name, planners);
    stage->setTimeout(planner_config_.connection_timeout);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    return stage;
}

std::unique_ptr<mtc::stages::MoveRelative> 
MTCStageFactory::createApproach(const std::string& name,
                                const std::shared_ptr<mtc::solvers::CartesianPath>& planner,
                                const geometry_msgs::msg::Vector3Stamped& direction,
                                double min_distance,
                                double max_distance) const {
    auto stage = std::make_unique<mtc::stages::MoveRelative>(name, planner);
    stage->properties().set("marker_ns", name);
    stage->properties().set("link", robot_config_.gripper_frame);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(min_distance, max_distance);
    stage->setDirection(direction);
    return stage;
}

std::unique_ptr<mtc::stages::ComputeIK> 
MTCStageFactory::createGraspGenerator(const std::string& object_name, const PickConfig& pick_config, mtc::Stage* monitored_stage) const {
    auto grasp_stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
    grasp_stage->properties().configureInitFrom(mtc::Stage::PARENT);
    grasp_stage->properties().set("marker_ns", "grasp_pose");
    grasp_stage->setPreGraspPose(pick_config.pre_grasp_pose);
    grasp_stage->setObject(object_name);
    grasp_stage->setAngleDelta(pick_config.grasp_angle_delta);
    
    if (monitored_stage) {
        grasp_stage->setMonitoredStage(monitored_stage);
    }
    
    auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp_stage));
    ik_wrapper->setMaxIKSolutions(pick_config.max_ik_solutions);
    ik_wrapper->setMinSolutionDistance(pick_config.min_solution_distance);
    ik_wrapper->setIKFrame(pick_config.grasp_frame_transform, robot_config_.gripper_frame);
    ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    
    return ik_wrapper;
}

std::unique_ptr<mtc::stages::ComputeIK> 
MTCStageFactory::createPoseGenerator(const geometry_msgs::msg::PoseStamped &pose, const PickConfig& pick_config, mtc::Stage* monitored_stage) const {
    auto grasp_stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
    grasp_stage->properties().configureInitFrom(mtc::Stage::PARENT);
    grasp_stage->properties().set("marker_ns", "grasp_pose");

    grasp_stage->setPose(pose);

    // grasp_stage->setPreGraspPose(pick_config.pre_grasp_pose);
    // grasp_stage->setObject(object_name);
    // grasp_stage->setAngleDelta(pick_config.grasp_angle_delta);
    
    if (monitored_stage) {
        grasp_stage->setMonitoredStage(monitored_stage);
    }

    std::cout << "grasp_frame_transform:\n" << pick_config.grasp_frame_transform.matrix() << std::endl;

    
    auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp_stage));
    ik_wrapper->setMaxIKSolutions(pick_config.max_ik_solutions);
    ik_wrapper->setMinSolutionDistance(pick_config.min_solution_distance);
    ik_wrapper->setIKFrame(pick_config.grasp_frame_transform, robot_config_.gripper_frame);
    //ik_wrapper->setIKFrame(robot_config_.gripper_frame);
    ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
    ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
    // auto boost_pose = ik_wrapper->properties().get("ik_frame");
    // geometry_msgs::msg::PoseStamped new_pose = boost::any_cast<geometry_msgs::msg::PoseStamped>(boost_pose);

    std::cout << "x = "  << pose.pose.position.x << std::endl;
    std::cout << "y = "  << pose.pose.position.y << std::endl;
    std::cout << "z = "  << pose.pose.position.z << std::endl;
    std::cout << "ox = " << pose.pose.orientation.x << std::endl;
    std::cout << "oy = " << pose.pose.orientation.y << std::endl;
    std::cout << "oz = " << pose.pose.orientation.z << std::endl;
    std::cout << "ow = " << pose.pose.orientation.w << std::endl;

    
    return ik_wrapper;
}

std::unique_ptr<mtc::stages::ComputeIK> MTCStageFactory::createPlaceGenerator(const std::string& object_name,
                                                                              const PlaceConfig& place_config,
                                                                              const geometry_msgs::msg::Pose place_pose,
                                                                              const double object_height,
                                                                              mtc::Stage* monitored_stage) const {
    auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
    stage->properties().set("marker_ns", "place_pose");
    stage->setObject(object_name);

    // Set target pose
    geometry_msgs::msg::PoseStamped target_pose_msg;
    target_pose_msg.header.frame_id = robot_config_.world_frame;
    target_pose_msg.pose = place_pose;
    target_pose_msg.pose.position.z += 0.5 * object_height;
    stage->setPose(target_pose_msg);
    
    if (monitored_stage) {
        stage->setMonitoredStage(monitored_stage);
    }

    auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
    wrapper->setMaxIKSolutions(place_config.max_ik_solutions);
    wrapper->setIKFrame(object_name);
    wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
    wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

    return wrapper;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene>
MTCStageFactory::createAllowCollision(const std::string& name,
                                      const std::string& first,
                                      const std::vector<std::string>& second,
                                      bool allow) const {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    stage->allowCollisions(first, second, allow);
    return stage;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene>
MTCStageFactory::createAttachObject(const std::string& name,
                                    const std::string& object) const {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    stage->attachObject(object, robot_config_.gripper_frame);
    return stage;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene> MTCStageFactory::createDetachObject(const std::string& name,
                                                                                      const std::string& object) const {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
    stage->detachObject(object, robot_config_.gripper_frame);
    return stage;
}


//=============================================================================
// PickPlaceTaskBuilder Implementation
//=============================================================================

PickPlaceTaskBuilder::PickPlaceTaskBuilder(const std::string& task_name,
                                           const rclcpp::Node::SharedPtr& node,
                                           const Config& config)
                                           // const RobotConfig& robot_config)
    : node_(node), config_(config) {
    task_.stages()->setName(task_name);
    task_.loadRobotModel(node_);
    
    // Set task properties
    task_.setProperty("group", config_.robot_config.arm_group_name);
    task_.setProperty("eef", config_.robot_config.gripper_group_name);
    task_.setProperty("ik_frame", config_.robot_config.gripper_frame);

    sampling_planner = createSamplingPlanner();
    interpolation_planner = createInterpolationPlanner();
    cartesian_planner = createCartesianPlanner();
    
    stage_factory_ = std::make_unique<MTCStageFactory>(node_, config_.robot_config, config_.planner_config);
}

PickPlaceTaskBuilder& PickPlaceTaskBuilder::addInitialStage() {
    // Add current state
    auto current_state = stage_factory_->createCurrentState("current");
    current_state_ptr_ = current_state.get();
    task_.add(std::move(current_state));
    
    return *this;
}

PickPlaceTaskBuilder& PickPlaceTaskBuilder::addPickSequence(const std::optional<geometry_msgs::msg::PoseStamped> &object_pose) {
    // Open gripper
    auto open_gripper = stage_factory_->createMoveTo("open gripper", interpolation_planner, config_.robot_config.gripper_group_name, config_.pick_config.pre_grasp_pose);
    task_.add(std::move(open_gripper));

    // Connect to pick
    auto connect = stage_factory_->createConnect("move to pick", sampling_planner, interpolation_planner);
    task_.add(std::move(connect));
    
    // Pick container
    auto pick_container = createPickContainer(object_pose);
    task_.add(std::move(pick_container));
    
    return *this;
}

PickPlaceTaskBuilder& PickPlaceTaskBuilder::addPlaceSequence() {
    auto connect = stage_factory_->createConnect("move to place", sampling_planner, interpolation_planner);
    task_.add(std::move(connect));

    auto place_container = createPlaceContainer();
    task_.add(std::move(place_container));

    return *this;
}


std::unique_ptr<mtc::SerialContainer> PickPlaceTaskBuilder::createPickContainer(const std::optional<geometry_msgs::msg::PoseStamped> &object_pose) {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task_.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
    
    // Approach
    geometry_msgs::msg::Vector3Stamped approach_direction;
    approach_direction.header.frame_id = config_.robot_config.gripper_frame;
    approach_direction.vector.z = 1.0;
    
    auto approach = stage_factory_->createApproach(
        "approach object", 
        cartesian_planner, 
        approach_direction,
        config_.pick_config.approach_min_distance,
        config_.pick_config.approach_max_distance);
    grasp->insert(std::move(approach));
    
    // Generate grasp pose
    std::unique_ptr<mtc::Stage> grasp_generator;
    if (object_pose) {
        grasp_generator = stage_factory_->createPoseGenerator(*object_pose, config_.pick_config, current_state_ptr_);
    } else {
        grasp_generator = stage_factory_->createGraspGenerator(config_.scene_config.object_name, config_.pick_config, current_state_ptr_);
    }
    grasp->insert(std::move(grasp_generator));

   std::vector<std::string> gripper_links = {
    "robot1_gripper_base", "robot1_gripper_left3", "robot1_gripper_left2", "robot1_gripper_left1",
    "robot1_gripper_right3", "robot1_gripper_right2", "robot1_gripper_right1"};

    // for (const std::string& w: links) {
    //     std::cout << w << ", ";
    // }

    // std::cout << std::endl;
    
    // Allow collision
    auto allow_collision = stage_factory_->createAllowCollision(
        "allow collision (gripper,object)",
        config_.scene_config.object_name,
        gripper_links,
            true);
    grasp->insert(std::move(allow_collision));
    
    // Close gripper
    auto close_gripper = stage_factory_->createMoveTo(
        "close gripper", interpolation_planner, 
        config_.robot_config.gripper_group_name, config_.pick_config.grasp_pose);
    grasp->insert(std::move(close_gripper));
    
    // Attach object
    auto attach = stage_factory_->createAttachObject("attach object", config_.scene_config.object_name);
    attach_object_stage_ = attach.get();
    grasp->insert(std::move(attach));
    
    // Lift
    geometry_msgs::msg::Vector3Stamped lift_direction;
    lift_direction.header.frame_id = config_.robot_config.world_frame;
    lift_direction.vector.z = 1.0;
    
    auto lift = stage_factory_->createApproach(
        "lift object", 
        cartesian_planner, 
        lift_direction,
        config_.pick_config.lift_min_distance,
        config_.pick_config.lift_max_distance);
    // auto lift = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    // lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    // lift->setMinMaxDistance(config_.pick_config.lift_min_distance, config_.pick_config.lift_max_distance);
    // lift->setIKFrame(config_.robot_config.gripper_frame);
    // lift->properties().set("marker_ns", "lift_object");
    // lift->setDirection(lift_direction);
    grasp->insert(std::move(lift));
    
    return grasp;
}

std::unique_ptr<mtc::SerialContainer> PickPlaceTaskBuilder::createPlaceContainer() {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task_.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // Lower object
    geometry_msgs::msg::Vector3Stamped lower_direction;
    lower_direction.header.frame_id = config_.robot_config.world_frame;
    lower_direction.vector.z = -1.0;
    auto lower = stage_factory_->createApproach(
        "lower object",
        cartesian_planner,
        lower_direction,
        config_.place_config.lower_object_min_distance,
        config_.place_config.lower_object_max_distance
    );
    place->insert(std::move(lower));

    // Generate place pose
    auto place_generator = stage_factory_->createPlaceGenerator(
        config_.scene_config.object_name,
        config_.place_config,
        utils::vectorToPose(config_.scene_config.place_pose),
        config_.scene_config.object_dimensions[0],
        current_state_ptr_
    );
    place->insert(std::move(place_generator));

    // Open gripper
    auto open_gripper_stage = stage_factory_->createMoveTo("open gripper", interpolation_planner, config_.robot_config.gripper_group_name, config_.place_config.post_place_pose);
    place->insert(std::move(open_gripper_stage));

    // Forbid collisions
    auto forbid_collision_stage = stage_factory_->createAllowCollision(
        "forbid collision (gripper, object)", 
        config_.scene_config.object_name,
        task_.getRobotModel()->getJointModelGroup(config_.robot_config.gripper_group_name)->getLinkModelNamesWithCollisionGeometry(),
        false
    );
    place->insert(std::move(forbid_collision_stage));

    auto detach_object_stage = stage_factory_->createDetachObject("detach object", config_.scene_config.object_name);
    place->insert(std::move(detach_object_stage));

    geometry_msgs::msg::Vector3Stamped retreat_direction;
    retreat_direction.header.frame_id = config_.robot_config.world_frame;
    retreat_direction.vector.z = -1.0;
    auto retreat_stage = stage_factory_->createApproach(
        "retreat",
        cartesian_planner,
        retreat_direction,
        config_.place_config.retreat_min_distance,
        config_.place_config.retreat_max_distance
    );
    place->insert(std::move(retreat_stage));

    return place;
}

std::shared_ptr<mtc::solvers::PipelinePlanner> PickPlaceTaskBuilder::createSamplingPlanner() const {
    std::unordered_map<std::string, std::string> planner_map = {
        {"ompl", config_.robot_config.arm_group_name + "[" + config_.planner_config.ompl_planner_id + "]"}
    };
    return std::make_shared<mtc::solvers::PipelinePlanner>(node_, planner_map);
}

std::shared_ptr<mtc::solvers::JointInterpolationPlanner> PickPlaceTaskBuilder::createInterpolationPlanner() const {
    return std::make_shared<mtc::solvers::JointInterpolationPlanner>();
}

std::shared_ptr<mtc::solvers::CartesianPath> PickPlaceTaskBuilder::createCartesianPlanner() const {
    auto planner = std::make_shared<mtc::solvers::CartesianPath>();
    planner->setMaxVelocityScalingFactor(config_.planner_config.cartesian_max_velocity_scaling);
    planner->setMaxAccelerationScalingFactor(config_.planner_config.cartesian_max_acceleration_scaling);
    planner->setStepSize(config_.planner_config.cartesian_step_size);
    return planner;
}

PickPlaceTaskBuilder& PickPlaceTaskBuilder::addReturnHome(const std::string& home_position) {
    auto stage = stage_factory_->createMoveTo(
        "return home", sampling_planner, config_.robot_config.arm_group_name, home_position);
    task_.add(std::move(stage));
    return *this;
}

mtc::Task PickPlaceTaskBuilder::build() {
    return std::move(task_);
}

//=============================================================================
// MTCGrabNode Implementation
//=============================================================================

MTCGrabNode::MTCGrabNode(const rclcpp::NodeOptions& options)
    : Node("mtc_grab_node", options) {
    loadParameters();
    get_frustrum_client = std::make_shared<GetFrustrumClient>();
}

void MTCGrabNode::loadParameters() {
    config_ = Config();
    config_.robot_config = loadRobotConfig();
    config_.planner_config = loadPlannerConfig();
    config_.pick_config = loadPickConfig();
    config_.place_config = loadPlaceConfig();
    config_.scene_config = loadSceneConfig();
}

RobotConfig MTCGrabNode::loadRobotConfig() {
    RobotConfig config;
    
    // Load from parameters
    this->declare_parameter("robot.arm_group_name", config.arm_group_name);
    this->declare_parameter("robot.gripper_group_name", config.gripper_group_name);
    this->declare_parameter("robot.gripper_frame", config.gripper_frame);
    this->declare_parameter("robot.base_frame", config.base_frame);
    this->declare_parameter("robot.world_frame", config.world_frame);
    
    config.arm_group_name = this->get_parameter("robot.arm_group_name").as_string();
    config.gripper_group_name = this->get_parameter("robot.gripper_group_name").as_string();
    config.gripper_frame = this->get_parameter("robot.gripper_frame").as_string();
    config.base_frame = this->get_parameter("robot.base_frame").as_string();
    config.world_frame = this->get_parameter("robot.world_frame").as_string();
    
    return config;
}

PlannerConfig MTCGrabNode::loadPlannerConfig() {
    PlannerConfig config;
    
    this->declare_parameter("planner.ompl_planner_id", config.ompl_planner_id);
    this->declare_parameter("planner.cartesian_max_velocity", config.cartesian_max_velocity_scaling);
    this->declare_parameter("planner.cartesian_max_acceleration", config.cartesian_max_acceleration_scaling);
    this->declare_parameter("planner.cartesian_step_size", config.cartesian_step_size);
    this->declare_parameter("planner.connection_timeout", config.connection_timeout);
    
    config.ompl_planner_id = this->get_parameter("planner.ompl_planner_id").as_string();
    config.cartesian_max_velocity_scaling = this->get_parameter("planner.cartesian_max_velocity").as_double();
    config.cartesian_max_acceleration_scaling = this->get_parameter("planner.cartesian_max_acceleration").as_double();
    config.cartesian_step_size = this->get_parameter("planner.cartesian_step_size").as_double();
    config.connection_timeout = this->get_parameter("planner.connection_timeout").as_double();
    
    return config;
}

PickConfig MTCGrabNode::loadPickConfig() {
    PickConfig config;
    
    this->declare_parameter("pick.approach_min_distance", config.approach_min_distance);
    this->declare_parameter("pick.approach_max_distance", config.approach_max_distance);
    this->declare_parameter("pick.grasp_angle_delta", config.grasp_angle_delta);
    this->declare_parameter("pick.pre_grasp_pose", config.pre_grasp_pose);
    this->declare_parameter("pick.grasp_pose", config.grasp_pose);
    this->declare_parameter("pick.max_ik_solutions", static_cast<int>(config.max_ik_solutions));
    this->declare_parameter("pick.min_solution_distance", config.min_solution_distance);
    this->declare_parameter("pick.lift_min_distance", config.lift_min_distance);
    this->declare_parameter("pick.lift_max_distance", config.lift_max_distance);
    
    // Load transform parameters
    this->declare_parameter("pick.grasp_transform.translation", config.translation);
    this->declare_parameter("pick.grasp_transform.rotation1", config.rotation1);
    this->declare_parameter("pick.grasp_transform.rotation2", config.rotation2);
    this->declare_parameter("pick.grasp_transform.rotation3", config.rotation3);
    
    auto translation = this->get_parameter("pick.grasp_transform.translation").as_double_array();
    auto rotation1 = this->get_parameter("pick.grasp_transform.rotation1").as_double_array();
    auto rotation2 = this->get_parameter("pick.grasp_transform.rotation2").as_double_array();
    auto rotation3 = this->get_parameter("pick.grasp_transform.rotation3").as_double_array();
    
    config.grasp_frame_transform = Eigen::Translation3d(translation[0], translation[1], translation[2]) *
                                   Eigen::AngleAxisd(rotation3[0], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(rotation3[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rotation3[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(rotation2[0], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(rotation2[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rotation2[2], Eigen::Vector3d::UnitZ()) *
                                   Eigen::AngleAxisd(rotation1[0], Eigen::Vector3d::UnitX()) *
                                   Eigen::AngleAxisd(rotation1[1], Eigen::Vector3d::UnitY()) *
                                   Eigen::AngleAxisd(rotation1[2], Eigen::Vector3d::UnitZ());
    
    return config;
}

PlaceConfig MTCGrabNode::loadPlaceConfig() {
    PlaceConfig config;
    
    this->declare_parameter("place.lower_object_min_distance", config.lower_object_min_distance);
    this->declare_parameter("place.lower_object_max_distance", config.lower_object_max_distance);

    this->declare_parameter("place.retreat_min_distance", config.retreat_min_distance);
    this->declare_parameter("place.retreat_max_distance", config.retreat_max_distance);

    this->declare_parameter("place.max_ik_solutions", static_cast<int>(config.max_ik_solutions));
    this->declare_parameter("place.min_solution_distance", config.min_solution_distance);

    config.lower_object_min_distance = this->get_parameter("place.lower_object_min_distance").as_double();
    config.lower_object_max_distance = this->get_parameter("place.lower_object_max_distance").as_double();

    config.retreat_min_distance = this->get_parameter("place.retreat_min_distance").as_double();
    config.retreat_max_distance = this->get_parameter("place.retreat_max_distance").as_double();

    config.max_ik_solutions          = this->get_parameter("place.max_ik_solutions").as_int();
    config.min_solution_distance     = this->get_parameter("place.min_solution_distance").as_double();

    return config;
}

SceneConfig MTCGrabNode::loadSceneConfig() {
    SceneConfig config;
    
    this->declare_parameter("scene.object_name", config.object_name);
    this->declare_parameter("scene.object_dimensions", config.object_dimensions);
    this->declare_parameter("scene.object_pose", config.object_pose);
    this->declare_parameter("scene.place_pose", config.place_pose);
    
    config.object_name = this->get_parameter("scene.object_name").as_string();
    // config.object_dimensions = this->get_parameter("scene.object_dimensions");
    // config.object_pose = this->get_parameter("scene.object_pose");
    // config.place_pose = this->get_parameter("scene.place_pose");
    
    return config;
}

void MTCGrabNode::setupPlanningScene() {
    // moveit_msgs::msg::CollisionObject object;
    // object.id = "object";
    // object.header.frame_id = config_.robot_config.base_frame;
    // object.primitives.resize(1);
    // object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // object.primitives[0].dimensions = {constants::cylinder_height, constants::cylinder_radius};
    
    // geometry_msgs::msg::Pose pose;
    // pose.position.x     = constants::cylinder_x;
    // pose.position.y     = constants::cylinder_y;
    // pose.position.z     = constants::cylinder_z;
    // pose.orientation.x  = constants::cylinder_ox;
    // pose.orientation.y  = constants::cylinder_oy;
    // pose.orientation.z  = constants::cylinder_oz;
    // pose.orientation.w  = constants::cylinder_ow;
    // object.pose = pose;
    
    // addObject(object);

    auto object_name = config_.scene_config.object_name;

    RCLCPP_INFO(this->get_logger(), "Sending GetFrustrum service request...");
    auto response = get_frustrum_client->call_service();
    RCLCPP_INFO(this->get_logger(), "Service call to the GetFrustrum service completed.");

    geometry_msgs::msg::Pose object_pose = response.pose;
    double min_dist = response.min_dist;
    double max_dist = response.max_dist;

    moveit_msgs::msg::CollisionObject object;
    object.id = object_name;
    object.header.frame_id = "world";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { std::abs(max_dist - min_dist), response.radius_2 };
    object.pose = object_pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
    
}

void MTCGrabNode::addObject(const moveit_msgs::msg::CollisionObject& object) {
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCGrabNode::clearPlanningScene() {
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.removeCollisionObjects(psi.getKnownObjectNames());
}

mtc::Task MTCGrabNode::createPickTask(const std::optional<geometry_msgs::msg::PoseStamped> &object_pose) {
    PickPlaceTaskBuilder builder("pick_task", shared_from_this(), config_);
    
    auto task = builder
        // .setPlannerConfig(planner_config_)
        // .setPickConfig(pick_config_)
        .addInitialStage()
        .addPickSequence(object_pose)
        // .addReturnHome(config_.robot_config.home)
        .build();

    temp_stage_ = builder.getAttachObjectStagePtr();

    return task;
}

mtc::Task MTCGrabNode::createPlaceTask() {
    PickPlaceTaskBuilder builder("place_task", shared_from_this(), config_);
    // builder.setAttachObjectStagePtr(temp_stage_);

    // .setPlannerConfig(planner_config_)
    // .setPlaceConfig(place_config_)
    return builder
        .addInitialStage()
        .addPlaceSequence()
        .addReturnHome(config_.robot_config.home)
        .build();
}

mtc::Task MTCGrabNode::createPickCylinderTask() {
    PickPlaceTaskBuilder builder("pick_cylinder_task", shared_from_this(), config_);
    // builder.setAttachObjectStagePtr(temp_stage_);

    // .setPlannerConfig(planner_config_)
    // .setPlaceConfig(place_config_)
    return builder
        .addInitialStage()
        .addPlaceSequence()
        .addReturnHome(config_.robot_config.home)
        .build();
}

bool MTCGrabNode::executePickTask(const std::optional<geometry_msgs::msg::PoseStamped> &object_pose) {
    RCLCPP_INFO(this->get_logger(), "Starting pick task");
    
    auto task = createPickTask(object_pose);
    
    try {
        task.init();
        RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
        return false;
    }
    
    if (!planTask(task)) {
        RCLCPP_ERROR(this->get_logger(), "Task planning failed");
        return false;
    }
    
    if (!executeTask(task)) {
        RCLCPP_ERROR(this->get_logger(), "Task execution failed");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Pick task completed successfully");
    return true;
}

bool MTCGrabNode::executePlaceTask() {
    RCLCPP_INFO(this->get_logger(), "Starting place task");
    
    auto task = createPlaceTask();
    
    try {
        task.init();
        RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
    }
    catch (mtc::InitStageException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
        return false;
    }
    
    if (!planTask(task)) {
        RCLCPP_ERROR(this->get_logger(), "Task planning failed");
        return false;
    }
    
    if (!executeTask(task)) {
        RCLCPP_ERROR(this->get_logger(), "Task execution failed");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Place task completed successfully");
    return true;
}

bool MTCGrabNode::planTask(mtc::Task& task, int max_solutions) {
    try {
        RCLCPP_INFO(this->get_logger(), "Starting task planning");
        return task.plan(max_solutions) == moveit::core::MoveItErrorCode::SUCCESS;
    } catch (mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
        return false;
    } catch (std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return false;
    }
}

bool MTCGrabNode::executeTask(mtc::Task& task) {
    try {
        auto result = task.execute(*task.solutions().front());
        return result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    } catch (std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return false;
    }
}

void MTCGrabNode::publishTaskSolution(mtc::Task& task) {
    task.introspection().publishSolution(*task.solutions().front());
}

// geometry_msgs::msg::PoseStamped get_cylinder_grasp_pose(geometry_msgs::msg::Pose object_pose) {
//     Eigen::Quaterniond q(constants::cylinder_ow, constants::cylinder_ox, constants::cylinder_oy, constants::cylinder_oz);

//     Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
//     grasp_pose.linear() = q.normalized().toRotationMatrix();
//     grasp_pose.translation() << constants::cylinder_x, constants::cylinder_y, constants::cylinder_z;
//     //grasp_pose.translation() << -0.1, constants::cylinder_y, constants::cylinder_z;

//     std::string world_frame = "world";
//     return utils::toPoseStamped(grasp_pose, world_frame);
// } 

geometry_msgs::msg::PoseStamped get_cylinder_grasp_pose(std::string object_name) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::string cylinder_id = object_name;  // ID used when adding the cylinder

    // Get all collision objects
    auto collision_objects = planning_scene_interface.getObjects({cylinder_id});

    if (collision_objects.find(cylinder_id) == collision_objects.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("moveit_example"), "Cylinder not found in planning scene!");
        throw std::runtime_error("Error: no cylinder found in planning scene");
    }

    // Extract the cylinder collision object
    const moveit_msgs::msg::CollisionObject& cylinder = collision_objects[cylinder_id];
    geometry_msgs::msg::Pose cylinder_pose = cylinder.pose; //primitive_poses[0];

    std::cout << "x = "  << cylinder_pose.position.x << std::endl;
    std::cout << "y = "  << cylinder_pose.position.y << std::endl;
    std::cout << "z = "  << cylinder_pose.position.z << std::endl;
    std::cout << "ox = " << cylinder_pose.orientation.x << std::endl;
    std::cout << "oy = " << cylinder_pose.orientation.y << std::endl;
    std::cout << "oz = " << cylinder_pose.orientation.z << std::endl;
    std::cout << "ow = " << cylinder_pose.orientation.w << std::endl;

    Eigen::Vector3d center_pos(cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
    Eigen::Quaterniond cylinder_quat(cylinder_pose.orientation.w, cylinder_pose.orientation.x, cylinder_pose.orientation.y, cylinder_pose.orientation.z);
    
    // Extract z-axis from quaternion (main cylinder axis)
    Eigen::Matrix3d rot = cylinder_quat.toRotationMatrix();
    Eigen::Vector3d z_cyl = rot.col(2);  // z-axis of cylinder frame

    // Now construct the desired frame (x = world up, y horizontal, z = cylinder axis)
    Eigen::Vector3d z_world(0.0, 0.0, 1.0);  // World up (vertical)
    Eigen::Vector3d y_cyl = z_cyl.cross(z_world).normalized();  // Horizontal
    Eigen::Vector3d x_cyl = y_cyl.cross(z_cyl).normalized();    // Should align with [0,0,1]

    // Build rotation matrix for consistent frame
    Eigen::Matrix3d new_rot;
    new_rot.col(0) = x_cyl;
    new_rot.col(1) = y_cyl;
    new_rot.col(2) = z_cyl;
    Eigen::Quaterniond new_quat(new_rot);

    // Get dimensions (for CYLINDER: [0] = height, [1] = radius)
    double height = cylinder.primitives[0].dimensions[0];
    double radius = cylinder.primitives[0].dimensions[1];

    // Compute tip position (e.g., positive z tip)
    Eigen::Vector3d tip_pos = center_pos + (height / 2.0 - 0.01) * z_cyl;

    cylinder_pose.position.x    = tip_pos.x();
    cylinder_pose.position.y    = tip_pos.y();
    cylinder_pose.position.z    = tip_pos.z();
    cylinder_pose.orientation.x = new_quat.x();
    cylinder_pose.orientation.y = new_quat.y();
    cylinder_pose.orientation.z = new_quat.z();
    cylinder_pose.orientation.w = new_quat.w();

    std::string world_frame = "world";
    return utils::toPoseStamped(cylinder_pose, world_frame);


    // Eigen::Quaterniond q(constants::cylinder_ow, constants::cylinder_ox, constants::cylinder_oy, constants::cylinder_oz);

    // Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
    // grasp_pose.linear() = q.normalized().toRotationMatrix();
    // grasp_pose.translation() << constants::cylinder_x, constants::cylinder_y, constants::cylinder_z;
    // //grasp_pose.translation() << -0.1, constants::cylinder_y, constants::cylinder_z;

    // std::string world_frame = "world";
    // return utils::toPoseStamped(grasp_pose, world_frame);
} 


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    int ret = 0;

    // Set up node options
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    // Create the MTC task node
    auto mtc_task_node = std::make_shared<MTCGrabNode>(options);

    // Set up a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mtc_task_node);

    auto spin_thread = std::make_unique<std::thread>([&executor]() {
        std::cout << "=== STDOUT: Executor thread started ===" << std::endl;
        executor.spin();
        std::cout << "=== STDOUT: Executor thread ended ===" << std::endl;
    });

    try {
        // Set up the planning scene and execute the task
        try {
            RCLCPP_INFO(mtc_task_node->get_logger(), "Setting up planning scene");
            mtc_task_node->setupPlanningScene();

            std::optional<geometry_msgs::msg::PoseStamped> opt_pose = get_cylinder_grasp_pose("object");
            mtc_task_node->executePickTask(opt_pose);
            // mtc_task_node->executePickTask(std::nullopt);

            // RCLCPP_INFO(mtc_task_node->get_logger(), "Executing task");
            // mtc_task_node->doTask();
            // RCLCPP_INFO(mtc_task_node->get_logger(), "Task execution completed. Keeping node alive for visualization. Press Ctrl+C to exit.");

            // // Keep the node running until Ctrl+C is pressed
            // executor.spin();
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(mtc_task_node->get_logger(), "Runtime error occurred: %s", e.what());
            ret = 1;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(mtc_task_node->get_logger(), "An error occurred: %s", e.what());
            ret = 1;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error during node setup: %s", e.what());
        ret = 1;
    }

    executor.cancel();
    if (spin_thread->joinable()) {
        spin_thread->join();
    }
    
    rclcpp::shutdown();
    return ret;
}
