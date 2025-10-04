#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace mtc = moveit::task_constructor;

geometry_msgs::msg::PoseStamped get_cylinder_grasp_pose() {
    geometry_msgs::msg::Pose pose;
    // pose.position.x = 0.455;
    // pose.position.y = 0.05;
    // pose.position.z = 0.012;
    pose.position.x = 0.35;
    pose.position.y = 0.29;
    pose.position.z = 0.217;

    pose.orientation.w = 0.7071;
    pose.orientation.x = -0.7071;
    pose.orientation.y = -0.7071;
    pose.orientation.z = -0.7071;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "World";
    pose_stamped.pose = pose;
    return pose_stamped;
}


class MTCTaskNode : public rclcpp::Node {
    public:
    MTCTaskNode(const rclcpp::NodeOptions& options,
                std::string arm_group_name_ = "panda_arm",
                std::string gripper_group_name_ = "hand",
                std::string gripper_frame_ = "panda1_panda_hand");
    void doTask();
    void setupPlanningScene();

    private:
    mtc::Task createTask();
    mtc::Task task_;
    const std::string arm_group_name;
    const std::string gripper_group_name;
    const std::string gripper_frame;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options,
                        std::string arm_group_name_,
                        std::string gripper_group_name_,
                        std::string gripper_frame_) 
: Node("mtc_node", options),
    arm_group_name(std::move(arm_group_name_)),
    gripper_group_name(std::move(gripper_group_name_)),
    gripper_frame(std::move(gripper_frame_)) {}

void MTCTaskNode::setupPlanningScene() {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "World";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.16, 0.02 }; 
    // object.primitives[0].dimensions = { 0.02, 0.02, 0.35 }; // Cube

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.45; pose.position.y = -0.08; pose.position.z = 0.015;
    pose.orientation.w = 0.7071;
    pose.orientation.x = 0.7071;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCTaskNode::doTask() {
    task_ = createTask();

    try {
        task_.init();
        RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR(this->get_logger(), "InitStageException: %s", e.what());
        RCLCPP_ERROR(this->get_logger(), "Failed stage: Unknown (add stage-specific logging)");
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
        return;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected exception: %s", e.what());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting task planning");

    try {
        if (!task_.plan(5)) {
            RCLCPP_ERROR(this->get_logger(), "Task planning failed");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Task planning succeeded");
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR(this->get_logger(), "InitStageException during planning: %s", e.what());
        RCLCPP_ERROR_STREAM(this->get_logger(), e);
        RCLCPP_ERROR(this->get_logger(), "InitStageException test test planning: %s", e.what());
        // Log all stage names to identify the failing one
        // for (const auto& stage : *task_.stages()) {
        //     RCLCPP_INFO(this->get_logger(), "Stage in pipeline: %s", stage->name().c_str());
        // }
        return;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected planning exception: %s", e.what());
        return;
    }

    task_.introspection().publishSolution(*task_.solutions().front());

    try {
        auto result = task_.execute(*task_.solutions().front());

        RCLCPP_INFO(this->get_logger(), "Updated planning scene with final robot state");
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Task execution failed");
            return;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error while executing plan: %s", e.what());
        return;
    }

    // Update move_group's state
    // moveit::planning_interface::PlanningSceneInterface psi;
    // moveit_msgs::msg::PlanningScene scene;
    // psi.applyPlanningScene(scene);

    return;
}

mtc::Task MTCTaskNode::createTask() {
    mtc::Task task;
    task.stages()->setName("pick_place_task");
    task.loadRobotModel(shared_from_this());

    // auto controller_names = std::vector<std::string>{"/panda1/panda_arm_controller", "/panda1/panda_hand_controller"};

    task.setProperty("group", arm_group_name);
    task.setProperty("eef", gripper_group_name);
    task.setProperty("ik_frame", gripper_frame);
    // task.setProperty("trajectory_execution_info",
    //     mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));

    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    // current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));
    
    std::unordered_map<std::string, std::string> ompl_map_arm = {
        {"ompl", arm_group_name + "[RRTConnectkConfigDefault]"}
    };
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this(), ompl_map_arm);
    sampling_planner->setMaxVelocityScalingFactor(0.25);
    sampling_planner->setMaxAccelerationScalingFactor(0.25);

    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage_open_gripper->setGroup(gripper_group_name);
    stage_open_gripper->setGoal("open");
    // stage_open_gripper->properties().set("trajectory_execution_info", mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
    current_state_ptr = stage_open_gripper.get();
    task.add(std::move(stage_open_gripper));

    auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{ 
            { arm_group_name, sampling_planner },
            // {gripper_group_name, interpolation_planner} 
        });
    stage_move_to_pick->setTimeout(5.0);
    stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_pick));

    mtc::Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

    geometry_msgs::msg::PoseStamped pose_stamped = get_cylinder_grasp_pose();

    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                                { "eef", "group", "ik_frame" });

        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", gripper_frame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            // stage->properties().set("trajectory_execution_info",
            //     mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
            stage->setMinMaxDistance(0.05, 0.15);

            // Set gripper forward direction
            geometry_msgs::msg::Vector3Stamped vec;
            vec.header.frame_id = gripper_frame;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        // {
        //     // Sample grasp pose
        //     auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
        //     stage->properties().configureInitFrom(mtc::Stage::PARENT);
        //     stage->properties().set("marker_ns", "grasp_pose");
        //     stage->setPreGraspPose("open");
        //     stage->setObject("object");
        //     stage->setAngleDelta(0.1309);
        //     stage->setMonitoredStage(current_state_ptr);  // Hook into current state

        //     Eigen::Isometry3d grasp_frame_transform = Eigen::Translation3d(0.0, 0.0, 0.1034) *
        //                                               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
        //                                               Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitY()) *
        //                                               Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

        //     // Compute IK
        //     auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
        //     wrapper->setMaxIKSolutions(10);
        //     wrapper->setMinSolutionDistance(0.8);
        //     wrapper->setIKFrame(grasp_frame_transform, gripper_frame);
        //     wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        //     wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
        //     grasp->insert(std::move(wrapper));
        // }

        {
            auto grasp_stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
            grasp_stage->properties().configureInitFrom(mtc::Stage::PARENT);
            grasp_stage->properties().set("marker_ns", "grasp_pose");

            grasp_stage->setPose(pose_stamped);
            
            if (current_state_ptr) {
                grasp_stage->setMonitoredStage(current_state_ptr);
            }

            // std::cout << "grasp_frame_transform:\n" << pick_config.grasp_frame_transform.matrix() << std::endl;

            Eigen::Isometry3d grasp_frame_transform = Eigen::Translation3d(0.0, 0.0, 0.0584) *
                                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
                                                    //   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                                    //   Eigen::AngleAxisd(1.5708, Eigen::Vector3d::UnitY()) *
                                                    //   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
                                                    
            
            auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp_stage));
            ik_wrapper->setMaxIKSolutions(8);
            ik_wrapper->setMinSolutionDistance(0.2);
            ik_wrapper->setIKFrame(grasp_frame_transform, gripper_frame);
            //ik_wrapper->setIKFrame(robot_config_.gripper_frame);
            ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
            ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
            // auto boost_pose = ik_wrapper->properties().get("ik_frame");
            // geometry_msgs::msg::PoseStamped new_pose = boost::any_cast<geometry_msgs::msg::PoseStamped>(boost_pose);
            grasp->insert(std::move(ik_wrapper));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
            stage->allowCollisions("object",
                                    task.getRobotModel()
                                        ->getJointModelGroup(gripper_group_name)
                                        ->getLinkModelNamesWithCollisionGeometry(),
                                    true);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
            stage->setGroup(gripper_group_name);
            stage->setGoal("closed");
            // stage->setTimeout(5.0);
            grasp->insert(std::move(stage));
        }

        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
            stage->attachObject("object", gripper_frame);
            attach_object_stage = stage.get();
            grasp->insert(std::move(stage));
        }

        // {
        //     auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
        //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        //     stage->setMinMaxDistance(0.1, 0.3);
        //     stage->setIKFrame(gripper_frame);
        //     stage->properties().set("marker_ns", "lift_object");

        //     // Set upward direction
        //     geometry_msgs::msg::Vector3Stamped vec;
        //     vec.header.frame_id = "World";
        //     vec.header.frame_id = "World";
        //     vec.vector.z = 1.0;
        //     stage->setDirection(vec);
        //     grasp->insert(std::move(stage));
        // }

        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("set object peeling pose", cartesian_planner);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });

            geometry_msgs::msg::PoseStamped pose = pose_stamped;
            Eigen::Quaterniond current_quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

            double angle = - M_PI / 4.0;
            Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitY());
            Eigen::Quaterniond rotation_quat(rotation);

            Eigen::Quaterniond new_quat = current_quat * rotation_quat;
            new_quat.normalize();

            pose.pose.position.z += 0.16;
            pose.pose.orientation.w = new_quat.w();
            pose.pose.orientation.x = new_quat.x();
            pose.pose.orientation.y = new_quat.y();
            pose.pose.orientation.z = new_quat.z();

            stage->setGoal(pose);
            stage->setGroup(arm_group_name);
            stage->setIKFrame(gripper_frame);
            // stage->setTimeout(5.0);
            grasp->insert(std::move(stage));
        }

        task.add(std::move(grasp));
    }

    // {
    //     auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    //     stage->properties().set("trajectory_execution_info",
    //                 mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
    //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    //     stage->setGoal("ready");
    //     task.add(std::move(stage));
    // }

    return task;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;

    auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
        executor.add_node(mtc_task_node);
        executor.spin();
        executor.remove_node(mtc_task_node);
    });

    mtc_task_node->setupPlanningScene();
    mtc_task_node->doTask();
    spin_thread->join();

    rclcpp::shutdown();
    return 0;
}