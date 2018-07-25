#ifndef KR6_KINEMATICS_PLUGIN
#define KR6_KINEMATICS_PLUGIN

// MSG
#include <geometry_msgs/PoseStamped.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>

// Other
#include <arm_kinematics/KR6ArmKinematics.h>
#include <urdf/model.h>

namespace kr6_kinematics_plugin
{


class KR6KinematicsPlugin : public kinematics::KinematicsBase
{

public:

    /**
    *  @brief Default Ctor. and Dtor.
    */
    KR6KinematicsPlugin();
    virtual ~KR6KinematicsPlugin();

    /**
    * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
    * @param ik_pose the desired pose of the link
    * @param ik_seed_state an initial guess solution for the inverse kinematics
    * @param solution the solution vector
    * @param error_code an error code that encodes the reason for failure or success
    * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
    * @return True if a valid solution was found, false otherwise
    */
    bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                       const std::vector<double> &ik_seed_state,
                       std::vector<double> &solution,
                       moveit_msgs::MoveItErrorCodes &error_code,
                       const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const;

    /**
    * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
    * This particular method is intended for "searching" for a solutions by stepping through the redundancy
    * (or other numerical routines).
    * @param ik_pose the desired pose of the link
    * @param ik_seed_state an initial guess solution for the inverse kinematics
    * @param timeout The amount of time (in seconds) available to the solver
    * @param solution the solution vector
    * @param error_code an error code that encodes the reason for failure or success
    * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
    * @return True if a valid solution was found, false otherwise
    */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options =
                          kinematics::KinematicsQueryOptions()) const;

    /**
    * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
    * This particular method is intended for "searching" for a solutions by stepping through the redundancy
    * (or other numerical routines).
    * @param ik_pose the desired pose of the link
    * @param ik_seed_state an initial guess solution for the inverse kinematics
    * @param timeout The amount of time (in seconds) available to the solver
    * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
    * @param solution the solution vector
    * @param error_code an error code that encodes the reason for failure or success
    * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
    * @return True if a valid solution was found, false otherwise
    */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          const std::vector<double> &consistency_limits,
                          std::vector<double> &solution,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options =
                          kinematics::KinematicsQueryOptions()) const;

    /**
    * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
    * This particular method is intended for "searching" for a solutions by stepping through the redundancy
    * (or other numerical routines).
    * @param ik_pose the desired pose of the link
    * @param ik_seed_state an initial guess solution for the inverse kinematics
    * @param timeout The amount of time (in seconds) available to the solver
    * @param solution the solution vector
    * @param solution_callback A callback solution for the IK solution
    * @param error_code an error code that encodes the reason for failure or success
    * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
    * @return True if a valid solution was found, false otherwise
    */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options =
                          kinematics::KinematicsQueryOptions()) const;

    /**
    * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
    * This particular method is intended for "searching" for a solutions by stepping through the redundancy
    * (or other numerical routines).
    * @param ik_pose the desired pose of the link
    * @param ik_seed_state an initial guess solution for the inverse kinematics
    * @param timeout The amount of time (in seconds) available to the solver
    * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
    * @param solution the solution vector
    * @param solution_callback A callback solution for the IK solution
    * @param error_code an error code that encodes the reason for failure or success
    * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
    * @return True if a valid solution was found, false otherwise
    */
    bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                          const std::vector<double> &ik_seed_state,
                          double timeout,
                          const std::vector<double> &consistency_limits,
                          std::vector<double> &solution,
                          const IKCallbackFn &solution_callback,
                          moveit_msgs::MoveItErrorCodes &error_code,
                          const kinematics::KinematicsQueryOptions &options =
                          kinematics::KinematicsQueryOptions()) const;

    /**
    * @brief Given a set of joint angles and a set of links, compute their pose
    * @param link_names A set of links for which FK needs to be computed
    * @param joint_angles The state for which FK is being computed
    * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
    * @return True if a valid solution was found, false otherwise
    */
    bool getPositionFK(const std::vector<std::string> &link_names,
                       const std::vector<double> &joint_angles,
                       std::vector<geometry_msgs::Pose> &poses) const;


    /**
    * @brief  Initialization function for the kinematics, for use with kinematic chain IK solvers
    * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
    * For example, the name of the ROS parameter that contains the robot description;
    * @param group_name The group for which this solver is being configured
    * @param base_frame The base frame in which all input poses are expected.
    * This may (or may not) be the root frame of the chain that the solver operates on
    * @param tip_frame The tip of the chain
    * @param search_discretization The discretization of the search when the solver steps through the redundancy
    * @return True if initialization was successful, false otherwise
    */
    bool initialize(const std::string& robot_description,
                    const std::string& group_name,
                    const std::string& base_frame,
                    const std::string& tip_frame,
                    double search_discretization);

    /**
    * @brief  Return all the joint names in the order they are used internally
    */
    const std::vector<std::string> & getJointNames() const;

    /**
    * @brief  Return all the link names in the order they are represented internally
    */
    const std::vector<std::string> & getLinkNames() const;


private:

    bool extractKinematicData(const urdf::Model &robot_model,
                              const std::string &base_frame,
                              const std::string &tip_frame,
                              std::vector<std::string> &joint_names,
                              std::vector<std::string> &link_names,
                              std::vector<double> &lower_limits,
                              std::vector<double> &upper_limits) const;


    // Select one prior solution
    std::vector<double> selectIKSolution(const std::vector<JointValues> & solutions, const JointValues & initialState) const;

    Pose poseMsgToMatrix(const geometry_msgs::Pose & p) const;
    std::vector<double> configurationJvToMsg(const JointValues & jv) const;
    JointValues configurationMsgToJv(const std::vector<double> msg) const;


    /**
     * The joints on which the kinematics plugin is working.
     */
    std::vector<std::string> joint_names_;

    /**
     * The links in the kinematic chain between the base frame and the tip
     * frame (as provided to the @see KinematicsPlugin::initialize
     * function).
     */
    std::vector<std::string> link_names_;

    /**
     * The kinematics solver.
     */
    boost::shared_ptr<kr6_arm_kinematics::Kinematics> kinematics;

};
}

#endif // KR6_KINEMATICS_PLUGIN