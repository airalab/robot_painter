// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <kuka_manipulation_moveit/KukaMoveit.hpp>

// Boost
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

// Terminal
#include <termios.h>

// Filesystem
#include <fstream>

// TF ??
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/utils.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// MSG
#include <geometry_msgs/PoseStamped.h>


#define FILE_NAME "data.csv"
typedef std::vector<double> JointValue;

// utils
int getch();

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "tool_calibration");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    JointValue jv;
    std::vector<JointValue> configurations;

    // Set logger level to debug
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    std::string path = ros::package::getPath("local_task_planner") + "/" + std::string(FILE_NAME);

    if (!boost::filesystem::exists(path)) {
        KukaMoveit manipulator("manipulator");

        boost::filesystem::path file{path};
        boost::filesystem::ofstream ofs{file};
        std::cout << "\n\n";
        ROS_INFO("Writing data to file: %s", path.c_str());
        ROS_INFO("To record current configuration press SPACE");

        ofs << "jv0, jv1, jv2, jv3, jv4, jv5\n";

        while (getch() == 32 && ros::ok()) {
            jv = manipulator.getMoveGroup()->getCurrentJointValues();

            ofs << jv[0];
            for (size_t i = 1; i < 6; ++i)
                ofs << ", " << jv[i];
            ofs << "\n";
            configurations.push_back(jv);

            ROS_INFO_STREAM("JV: " << jv[0] << ", " << jv[1] << ", " << jv[2] << ", " << jv[3] << ", " << jv[4] << ", " << jv[5]);
        }
    } else {
        std::ifstream file(path);
        std::string line;
        std::cout << "\n\n";
        ROS_INFO("Read data from file: %s", path.c_str());

        // Parse first line
        std::getline(file, line);

        // Parse other
        while (std::getline(file, line)) {
            std::vector<std::string> cont;
            jv.resize(0);
            ROS_DEBUG(">>: %s", line.c_str());
            boost::split(cont, line, boost::is_any_of(","));
            for (size_t i = 0; i < cont.size(); ++i) {
                jv.push_back(std::stod(cont[i]));
            }
            configurations.push_back(jv);
        }
    }

    // Calculation
    // TODO Test list squares method
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const Eigen::Affine3d & end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
    Eigen::Vector3d FK, FK1;
    Eigen::Matrix<double,3,3> R, R1;

    ROS_DEBUG(">> [%f, %f, %f, %f, %f, %f]", configurations[0][0], configurations[0][1], configurations[0][2], configurations[0][3], configurations[0][4], configurations[0][5]);
    kinematic_state->setJointGroupPositions("manipulator", configurations[0]);
    kinematic_state->updateLinkTransforms();
    FK1 = end_effector_state.translation();
    R1 = end_effector_state.linear();

    ROS_DEBUG(">> [%f, %f, %f, %f, %f, %f]", configurations[0][0], configurations[0][1], configurations[0][2], configurations[0][3], configurations[0][4], configurations[0][5]);
    kinematic_state->setJointGroupPositions("manipulator", configurations[0]);
    kinematic_state->updateLinkTransforms();
    FK = FK1 - end_effector_state.translation();
    R = R1 - end_effector_state.linear();

    for (size_t i = 2; i < configurations.size(); ++i) {
        ROS_DEBUG(">> [%f, %f, %f, %f, %f, %f]", configurations[i][0], configurations[i][1], configurations[i][2], configurations[i][3], configurations[i][4], configurations[i][5]);
        kinematic_state->setJointGroupPositions("manipulator", configurations[i]);
        kinematic_state->updateLinkTransforms();
        FK += FK1 - end_effector_state.translation();
        R += R1 - end_effector_state.linear();
        // std::cout << end_effector_state.matrix() << std::endl;
        // std::cout << end_effector_state.linear() << std::endl;
        // std::cout << end_effector_state.translation() << std::endl;
    }

    ROS_INFO_STREAM("Result point:\n" << -R.inverse()*FK);
}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);               // save old settings
    newt = oldt;
    newt.c_lflag &=~ (ICANON | ECHO);
    newt.c_cc[VEOL] = 1;
    newt.c_cc[VEOF] = 2;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);      // apply new settings

    int c = std::getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);      // restore old settings
    return c;
}