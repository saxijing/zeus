#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualize07");
    ros::NodeHandle nh;

    // 创建 rviz_visual_tools 对象
    rviz_visual_tools::RvizVisualToolsPtr visual_tools;
    visual_tools.reset(new rviz_visual_tools::RvizVisualTools("map", "/rviz_visual_markers"));

    // CSV 文件路径
    std::string csv_file = "/home/saxijing/carla-ros-bridge/catkin_ws/data/reference_point/T7_waypoints_and_roadEdge_map.csv";

    // 打开 CSV 文件
    std::ifstream file(csv_file);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open the CSV file.");
        return 1;
    }

    std::string line;
    std::vector<std::vector<double>> path_points;

    // 读取文件的每一行（跳过第一行的头部）
    std::getline(file, line);
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::vector<double> point(3);
        std::string value;
        
        // 分别读取x, y, z坐标
        for(int col=0; getline(ss, line, ','); ++col)
        {
            try
            {
                switch(col)
                {
                    case 13:
                        point[0]=stod(line);//x
                        break;
                    case 14:
                        point[1]=stod(line);//y
                        break;
                }
            }
            catch(const invalid_argument &e)
            {    cerr<<"转换错误："<<e.what()<<endl;}
            catch(const out_of_range &e)
            {    cerr<<"值超出范围："<<e.what()<<endl;}

        }
        point[2]=0.0;

        // 将点加入路径
        path_points.push_back(point);
    }

    file.close();

    // 显示路径点
    for (const auto& point : path_points)
    {
        geometry_msgs::Point pt;
        pt.x = point[0];
        pt.y = point[1];
        pt.z = point[2];

        // 发布每个点为红色的大球体
        visual_tools->publishSphere(pt, rviz_visual_tools::RED, rviz_visual_tools::LARGE, "Path Point");
    }

    // 发布并显示所有标记
    visual_tools->trigger();

    // 让节点保持运行
    ros::spin();
    return 0;
}
