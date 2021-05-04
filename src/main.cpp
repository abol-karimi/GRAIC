#include "ros/ros.h"
#include "VoronoiPlanner.h"
#include "voronoi/VoronoiPlannerInput.h"
#include "voronoi/VoronoiPlannerOutput.h"
#include <ostream>

class ManageROS
{
public:
    float allowed_obs_dist = 0.3f; // in meters

    ManageROS()
    {
        planner_sub = ros_node.subscribe<voronoi::VoronoiPlannerInput>("voronoi_input", 1, &ManageROS::OnVoronoiInput, this);
        planner_pub = ros_node.advertise<voronoi::VoronoiPlannerOutput>("voronoi_output", 1);
        marker_pub = ros_node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    }

    void OnVoronoiInput(const voronoi::VoronoiPlannerInput::ConstPtr &msg)
    {
        // Convert VoronoiPlannerInput
        point_type car_location(msg->car_location.x, msg->car_location.y);
        point_type milestone(msg->milestone.x, msg->milestone.y);
        std::vector<segment_type>
            obstacles;
        for (auto &obs : msg->obstacles)
        {
            point_type start(obs.start.x, obs.start.y);
            point_type end(obs.end.x, obs.end.y);
            obstacles.push_back(segment_type(start, end));
        }

        VoronoiPlanner planner;
        const std::vector<point_type> &plan = planner.GetPlan(car_location, milestone, obstacles, allowed_obs_dist);

        Publish(plan);

        // Visualize the obstacles and the plan
        DrawWalls(obstacles);
        DrawPlan(plan);
    }

    void Publish(const std::vector<point_type> &plan) const
    {
        voronoi::VoronoiPlannerOutput outPlan;
        for (auto &point : plan)
        {
            geometry_msgs::Vector3 outPoint;
            outPoint.x = point.x();
            outPoint.y = point.y();
            outPoint.z = 0.0;
            outPlan.plan.push_back(outPoint);
        }
        planner_pub.publish(outPlan);
    }

private:
    // Visualizations (in laser frame)
    void DrawWalls(const std::vector<segment_type> &obstacles)
    {
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "/laser";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        // Line width
        line_list.scale.x = 0.1;
        // Walls are green
        line_list.color.g = 1.0;
        line_list.color.a = 1.0;

        for (auto &wall : obstacles)
        {
            geometry_msgs::Point p0, p1;
            p0.x = wall.low().x();
            p0.y = wall.low().y();
            p0.z = 0.f;
            p1.x = wall.high().x();
            p1.y = wall.high().y();
            p1.z = 0.f;
            line_list.points.push_back(p0);
            line_list.points.push_back(p1);
        }
        marker_pub.publish(line_list);
    }

    void DrawPlan(const std::vector<point_type> &Plan)
    {
        if (Plan.size() == 0)
            return;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "/laser";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 2;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        // Line width
        line_strip.scale.x = 0.1;
        // Plan is white
        line_strip.color.r = 1.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        for (auto &point : Plan)
        {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0.f;
            line_strip.points.push_back(p);
        }
        marker_pub.publish(line_strip);
    }

    ros::NodeHandle ros_node;
    ros::Subscriber planner_sub;
    ros::Publisher planner_pub;
    ros::Publisher marker_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voronoi_controller");
    ManageROS manageROS;

    ros::spin();

    return EXIT_SUCCESS;
}