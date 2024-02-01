#include <ros/ros.h>
#include <people_msgs/People.h>
#include <people_msgs/Person.h> // Include the Person message
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_people_detection");

  ros::NodeHandle priv_n("~");
  ros::NodeHandle n;

  double x1, y1, phi1, vel1, x2, y2, phi2, vel2;

  visualization_msgs::Marker people_marker1, people_marker2;
  people_marker1.header.frame_id = "map";
  people_marker1.ns = "model1";
  people_marker1.id = 0;
  people_marker1.type = visualization_msgs::Marker::MESH_RESOURCE;
  people_marker1.action = visualization_msgs::Marker::MODIFY;
  people_marker1.mesh_resource = "package://social_nav_simulation/gazebo/models/human/meshes/standing.dae";
  people_marker1.mesh_use_embedded_materials = true;
  people_marker1.scale.x = 1.0;
  people_marker1.scale.y = 1.0;
  people_marker1.scale.z = 1.0;

  people_marker2.header.frame_id = "map";
  people_marker2.ns = "model2";
  people_marker2.id = 1;
  people_marker2.type = visualization_msgs::Marker::MESH_RESOURCE;
  people_marker2.action = visualization_msgs::Marker::MODIFY;
  people_marker2.mesh_resource = "package://social_nav_simulation/gazebo/models/human/meshes/standing.dae";
  people_marker2.mesh_use_embedded_materials = true;
  people_marker2.scale.x = 1.0;
  people_marker2.scale.y = 1.0;
  people_marker2.scale.z = 1.0;

  priv_n.param("x1", x1, 3.24);
  priv_n.param("y1", y1, -1.1);
  priv_n.param("phi1", phi1, 0.0);
  priv_n.param("vel1", vel1, 0.0);

  priv_n.param("x2", x2, 4.84);
  priv_n.param("y2", y2, -1.1);
  priv_n.param("phi2", phi2, M_PI); // Set orientation to face the opposite direction
  priv_n.param("vel2", vel2, 0.0);

  ros::Publisher people_pub = n.advertise<people_msgs::People>("people", 1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("people_viz", 1);

  people_msgs::People people;
  people_msgs::Person person1, person2;

  ros::Duration(2.0).sleep();
  ROS_INFO("starting fake people detection");

  ros::Time start = ros::Time::now();

  ros::Rate rate(10);

  while (ros::ok())
  {
    people.people.clear();
    ros::Duration delta = (ros::Time::now() - start);
    double delta_t = delta.toSec();

    // Person 1
    person1.velocity.x = vel1 * cos(phi1);
    person1.velocity.y = vel1 * sin(phi1);
    person1.position.x = x1 + delta_t * person1.velocity.x;
    person1.position.y = y1 + delta_t * person1.velocity.y;
    person1.name = "person1";

    people_marker1.pose.position.x = person1.position.x;
    people_marker1.pose.position.y = person1.position.y;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(phi1 + M_PI_2), people_marker1.pose.orientation);

    people.header.frame_id = "map";
    people.header.stamp = start + delta;
    people.people.push_back(person1);

    // Person 2
    person2.velocity.x = vel2 * cos(phi2);
    person2.velocity.y = vel2 * sin(phi2);
    person2.position.x = x2 + delta_t * person2.velocity.x;
    person2.position.y = y2 + delta_t * person2.velocity.y;
    person2.name = "person2";

    people_marker2.pose.position.x = person2.position.x;
    people_marker2.pose.position.y = person2.position.y;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(phi2 + M_PI_2), people_marker2.pose.orientation);

    people.people.push_back(person2);

    people_pub.publish(people);
    marker_pub.publish(people_marker1);
    marker_pub.publish(people_marker2);
    rate.sleep();
  }

  return 0;
}

