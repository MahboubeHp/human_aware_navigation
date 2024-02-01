#include <people_prediction/constant_velocity_prediction.h>

using namespace human_aware_navigation;

ConstantVelocityPrediction::ConstantVelocityPrediction(std::string name)
{
  //initialize publishers and subscribers
  people_sub_ = nh_.subscribe("people", 1, &ConstantVelocityPrediction::peopleCallback, this);
  prediction_pub_ = nh_.advertise<people_msgs::PeoplePrediction>("people_prediction", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("prediction_viz", 1);

  //get the prediction settings from ros params
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("time_resolution", time_resolution_, 0.5);
  private_nh.param("num_predictions", num_predictions_, 20);

  //initialize the marker for the people prediction
  prediction_marker_.type = visualization_msgs::Marker::SPHERE;
  prediction_marker_.action = visualization_msgs::Marker::MODIFY;
  prediction_marker_.ns = "predictor";
  prediction_marker_.pose.orientation.w = 1;
  prediction_marker_.color.r = 0;
  prediction_marker_.color.g = 0;
  prediction_marker_.color.b = 0.5;
  prediction_marker_.scale.x = 0.2;
  prediction_marker_.scale.y = 0.2;
  prediction_marker_.scale.z = 0.2;
}

void ConstantVelocityPrediction::peopleCallback(people_msgs::People msg)
{
  //parse the people message and predict the trajectories of all people
  people_msgs::People people = msg;
  people_msgs::Person person_one_timestep;
  people_msgs::PeoplePrediction predictions;

  visualization_msgs::MarkerArray markers;

  //if the message stamp is empty, we assign the current time
  if(people.header.stamp == ros::Time(0))
  {
    people.header.stamp = ros::Time::now();
  }

  //in this loop, we take the people from the message and their velocities and
  //fill the prediction container
  for(int i=0; i<num_predictions_; i++)
  {
    people_msgs::People people_one_timestep;
    people_one_timestep.header.frame_id = people.header.frame_id;
    people_one_timestep.header.stamp = people.header.stamp + ros::Duration(i * time_resolution_);

    //loop for the number of people in the environment
    for(int j=0; j<people.people.size(); j++)
    {
      people_msgs::Person person = people.people.at(j);
      people_msgs::Person predicted_person;

      //calculate the position for time step i
      predicted_person.position.x = person.position.x +
          (i * time_resolution_ * person.velocity.x);
      predicted_person.position.y = person.position.y +
          (i * time_resolution_ * person.velocity.y);
      predicted_person.position.z = person.position.z +
          (i * time_resolution_ * person.velocity.z);

      //the velocity stays the same
      predicted_person.velocity = person.velocity;
      
       // set the mob_id for the predicted person
  predicted_person.mob_id = person.mob_id;

      //push back the prediction step
      people_one_timestep.people.push_back(predicted_person);

      //create the prediction marker
      visualization_msgs::Marker prediction_marker;
      prediction_marker.header.frame_id = people.header.frame_id;
      prediction_marker.header.stamp = people.header.stamp;
      prediction_marker.id = j * num_predictions_ + i; // Unique ID for each marker

      prediction_marker.type = visualization_msgs::Marker::SPHERE;
      prediction_marker.action = visualization_msgs::Marker::MODIFY;
      prediction_marker.ns = "predictor";
      prediction_marker.pose.position = predicted_person.position;
      prediction_marker.pose.orientation.w = 1;
      prediction_marker.color.r = 0;
      prediction_marker.color.g = 0;
      prediction_marker.color.b = 0.5;
      prediction_marker.scale.x = 0.2;
      prediction_marker.scale.y = 0.2;
      prediction_marker.scale.z = 0.2;
      // the opacity of the marker is adjusted according to the prediction step
      prediction_marker.color.a = 1 - (i * 1.0 / (num_predictions_ * 1.0));

      markers.markers.push_back(prediction_marker);
    }
    // push back the predictions for this time step to the prediction container
    predictions.predicted_people.push_back(people_one_timestep);
  }

  // publish predictions and prediction markers
  prediction_pub_.publish(predictions);
  marker_pub_.publish(markers);
}

int main(int argc, char **argv)
{
  std::string node_name = "constant_velocity_prediction";
  ros::init(argc, argv, node_name);

  ConstantVelocityPrediction vel_prediction(node_name);

  ros::spin();
  return 0;
}

