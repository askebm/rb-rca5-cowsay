#include <Robot/SimpleRobotControlGazebo.hpp>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

int main(int argc, char *argv[])
{
	gazebo::client::setup();

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

	rba::SimpleRobotControlGazebo agent{node};


	while(true)
		;

  gazebo::client::shutdown();

	return 0;
}
