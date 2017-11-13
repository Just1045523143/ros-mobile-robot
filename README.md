# Mobile Robot

ROS package with a library providing tools for wheeled mobile robots e.g. for kinematics calculation, ackermann steering calculation, etc.

## Prerequisites

* Eigen

## Usage

Set the mobile_robot package as a dependency in package.xml and CMakeLists.txt of your package. 

```cpp
#include <mobile_robot/mobile_robot.h>

...

MobileRobot robot;

//for each wheel, call addWheel
robot.addWheel(wheel_radius, wheel_distance, wheel_alpha, wheel_beta,  max_angular_speed, min_steering_angle, max_steering_angle);

//solving forward kinematics to get odometry
std::vector<float> wheel_angles; //size is number of wheels
std::vector<float> wheel_velocities;
robot.setWheelState(wheel_angles, wheel_velocities);
robot::Velocity vel = robot.getVelocity();

```

The wheel parameter naming follows Springer Handbook of Robotics notation.


### Ackermann steering

For steerable wheels you can get double-ackermann wheel steering angles and velocities under assumptions:

* center of rotation is at [0,R] in _/base\_link_ frame.

The input are virtual angle and robot linear velocity. Output are angles of the wheels and angular velocities. If the input is over the limits, max possible angle and angular velocity is chosen.


```cpp

std::vector<float> output_wheel_angles;
std::vector<float> output_wheel_angular_velocities;
robot.getAckermann(virtual_angle, robot_velocity, output_wheel_angles, output_wheel_angular_velocities);

```

### URDF configuration

Typically, the configuration of the wheels of the robot is stored in the URDF robot description. Sample code retreiving wheel configuration from robot description for wheels with link names {fl,fr,rl,rr}\_wheel\_link and traction joint names {fl,fr,rl,rr}\_wheel\_joint, and steering joint names {fl,fr,rl,rr}\_steering\_wheel\_joint.

```cpp

#include <urdf/model.h>

...

urdf::Model model;
//read robot description
model.initParam("robot_description"); 
std::array<std::string,4> wheels = {"fl", "fr", "rl", "rr"};

for (int i = 0; i < wheels.size(); ++i) {
	const urdf::Geometry *geom = model.getLink( wheels.at(i) + "_wheel_link" )->collision->geometry.get();

	const urdf::Cylinder *c= dynamic_cast<const urdf::Cylinder*> (geom) ;

	float wheel_radius = (float) c->radius;
	urdf::Pose wheel_pose = model.getJoint(wheels.at(i) + "_steering_wheel_joint" )->parent_to_joint_origin_transform;

	urdf::JointLimits steering_limits = *model.getJoint( wheels.at(i) + "_steering_wheel_joint" )->limits;
	urdf::JointLimits traction_limits = *model.getJoint( wheels.at(i) + "_wheel_joint" )->limits;

	//conversion to polar coordinates in plane
	float wheel_distance = std::sqrt(wheel_pose.position.x * wheel_pose.position.x + wheel_pose.position.y * wheel_pose.position.y );

	float wheel_alpha = std::atan(wheel_pose.position.y / wheel_pose.position.x);
	if(wheel_pose.position.x < 0) wheel_alpha += M_PI;

	//conversion the angle to range (-pi, pi>
	if(wheel_alpha > M_PI) wheel_alpha -= 2 * M_PI;
	else if(wheel_alpha <= -M_PI) wheel_alpha += 2 * M_PI;

	Eigen::Quaternion<float> q(wheel_pose.rotation.w, wheel_pose.rotation.x, wheel_pose.rotation.y,wheel_pose.rotation.z);
	Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
	float wheel_beta = euler[1] - wheel_alpha;

	//conversion the angle to range (-pi, pi>
	if(wheel_beta > M_PI) wheel_beta -= 2 * M_PI;
	else if(wheel_beta <= -M_PI) wheel_beta += 2 * M_PI;

	ftr.addWheel(wheel_radius, wheel_distance, wheel_alpha, wheel_beta,  traction_limits.velocity, steering_limits.lower, steering_limits.upper);
}
```


## Versions

* v0.0.1
	* Steerable wheels
	* Forward kinematics
	* Double Ackermann steering solution