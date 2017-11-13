/*
 * mobile_robot.cpp
 *  -> Class providing generic tools for mobile robots.
 *
 *  Created on: September, 2016
 *      Author: Jakub Tomasek jakub@tomasek.fr
 */

#include <mobile_robot/mobile_robot.h>


MobileRobot::MobileRobot():
N(0),//number of wheels
//initializing limits to high numbers. When wheels added, limits are adjusted.
max_speed_(1000),
max_ackermann_speed_(1000),
max_angular_speed_(1000),
max_ackermann_angle_(M_PI / 2),
max_crab_angle_(M_PI / 2)

{


}



void MobileRobot::addWheel(float radius, float l, float alpha, float beta, float max_angular_speed, float min_angle, float max_angle) {
		robot::Wheel new_wheel;
		new_wheel.id = N;
		N++; //increase number of wheels

		new_wheel.type = robot::wheel_type::STEERABLE;
		new_wheel.r = radius;
		new_wheel.l = l;
		new_wheel.alpha = alpha;
		new_wheel.beta = beta;
		new_wheel.max_angular_speed = max_angular_speed;
		new_wheel.min_angle = min_angle;
		new_wheel.max_angle = max_angle;

		float max_robot_speed = max_angular_speed * radius;
		if (max_robot_speed < max_speed_) max_speed_ = max_robot_speed;

		float max_robot_angular_speed =  max_robot_speed / l;
		if (max_robot_angular_speed < max_angular_speed_) max_angular_speed_ = max_robot_angular_speed;

		float ackermann_steering_limit, ackermann_speed_limit;
		getWheelAckermannLimit(new_wheel, ackermann_steering_limit, ackermann_speed_limit);
		if (ackermann_steering_limit < max_ackermann_angle_ ) max_ackermann_angle_ = ackermann_steering_limit;
		if (ackermann_speed_limit < max_ackermann_speed_ ) max_ackermann_speed_ = ackermann_speed_limit;

		if (std::fabs(min_angle) < max_crab_angle_ ) max_crab_angle_ = std::fabs(min_angle);
		if (std::fabs(max_angle) < max_crab_angle_ ) max_crab_angle_ = std::fabs(max_angle);



		wheels.push_back(new_wheel);
}

float MobileRobot::getMaxSpeed() {
	return max_speed_;
}

float MobileRobot::getMaxAngularSpeed() {
	return max_angular_speed_;
}

float MobileRobot::getMaxAckermannSpeed() {
	return max_ackermann_speed_;
}


float MobileRobot::getMaxCrabAngle() {
	return max_crab_angle_;
}


float MobileRobot::getMaxAckermannAngle() {
	return max_ackermann_angle_;
}


bool MobileRobot::inBounds(float& value, const float min_value, const float max_value) {
	if(value < min_value) {
		value = min_value;
		return false;
	}
	else if(value > max_value) {
		value = max_value;
		return false;
	}

	return true;
}


bool MobileRobot::checkMoveCrab(float& robot_velocity, float& crab_angle) {
	return inBounds(robot_velocity, -max_speed_, max_speed_) and  inBounds(crab_angle, - max_crab_angle_, max_crab_angle_);
}


bool MobileRobot::checkMoveAckermann(float& robot_velocity, float& ackermann_angle) {
	return inBounds(robot_velocity, -max_speed_, max_speed_) and  inBounds(ackermann_angle, - max_ackermann_angle_, max_ackermann_angle_);
}

bool MobileRobot::checkMovePointTurn(float& rover_angular_velocity) {
	return inBounds(rover_angular_velocity, -max_angular_speed_, max_angular_speed_);
}


bool MobileRobot::checkMoveWheel(float& wheel_speed) {
	bool result = true;
	for (int i = 0; i < N; ++i) {
		robot::Wheel w = wheels.at(i);
		if(not checkMoveWheel(i, wheel_speed) ) result = false;
	}
	return result;
}


bool MobileRobot::checkMoveWheel(int wheel_id, float& wheel_speed) {
	robot::Wheel w = wheels.at(wheel_id);

	return inBounds(wheel_speed, -w.max_angular_speed, w.max_angular_speed);
}

bool MobileRobot::checkSteerWheel(int wheel_id, float& wheel_angle) {
	if(wheel_id > N) {
		throw std::invalid_argument("wheel_id higher than the number of wheels");
		return false;
	}
	robot::Wheel w = wheels.at(wheel_id);

	if(w.type != robot::wheel_type::STEERABLE) {
		wheel_angle = 0;
		return false;
	}
	return inBounds(wheel_angle, w.min_angle, w.max_angle);


}


float MobileRobot::getWheelAngularSpeed(float robot_speed) {
	if(N > 0) {
		robot::Wheel w = wheels.at(0);
		return robot_speed / w.r;
	}
	else return 0;

}

robot::Velocity MobileRobot::getVelocity() {
	robot::Velocity result;
	result.angular_velocity=0;
	result.velocity.x = 0;
	result.velocity.y = 0;

	//build condition matrix
	Eigen::MatrixXf A (2 * N, 3);
	Eigen::MatrixXf b (2 * N,1);

	for (int i = 0; i < N; ++i) {
		robot::Wheel w = wheels.at(i);

		if(w.type == robot::wheel_type::STEERABLE) {
			//traction condition
			A(i,0) = (float) sin(w.alpha + w.beta + w.theta);
			A(i,1) = (float) -cos(w.alpha + w.beta + w.theta);
			A(i,2) = (float) - w.l * cos(w.beta + w.theta);
			b(i,0) = (float) w.r * w.omega;
			//no side slip condition
			A(N+i,0) = (float) cos(w.alpha + w.beta + w.theta);
			A(N+i,1) = (float) sin(w.alpha + w.beta + w.theta);
			A(N+i,2) = (float) w.l * sin(w.beta + w.theta);
			b(N+i,0) = (float) 0;
		}
		else {
			throw std::logic_error("Function not yet implemented");
			return result;
		}

	}



	//least squares solution of the system of equation

	Eigen::Vector3f xi = (A.transpose() * A).ldlt().solve(A.transpose() * b);
	 result.velocity.x = xi(0);
	 result.velocity.y = xi(1);
	 result.angular_velocity = xi(2);

	 return result;

}


void MobileRobot::getAckermann(const float virtual_angle, const float robot_velocity, std::vector <float>& wheel_angles, std::vector <float>& wheel_angular_velocities) {

	if(N <= 0 ) {
		throw std::invalid_argument("getMaxSteeringDistance: there are no wheels.");
		return;
	}

	float v_r = robot_velocity;
	float phi = virtual_angle;
	inBounds(v_r, - max_ackermann_speed_, max_ackermann_speed_);
	inBounds(phi, - max_ackermann_angle_, max_ackermann_angle_);



	//computing radius of rotation of the rover
	//the center of rotation is at [0,R] or [0,-R] for negative virtual angle in robot frame
	float R;
	if(std::fabs(phi) > 0)	R = wheels.at(0).getX() / std::tan(phi);
	else { //robot goes forward in the robot frame
		for (int i = 0; i < N; ++i) {
			robot::Wheel w = wheels.at(i);

			//wheels should be aligned forward
			float wheel_angle_wheel = ( M_PI / 2.0 ) - w.alpha - w.beta;
			wheel_angles.push_back( getAngleBase (wheel_angle_wheel)  );

			//velocity of the rover covnerted to angular velocity of the wheel
			wheel_angular_velocities.push_back(v_r / w.r);

		}
		return;
	}




	//computing the angles for each wheel
	for (int i = 0; i < N; ++i) {
		robot::Wheel w = wheels.at(i);

		//computing wheel angle in the frame of the robot
		float wheel_angle_robot =  std::atan ( w.getX() / ( R - w.getY() ) ) + (M_PI / 2.0);
		//computing wheel angle in the frame of the wheel
		float wheel_angle_wheel = wheel_angle_robot - w.alpha - w.beta;
		wheel_angles.push_back( getAngleBase (wheel_angle_wheel)  );



		wheel_angular_velocities.push_back( v_r / (w.r * std::fabs(R) ) * w.getDistance(0, R) );

	}

	return;
}

void MobileRobot::getWheelAckermannLimit(const robot::Wheel wheel, float& max_ackermann_angle, float& max_ackermann_speed) {
	robot::Wheel w = wheel;
	float theta;

	//choose tighter limit of the wheel
	if(std::fabs(w.max_angle) < std::fabs(w.min_angle)){
		theta = std::fabs(w.max_angle);
	}
	else {
		theta = std::fabs(w.min_angle);
	}

	//computing the steering angle limit
	max_ackermann_angle = std::atan( w.getX() / ( w.getX() / std::tan(theta  + w.alpha + w.beta - M_PI / 2.0)  + w.getY() ) );

	 //we take the smallest ackermann circle possible = highest ackermann angle
	float R = std::fabs(w.getX() / std::tan(max_ackermann_angle));


	//get the higher distance depending on turning left or right
	float dr;
	if(w.getDistance(0, R) > w.getDistance(0, -R) ) dr = w.getDistance(0, R) - R;
	else dr = w.getDistance(0, -R) - R;

	//max robot linear speed
	float v_max =  w.max_angular_speed * w.r;

	//compute max ackermann speed
	max_ackermann_speed = v_max - v_max * dr/ R;
}


float MobileRobot::getAngleBase(const float angle){
	if(angle > M_PI) return (float)(angle - 2.0 * M_PI);
	else if(angle <= -M_PI) return (float)(angle + 2.0 * M_PI);
	else return angle;
}

void MobileRobot::setWheelState(const std::vector <float>& wheel_angle, const std::vector <float>& wheel_velocity) {
	//check whether the size is correct
	if(wheel_angle.size() != N or wheel_velocity.size() != N) {
		throw std::invalid_argument("setWheelState: number of wheels of the mobile robot is different than input arguments.");
		return;
	}
	for (int i = 0; i < N; ++i) {
		wheels.at(i).omega = wheel_velocity.at(i);
		wheels.at(i).theta = wheel_angle.at(i);
	}
}


float MobileRobot::getMaxSteeringDistance(const std::vector <float>& wheel_angle_goal) {

	if(wheel_angle_goal.size() != N ) {
		throw std::invalid_argument("getMaxSteeringDistance: number of wheels of the mobile robot is different than input argument.");
		return 0.0;
	}

	float max = 0;
	for (int i = 0; i < N; ++i) {
		float absolute_angle_difference = std::fabs( wheel_angle_goal.at(i) - wheels.at(i).theta );
		if(absolute_angle_difference > max) max = absolute_angle_difference;
	}
	return max;
}


MobileRobot::~MobileRobot() {

}


