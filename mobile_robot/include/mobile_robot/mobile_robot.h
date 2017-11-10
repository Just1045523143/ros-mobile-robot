/*
 * mobile_robot.h
 *  -> Class providing generic tools for mobile robots.
 *
 *  Created on: September, 2016
 *      Author: Jakub Tomasek jtomasek@gmv.com
 */

#ifndef MOBILE_ROBOT_H
#define MOBILE_ROBOT_H

#include <math.h>
#include <array>
#include <vector>
#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>


namespace robot {


	struct Vec2D {
	  float x,y;
	};

	struct Velocity {
		Vec2D velocity;
		float angular_velocity;
	};

	enum wheel_type {FIXED, STEERABLE, SWEDISH, CASTER};
	struct Wheel {
		int id;
		enum wheel_type type;
		float max_angular_speed; //units [rad/s]
		float min_angle, max_angle; //units [rad]
		float r; // radius
		float l, alpha; // coordinates of the wheel in polar coordinates: distance from robot frame [m] , angle[rad]
		float beta; // rotation of the wheel

		float theta = 0; //immediate steering angle of the wheel
		float omega = 0; //immediate angular velocity

		float getX() {
			return cos(alpha) * l;
		}

		float getY() {
			return sin(alpha) * l;
		}

		/*
		 * @brief distance in cartezian coordinates
		 */
		float getDistance(float x, float y) {
			return std::sqrt( ( getX() - x ) * ( getX() - x ) + ( getY() - y ) * ( getY() - y )  );
		}
	};


}

class MobileRobot {
private:
	int N; //number of wheels
	float max_speed_, max_angular_speed_, max_ackermann_speed_, max_crab_angle_, max_ackermann_angle_;
	std::vector<robot::Wheel> wheels;

public:

	MobileRobot();


	virtual ~MobileRobot();

	/*
	 * @brief Sets different than default parameters
	 */
	void setParams(const float max_robot_speed, const float max_crab_angle, const float max_ackermann_angle);

	/*
	 * @brief Adds a wheel.
	 * @param radius: [m]
	 * @param l: distance of wheel in the robot frame in polar coordinates
	 * @param alpha: angle in polar coordinates of the wheel
	 * @param beta: angle of the wheel
	 * @param max_angular_speed
	 * @param minimum relative rotation of the wheel
	 * @param maximum relative rotation of the wheel
	 */
	void addWheel(float radius, float l, float alpha, float beta, float max_angular_speed, float min_angle, float max_angle);

	/*
	 * @brief check whether value is in range <min_value, max_value>, if yes returns true, if not sets the value in the range and returns false
	 * @return true if in range
	 */
	bool inBounds(float& value, const float min_value, const float max_value);

	/*
	 * @brief checks whether the requested values satisfy the wheels limits for crab movement, if not change the value to max allowed
	 * @param robot_velocity [m/s]
	 * @param crab_angle: angle of the wheels [rad]
	 */
	bool checkMoveCrab(float& robot_velocity, float& crab_angle);

	/*
	 * @brief checks whether the requested values satisfy the wheels limits for Ackermann movement, if not change the value to max allowed
	 * @param robot_velocity:  requested robot velocity [m/s]
	 * @param ackermann_angle: requested virtual angle  [rad]
	 */
	bool checkMoveAckermann(float& robot_velocity, float& ackermann_angle);

	/*
		 * @brief checks limits of the rover among all wheels
		 * @param rover_angular_velocity: angular speed of the rover [rad/s]
		 *
		 */
	bool checkMovePointTurn(float& rover_angular_velocity);

	/*
	 * @brief checks limits of the wheel among all wheels
	 * @param wheel_speed: angular speed [rad/s]
	 *
	 */
	bool checkMoveWheel(float& wheel_speed);



	/*
	 * @brief checks limits of the wheel among all wheels
	 * @param wheel_id
	 * @param wheel_speed: angular speed [rad/s]
	 *
	 */
	bool checkMoveWheel(int wheel_id, float& wheel_speed);

	/*
	 * @brief checks limits of steering
	 * @wheel_angle: relative angle to home position (beta)
	 */
	bool checkSteerWheel(int wheel_id, float& wheel_angle);

	/*
	 * @brief Computes angular velocity for a wheel from angular velocity for the robot. Assumes symmetrical robot.
	 * @param robot_angular_velocity [rad/s]
	 * @return angular velocity [rad/s]
	 */
	float getPointTurnWheelSpeed(const float robot_angular_velocity);

	/*
	 * @brief Calculates angulars speed of a wheel from robot linear speed
	 * @param robot_speed : linear velocity of the robot [m/s]
	 * @return wheel angular velocity [rad/s]
	 */
	float getWheelAngularSpeed(float robot_speed);

	/*
	 * @brief Get immediate velocity of the robot given the state of the wheels.
	 * @param wheel_angle [rad]
	 * @param wheel_velocity [rad/s]
	 */
	robot::Velocity getVelocity();

	/*
	 * @brief variable getter
	 */
	float getMaxSpeed();

	/*
	 * @brief variable getter
	 */
	float getMaxAngularSpeed();

	/*
	 * @brief variable getter
	 */
	float getMaxAckermannSpeed();


	/*
	 * @brief variable getter
	 */
	float getMaxCrabAngle();

	/*
	 * @brief variable getter
	 */
	float getMaxAckermannAngle();

	/*
	 * @brief computes angles and velocities for generalized ackermann steering with center of the arc at x = 0 and y given by virtual angle
	 * @param virtual_angle
	 * @param robot_velocity: linear velocity of the robot
	 * @param wheel_angle: output data structure with angles of wheels in the frame of the wheels
	 * @param wheel_velocity: output with velocities for each wheel
	 */
	void getAckermann(const float virtual_angle, const float robot_velocity, std::vector <float>& wheel_angles, std::vector <float>& wheel_angular_velocities);

	/*
	 * @brief computes maximum virtual angle and speed for ackermann steering given limits of the wheel and its position
	 * @param wheel
	 * @param max_ackermann_angle: output
	 * @param max_ackermann_speed: output
	 */
	void getWheelAckermannLimit(const robot::Wheel wheel, float& max_ackermann_angle, float& max_ackermann_speed);

	/*
	 * @brief converts angle to the base (-pi,+pi>
	 */
	float getAngleBase(const float angle);

	/*
	 * @brief sets state of the wheels
	 */
	void setWheelState(const std::vector <float>& wheel_angle, const std::vector <float>& wheel_velocity);

	/*
	 * @return max of absolute differences between @param wheel_angle_goal and current state of the wheels
	 */
	float getMaxSteeringDistance(const std::vector <float>& wheel_angle_goal);
};


#endif /* MOBILE_ROBOT_H */
