/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};

class Controller
{
public:
	Parameter_t &param;

	Eigen::Vector3d Kp;
	Eigen::Vector3d Kv;
	Eigen::Vector3d Kvi;
	Eigen::Vector3d Kvd;
	Eigen::Vector3d KAng;
	Eigen::Vector3d int_e_v;
	Eigen::Vector3d Gravity;
	std::queue<std::pair<ros::Time, double>> timed_thrust;
	ros::Time last_ctrl_timestamp_{ros::Time(0)};
	Eigen::Vector3d last_bodyrate_{Eigen::Vector3d(0,0,0)};

	quadrotor_msgs::Px4ctrlDebug debug; //debug

	// Thrust-accel mapping params
	double thr_scale_compensate;
	const double rho2 = 0.998; // do not change
	double thr2acc;
	double P;

	Controller(Parameter_t &);

	/* Algorithm0 from  Zhepei Wang*/
	quadrotor_msgs::Px4ctrlDebug update_alg0(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		double voltage);

	Eigen::Vector3d computeLimitedTotalAccFromThrustForce(
		const Eigen::Vector3d &thrustforce,
		const double &mass) const;

	bool flatnessWithDrag(const Eigen::Vector3d &vel,
						  const Eigen::Vector3d &acc,
						  const Eigen::Vector3d &jer,
						  const double &psi,
						  const double &dpsi,
						  double &thr,
						  Eigen::Vector4d &quat,
						  Eigen::Vector3d &omg,
						  const double &mass,
						  const double &grav,
						  const double &dh,
						  const double &dv,
						  const double &cp,
						  const double &veps) const;

	void minimumSingularityFlatWithDrag(const double mass,
										const double grav,
										const Eigen::Vector3d &vel,
										const Eigen::Vector3d &acc,
										const Eigen::Vector3d &jer,
										const double &yaw,
										const double &yawd,
										const Eigen::Quaterniond &att_est,
										Eigen::Quaterniond &att,
										Eigen::Vector3d &omg,
										double &thrust) const;

	/* Algorithm1 from  Zhepei Wang*/
	quadrotor_msgs::Px4ctrlDebug update_alg1(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		double voltage);

	void normalizeWithGrad(
		const Eigen::Vector3d &x,
		const Eigen::Vector3d &xd,
		Eigen::Vector3d &xNor,
		Eigen::Vector3d &xNord) const;

	void computeFlatInput(
		const Eigen::Vector3d &thr_acc,
		const Eigen::Vector3d &jer,
		const double &yaw,
		const double &yawd,
		const Eigen::Quaterniond &att_est,
		Eigen::Quaterniond &att,
		Eigen::Vector3d &omg) const;

	/*Algorithm from the rotor-drag paper*/
	void update_alg2(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		double voltage);

	void computeAeroCompensatedReferenceInputs(
		const Desired_State_t &des,
		const Odom_Data_t &odom, const Parameter_t &param,
		Controller_Output_t *u, Eigen::Vector3d *drag_acc) const;

	Eigen::Quaterniond computeDesiredAttitude(
		const Eigen::Vector3d &des_acc, const double reference_heading,
		const Eigen::Quaterniond &est_q) const;

	Eigen::Vector3d computeRobustBodyXAxis(
		const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
		const Eigen::Vector3d &y_C,
		const Eigen::Quaterniond &est_q) const;

	/* Shared functions*/
	Eigen::Vector3d computePIDErrorAcc(
		const Odom_Data_t &odom, const Desired_State_t &des,
		const Parameter_t &param);

	Eigen::Vector3d computeLimitedTotalAcc(
		const Eigen::Vector3d &PIDErrorAcc,
		const Eigen::Vector3d &ref_acc,
		const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

	Eigen::Vector3d computeLimitedAngularAcc(
		const Eigen::Vector3d candirate_bodyrate);

	double computeDesiredCollectiveThrustSignal(
		const Eigen::Quaterniond &est_q,
		const Eigen::Vector3d &est_v,
		const Eigen::Vector3d &des_acc,
		const Parameter_t &param,
		double voltage);

	double AccurateThrustAccMapping(
		const double des_acc_z,
		double voltage,
		const Parameter_t &param) const;

	Eigen::Vector3d computeFeedBackControlBodyrates(
		const Eigen::Quaterniond &des_q,
		const Eigen::Quaterniond &est_q,
		const Parameter_t &param);

	bool estimateThrustModel(
		const Eigen::Vector3d &est_v,
		const double voltage,
		const Parameter_t &param);

	bool almostZero(const double value) const;

	bool almostZeroThrust(const double thrust_value) const;

	void resetThrustMapping(void);

private:
	static constexpr double kMinNormalizedCollectiveAcc_ = 3;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	static constexpr double kAlmostZeroThrustThreshold_ = 0.01;
	static constexpr double kMaxBodyratesFeedback_ = 4;
	static constexpr double kMaxAngularAcc_ = 60;
};

#endif
