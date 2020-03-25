/**
 * @file quaternion.hpp
 *
 * All rotations and axis systems follow the right-hand rule.
 * The Hamilton quaternion product definition is used.
 *
 * In order to rotate a vector v by a righthand rotation defined by the quaternion q
 * one can use the following operation:
 * v_rotated = q^(-1) * [0;v] * q
 * where q^(-1) represents the inverse of the quaternion q.
 * The product z of two quaternions z = q1 * q2 represents an intrinsic rotation
 * in the order of first q1 followed by q2.
 * The first element of the quaternion
 * represents the real part, thus, a quaternion representing a zero-rotation
 * is defined as (1,0,0,0).
 *
 */

#pragma once

#include"../Eigen/Dense"
#include"euler.hpp"
#include<cmath>


	template<typename Type>
	Type wrap_pi(Type x)
	{
		if (!isfinite(x)) {
			return x;
		}

		while (x >= Type(M_PI)) {
			x -= Type(2.0 * M_PI);

		}

		while (x < Type(-M_PI)) {
			x += Type(2.0 * M_PI);

		}

		return x;
	}
	/**
	 * Quaternion class
	 *
	 * The rotation between two coordinate frames is
	 * described by this class.
	 */
	template<typename Type>
	class Quaternion : public Eigen::Matrix<Type, 4, 1>
	{
	public:

		/**
		 * Standard constructor
		 */
		Quaternion() :
			Eigen::Matrix<Type, 4, 1>()
		{
			Quaternion &q = *this;
			q(0) = 1;
			q(1) = 0;
			q(2) = 0;
			q(3) = 0;
		}

		/**
		 * Constructor from Matrix41
		 *
		 * @param other Matrix41 to copy
		 */
		Quaternion(const Eigen::Matrix<Type, 4, 1> &other) :
			Eigen::Matrix<Type, 4, 1>(other)
		{
		}

		/**
		 * Constructor from dcm
		 *
		 * Instance is initialized from a dcm representing coordinate transformation
		 * from frame 2 to frame 1.
		 *
		 * @param dcm dcm to set quaternion to
		 */
		Quaternion(const Eigen::Matrix<Type, 3, 3> &R) :
			Eigen::Matrix<Type, 4, 1>()
		{
			Quaternion &q = *this;
			Type t = R.trace();
			if (t > Type(0)) {
				t = sqrt(Type(1) + t);
				q(0) = Type(0.5) * t;
				t = Type(0.5) / t;
				q(1) = (R(2, 1) - R(1, 2)) * t;
				q(2) = (R(0, 2) - R(2, 0)) * t;
				q(3) = (R(1, 0) - R(0, 1)) * t;
			}
			else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
				t = sqrt(Type(1) + R(0, 0) - R(1, 1) - R(2, 2));
				q(1) = Type(0.5) * t;
				t = Type(0.5) / t;
				q(0) = (R(2, 1) - R(1, 2)) * t;
				q(2) = (R(1, 0) + R(0, 1)) * t;
				q(3) = (R(0, 2) + R(2, 0)) * t;
			}
			else if (R(1, 1) > R(2, 2)) {
				t = sqrt(Type(1) - R(0, 0) + R(1, 1) - R(2, 2));
				q(2) = Type(0.5) * t;
				t = Type(0.5) / t;
				q(0) = (R(0, 2) - R(2, 0)) * t;
				q(1) = (R(1, 0) + R(0, 1)) * t;
				q(3) = (R(2, 1) + R(1, 2)) * t;
			}
			else {
				t = sqrt(Type(1) - R(0, 0) - R(1, 1) + R(2, 2));
				q(3) = Type(0.5) * t;
				t = Type(0.5) / t;
				q(0) = (R(1, 0) - R(0, 1)) * t;
				q(1) = (R(0, 2) + R(2, 0)) * t;
				q(2) = (R(2, 1) + R(1, 2)) * t;
			}
		}

		/**
		 * Constructor from euler angles
		 *
		 * This sets the instance to a quaternion representing coordinate transformation from
		 * frame 2 to frame 1 where the rotation from frame 1 to frame 2 is described
		 * by a 3-2-1 intrinsic Tait-Bryan rotation sequence.
		 *
		 * @param euler angle instance
		 */
		Quaternion(const Euler<Type> &euler) :
			Eigen::Matrix<Type, 4, 1>()
		{
			Quaternion &q = *this;
			Type cosroll_2 = Type(cos(euler.GetRoll() / Type(2.0)));
			Type cospitch_2 = Type(cos(euler.GetPitch() / Type(2.0)));
			Type cosyaw_2 = Type(cos(euler.GetYaw() / Type(2.0)));
			Type sinroll_2 = Type(sin(euler.GetRoll() / Type(2.0)));
			Type sinpitch_2 = Type(sin(euler.GetPitch() / Type(2.0)));
			Type sinyaw_2 = Type(sin(euler.GetYaw() / Type(2.0)));
			q(0) = cosroll_2 * cospitch_2 * cosyaw_2 +
				sinroll_2 * sinpitch_2 * sinyaw_2;
			q(1) = sinroll_2 * cospitch_2 * cosyaw_2 -
				cosroll_2 * sinpitch_2 * sinyaw_2;
			q(2) = cosroll_2 * sinpitch_2 * cosyaw_2 +
				sinroll_2 * cospitch_2 * sinyaw_2;
			q(3) = cosroll_2 * cospitch_2 * sinyaw_2 -
				sinroll_2 * sinpitch_2 * cosyaw_2;
		}

		/**
		 * Constructor from quaternion values
		 *
		 * Instance is initialized from quaternion values representing coordinate
		 * transformation from frame 2 to frame 1.
		 * A zero-rotation quaternion is represented by (1,0,0,0).
		 *
		 * @param a set quaternion value 0
		 * @param b set quaternion value 1
		 * @param c set quaternion value 2
		 * @param d set quaternion value 3
		 */
		Quaternion(Type a, Type b, Type c, Type d) :
			Eigen::Matrix<Type, 4, 1>()
		{
			Quaternion &q = *this;
			q(0) = a;
			q(1) = b;
			q(2) = c;
			q(3) = d;
		}

		/**
		 * Quaternion multiplication operator
		 *
		 * @param q quaternion to multiply with
		 * @return product
		 */
		Quaternion operator*(const Quaternion &q) const
		{
			const Quaternion &p = *this;
			Quaternion r;
			r(0) = p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3);
			r(1) = p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2);
			r(2) = p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1);
			r(3) = p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
			return r;
		}

		/**
		 * Self-multiplication operator
		 *
		 * @param other quaternion to multiply with
		 */
		void operator*=(const Quaternion &other)
		{
			Quaternion &self = *this;
			self = self * other;
		}

		/**
		 * Scalar multiplication operator
		 *
		 * @param scalar scalar to multiply with
		 * @return product
		 */
		Quaternion operator*(Type scalar) const
		{
			const Quaternion &q = *this;
			return scalar * q;
		}

		/**
		 * Scalar self-multiplication operator
		 *
		 * @param scalar scalar to multiply with
		 */
		void operator*=(Type scalar)
		{
			Quaternion &q = *this;
			q = q * scalar;
		}

		/**
		 * Computes the derivative of q_12 when
		 * rotated with angular velocity expressed in frame 2
		 * v_2 = q_12 * v_1 * q_12^-1
		 * d/dt q_12 = 0.5 * q_12 * omega_12_2
		 *
		 * @param w angular rate in frame 2
		 */
		Eigen::Matrix<Type, 4, 1> derivative1(const Eigen::Matrix<Type, 3, 1> &w) const
		{
			const Quaternion &q = *this;
			Quaternion<Type> v(0, w(0, 0), w(1, 0), w(2, 0));
			return q * v  * Type(0.5);
		}

		/**
		 * Computes the derivative of q_12 when
		 * rotated with angular velocity expressed in frame 2
		 * v_2 = q_12 * v_1 * q_12^-1
		 * d/dt q_12 = 0.5 * omega_12_1 * q_12
		 *
		 * @param w angular rate in frame (typically reference frame)
		 */
		Eigen::Matrix<Type, 4, 1> derivative2(const Eigen::Matrix<Type, 3, 1> &w) const
		{
			const Quaternion &q = *this;
			Quaternion<Type> v(0, w(0, 0), w(1, 0), w(2, 0));
			return v * q  * Type(0.5);
		}

		/**
		 * Invert quaternion in place
		 */
		void Invert()
		{
			*this = this->Inversed();
		}

		/**
		 * Invert quaternion
		 *
		 * @return inverted quaternion
		 */
		Quaternion Inversed()
		{
			Quaternion &q = *this;
			Type normSq = q.dot(q);
			return Quaternion(
				q(0) / normSq,
				-q(1) / normSq,
				-q(2) / normSq,
				-q(3) / normSq);
		}

		void Norm() {
			Quaternion &q = *this;
			Type normSq = sqrt(q.dot(q));
			for (int i = 0; i != 4; i++)
				q(i) /= normSq;
		}
		void from_axis_angle(Eigen::Matrix<Type, 3, 1> vec)
		{
			Quaternion &q = *this;
			Type theta = vec.norm();

			if (theta < Type(1e-10)) {
				q(0) = Type(1.0);
				q(1) = q(2) = q(3) = 0;
				return;
			}

			vec /= theta;
			from_axis_angle(vec, theta);
		}

		void from_axis_angle(const Eigen::Matrix<Type,3, 1> &axis, Type theta)
		{
			Quaternion &q = *this;

			if (theta < Type(1e-10)) {
				q(0) = Type(1.0);
				q(1) = q(2) = q(3) = 0;
			}
			Type magnitude = sin(theta / 2.0);

			q(0) = cos(theta / 2.0);
			q(1) = axis(0) * magnitude;
			q(2) = axis(1) * magnitude;
			q(3) = axis(2) * magnitude;
		}

		Eigen::Matrix<Type, 3, 1> to_axis_angle()
		{
			Quaternion &q = *this;
			Type axis_magnitude = Type(sqrt(q(1) * q(1) + q(2) * q(2) + q(3) * q(3)));
			Eigen::Matrix<Type, 3, 1> vec;
			vec(0) = q(1);
			vec(1) = q(2);
			vec(2) = q(3);

			if (axis_magnitude >= Type(1e-10)) {
				vec = vec / axis_magnitude;
				vec = vec * wrap_pi(Type(2.0) * atan2(axis_magnitude, q(0)));
			}

			return vec;
		}
	};

	template<typename Type>
	Eigen::Matrix<Type, 3, 3> Dcm(const Quaternion<Type> &q)
	{
		Eigen::Matrix<Type, 3, 3> dcm;
		Type a = q(0);
		Type b = q(1);
		Type c = q(2);
		Type d = q(3);
		Type aSq = a * a;
		Type bSq = b * b;
		Type cSq = c * c;
		Type dSq = d * d;
		dcm(0, 0) = aSq + bSq - cSq - dSq;
		dcm(0, 1) = 2 * (b * c - a * d);
		dcm(0, 2) = 2 * (a * c + b * d);
		dcm(1, 0) = 2 * (b * c + a * d);
		dcm(1, 1) = aSq - bSq + cSq - dSq;
		dcm(1, 2) = 2 * (c * d - a * b);
		dcm(2, 0) = 2 * (b * d - a * c);
		dcm(2, 1) = 2 * (a * b + c * d);
		dcm(2, 2) = aSq - bSq - cSq + dSq;
		return dcm;
	}

	typedef Quaternion<float> Quatf;
	typedef Quaternion<float> Quaternionf;
	typedef Quaternion<double>Quatd;
	typedef Quaternion<double>Quaterniond;
