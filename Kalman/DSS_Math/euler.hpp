/**
 * @file euler.hpp
 *
 * All rotations and axis systems follow the right-hand rule
 *
 * An instance of this class defines a rotation from coordinate frame1 to coordinate frame2.
 * It follows the convention of a 3-2-1 intrinsic Tait-Bryan rotation sequence.
 * In order to go from frame 1 to frame 2 we apply the following rotations consecutively.
 * 1) We rotate about our initial Z axis by an angle of yaw.
 * 2) We rotate about the newly created Y' axis by an angle of pitch.
 * 3) We rotate about the newly created X'' axis by an angle of roll.
 *
 */

#pragma once

#include"../Eigen/Dense"
#include<cmath>


#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

	/**
	 * Euler angles class
	 *
	 * This class describes the rotation from frame 1
	 * to frame 2 via 3-2-1 intrinsic Tait-Bryan rotation sequence.
	 */
	template<typename Type>
	class Euler : public Eigen::Matrix<Type, 3, 1>
	{
	public:
		/**
		 * Standard constructor
		 */
		Euler() : Eigen::Matrix<Type, 3, 1>()
		{
			Euler& euler = *this;
			euler.setZero();
		}

		/**
		 * Copy constructor
		 *
		 * @param other vector to copy
		 */
		Euler(const Eigen::Matrix<Type, 3, 1> &other) :
			Eigen::Matrix<Type, 3, 1>(other)
		{
		}

		/**
		 * Constructor from euler angles
		 *
		 * Instance is initialized from an 3-2-1 intrinsic Tait-Bryan
		 * rotation sequence representing transformation from frame 1
		 * to frame 2.
		 *
		 * @param roll rotation angle about X axis
		 * @param pitch rotation angle about Y axis
		 * @param yaw rotation angle about Z axis
		 */
		Euler(Type roll, Type pitch, Type yaw) :
			Eigen::Matrix<Type, 3, 1>(roll, pitch, yaw)
		{

		}

		/**
		 * Constructor from rotation matrix
		 *
		 * Instance is set from rotation matrix representing transformation from
		 * frame 2 to frame 1.
		 * This instance will hold the angles defining the 3-2-1 intrinsic
		 * Tait-Bryan rotation sequence from frame 1 to frame 2.
		 *
		 * @param rotation matrix Direction cosine matrix
		*/
		Euler(const Eigen::Matrix<Type, 3, 3> &dcm) :Eigen::Matrix<Type, 3, 1>()
		{
			Euler &euler = *this;
			Type rollval = Type(atan2(dcm(2, 1), dcm(2, 2)));
			Type pitchval = Type(asin(-dcm(2, 0)));
			Type yawval = Type(atan2(dcm(1, 0), dcm(0, 0)));
			Type pi = Type(M_PI);

			if (Type(fabs(pitchval - pi / Type(2))) < Type(1.0e-3)) {
				rollval = Type(0.0);
				yawval = Type(atan2(dcm(1, 2), dcm(0, 2)));

			}
			else if (Type(fabs(pitchval + pi / Type(2))) < Type(1.0e-3)) {
				rollval = Type(0.0);
				yawval = Type(atan2(-dcm(1, 2), -dcm(0, 2)));
			}

			euler(0) = rollval;
			euler(1) = pitchval;
			euler(2) = yawval;
		}

		inline Type GetRoll() const
		{
			return (*this)(0);
		}
		inline Type GetPitch() const
		{
			return (*this)(1);
		}
		inline Type GetYaw() const
		{
			return (*this)(2);
		}

		inline Type &GetRoll()
		{
			return (*this)(0);
		}
		inline Type &GetPitch()
		{
			return (*this)(1);
		}
		inline Type &GetYaw()
		{
			return (*this)(2);
		}
	};

	template<typename Type>
	Eigen::Matrix<Type, 3, 3> Dcm(const Euler<Type> &euler)
	{
		Eigen::Matrix<Type, 3, 3> dcm;
		Type cosroll = Type(cos(euler.GetRoll()));
		Type sinroll = Type(sin(euler.GetRoll()));
		Type cospitch = Type(cos(euler.GetPitch()));
		Type sinpitch = Type(sin(euler.GetPitch()));
		Type cosyaw = Type(cos(euler.GetYaw()));
		Type sinyaw = Type(sin(euler.GetYaw()));

		dcm(0, 0) = cospitch * cosyaw;
		dcm(0, 1) = -cosroll * sinyaw + sinroll * sinpitch * cosyaw;
		dcm(0, 2) = sinroll * sinyaw + cosroll * sinpitch * cosyaw;

		dcm(1, 0) = cospitch * sinyaw;
		dcm(1, 1) = cosroll * cosyaw + sinroll * sinpitch * sinyaw;
		dcm(1, 2) = -sinroll * cosyaw + cosroll * sinpitch * sinyaw;

		dcm(2, 0) = -sinpitch;
		dcm(2, 1) = sinroll * cospitch;
		dcm(2, 2) = cosroll * cospitch;
		return dcm;
	}

	typedef Euler<float> Eulerf;
	typedef Euler<double> Eulerd;
