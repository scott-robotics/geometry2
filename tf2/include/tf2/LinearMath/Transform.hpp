/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef TF2__LINEARMATH__TRANSFORM_HPP
#define TF2__LINEARMATH__TRANSFORM_HPP


#include "Matrix3x3.hpp"
#include "tf2/visibility_control.h"


namespace tf2
{

#define TransformData TransformDoubleData


/**@brief The Transform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *It can be used in combination with Vector3, Quaternion and Matrix3x3 linear algebra classes. */
class Transform {

  ///Storage for the rotation
	Quaternion m_basis;
  ///Storage for the translation
	Vector3   m_origin;

public:

  /**@brief No initialization constructor */
        TF2_PUBLIC
	Transform() {}
  /**@brief Constructor from Quaternion (optional Vector3 )
   * @param q Rotation from quaternion
   * @param c Translation from Vector (default 0,0,0) */
	explicit TF2SIMD_FORCE_INLINE Transform(const Quaternion& q,
		const Vector3& c = Vector3(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0)))
		: m_basis(q),
		m_origin(c)
	{}

  /**@brief Constructor from Matrix3x3 (optional Vector3)
   * @param b Rotation from Matrix
   * @param c Translation from Vector default (0,0,0)*/
	explicit TF2SIMD_FORCE_INLINE Transform(const Matrix3x3& b,
		const Vector3& c = Vector3(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0)))
		: m_origin(c)
	{
		b.getRotation(m_basis);
	}

  /**@brief Copy constructor */
	TF2SIMD_FORCE_INLINE Transform (const Transform& other)
		: m_basis(other.m_basis),
		m_origin(other.m_origin)
	{
	}
  /**@brief Assignment Operator */
	TF2SIMD_FORCE_INLINE Transform& operator=(const Transform& other)
	{
		m_basis = other.m_basis;
		m_origin = other.m_origin;
		return *this;
	}

  /**@brief Set the current transform as the value of the product of two transforms
   * @param t1 Transform 1
   * @param t2 Transform 2
   * This = Transform1 * Transform2 */
		TF2SIMD_FORCE_INLINE void mult(const Transform& t1, const Transform& t2) {
			m_basis = t1.m_basis * t2.m_basis;
			m_origin = t1(t2.m_origin);
		}

/*		void multInverseLeft(const Transform& t1, const Transform& t2) {
			Vector3 v = t2.m_origin - t1.m_origin;
			m_basis = tf2MultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

/**@brief Return the transform of the vector */
	TF2SIMD_FORCE_INLINE Vector3 operator()(const Vector3& x) const
	{
		return quatRotate(m_basis, x);
	}

  /**@brief Return the transform of the vector */
	TF2SIMD_FORCE_INLINE Vector3 operator*(const Vector3& x) const
	{
		return (*this)(x);
	}

  /**@brief Return the transform of the Quaternion */
	TF2SIMD_FORCE_INLINE Quaternion operator*(const Quaternion& q) const
	{
		return getRotation() * q;
	}

  /**@brief Return the basis matrix for the rotation */
	TF2SIMD_FORCE_INLINE Matrix3x3        getBasis()    const { return Matrix3x3(m_basis); }

  /**@brief Return the origin vector translation */
	TF2SIMD_FORCE_INLINE Vector3&         getOrigin()         { return m_origin; }
  /**@brief Return the origin vector translation */
	TF2SIMD_FORCE_INLINE const Vector3&   getOrigin()   const { return m_origin; }

  /**@brief Return a quaternion representing the rotation */
    TF2SIMD_FORCE_INLINE TF2_PUBLIC
	const Quaternion& getRotation() const { return m_basis; }


  /**@brief Set from an array
   * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
        TF2_PUBLIC
	void setFromOpenGLMatrix(const tf2Scalar *m)
	{
		Matrix3x3 mat;
		mat.setFromOpenGLSubMatrix(m);
		mat.getRotation(m_basis);
		m_origin.setValue(m[12],m[13],m[14]);
	}

  /**@brief Fill an array representation
   * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
        TF2_PUBLIC
	void getOpenGLMatrix(tf2Scalar *m) const
	{
		Matrix3x3(m_basis).getOpenGLSubMatrix(m);
		m[12] = m_origin.x();
		m[13] = m_origin.y();
		m[14] = m_origin.z();
		m[15] = tf2Scalar(1.0);
	}

  /**@brief Set the translational element
   * @param origin The vector to set the translation to */
	TF2SIMD_FORCE_INLINE void setOrigin(const Vector3& origin)
	{
		m_origin = origin;
	}

	TF2SIMD_FORCE_INLINE Vector3 invXform(const Vector3& inVec) const;


  /**@brief Set the rotational element by Matrix3x3 */
	TF2SIMD_FORCE_INLINE void setBasis(const Matrix3x3& basis)
	{
		basis.getRotation(m_basis);
	}

  /**@brief Set the rotational element by Quaternion */
	TF2SIMD_FORCE_INLINE void setRotation(const Quaternion& q)
	{
		m_basis = q;
	}


  /**@brief Set this transformation to the identity */
        TF2_PUBLIC
	void setIdentity()
	{
		m_basis.setEuler(0, 0, 0);
		m_origin.setValue(tf2Scalar(0.0), tf2Scalar(0.0), tf2Scalar(0.0));
	}

  /**@brief Multiply this Transform by another(this = this * another)
   * @param t The other transform */
        TF2_PUBLIC
	Transform& operator*=(const Transform& t)
	{
		m_origin += quatRotate(m_basis, t.m_origin);
		m_basis *= t.m_basis;
		return *this;
	}

  /**@brief Return the inverse of this transform */
        TF2_PUBLIC
	Transform inverse() const
	{
		auto inv = m_basis.inverse();
		return Transform(inv, quatRotate(m_basis, -m_origin));
	}

  /**@brief Return the inverse of this transform times the other transform
   * @param t The other transform
   * return this.inverse() * the other */
        TF2_PUBLIC
	Transform inverseTimes(const Transform& t) const;

  /**@brief Return the product of this transform and the other */
        TF2_PUBLIC
	Transform operator*(const Transform& t) const;

  /**@brief Return an identity transform */
        TF2_PUBLIC
	static const Transform&	getIdentity()
	{
		static const Transform identityTransform(Matrix3x3::getIdentity());
		return identityTransform;
	}

        TF2_PUBLIC
	void	serialize(struct	TransformData& dataOut) const;

        TF2_PUBLIC
	void	serializeFloat(struct	TransformFloatData& dataOut) const;

        TF2_PUBLIC
	void	deSerialize(const struct	TransformData& dataIn);

        TF2_PUBLIC
	void	deSerializeDouble(const struct	TransformDoubleData& dataIn);

        TF2_PUBLIC
	void	deSerializeFloat(const struct	TransformFloatData& dataIn);

};


TF2SIMD_FORCE_INLINE Vector3
Transform::invXform(const Vector3& inVec) const
{
	Vector3 v = inVec - m_origin;
	return quatRotate(m_basis.inverse(), v);
}

TF2SIMD_FORCE_INLINE Transform
Transform::inverseTimes(const Transform& t) const
{
	Vector3 v = t.getOrigin() - m_origin;
	auto inv = m_basis.inverse();
	return Transform(inv * t.m_basis, quatRotate(inv, v));
}

TF2SIMD_FORCE_INLINE Transform
Transform::operator*(const Transform& t) const
{
	return Transform(m_basis * t.m_basis,
		(*this)(t.m_origin));
}

/**@brief Test if two transforms have all elements equal */
TF2SIMD_FORCE_INLINE bool operator==(const Transform& t1, const Transform& t2)
{
   return ( t1.getRotation()  == t2.getRotation() &&
            t1.getOrigin() == t2.getOrigin() );
}


///for serialization
struct	TransformFloatData
{
	Matrix3x3FloatData	m_basis;
	Vector3FloatData	m_origin;
};

struct	TransformDoubleData
{
	Matrix3x3DoubleData	m_basis;
	Vector3DoubleData	m_origin;
};

TF2SIMD_FORCE_INLINE	void	Transform::serialize(TransformData& dataOut) const
{
	Matrix3x3(m_basis).serialize(dataOut.m_basis);
	m_origin.serialize(dataOut.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::serializeFloat(TransformFloatData& dataOut) const
{
	Matrix3x3(m_basis).serializeFloat(dataOut.m_basis);
	m_origin.serializeFloat(dataOut.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::deSerialize(const TransformData& dataIn)
{
	Matrix3x3 mat;
	mat.deSerialize(dataIn.m_basis);
	mat.getRotation(m_basis);
	m_origin.deSerialize(dataIn.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::deSerializeFloat(const TransformFloatData& dataIn)
{
	Matrix3x3 mat;
	mat.deSerializeFloat(dataIn.m_basis);
	mat.getRotation(m_basis);
	m_origin.deSerializeFloat(dataIn.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::deSerializeDouble(const TransformDoubleData& dataIn)
{
	Matrix3x3 mat;
	mat.deSerializeDouble(dataIn.m_basis);
	m_origin.deSerializeDouble(dataIn.m_origin);
}

}

#endif  // TF2__LINEARMATH__TRANSFORM_HPP
