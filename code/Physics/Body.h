//
//	Body.h
//
#pragma once
#include "../Math/Vector.h"
#include "../Math/Quat.h"
#include "../Math/Matrix.h"
#include "../Math/Bounds.h"
#include "Shapes.h"

#include "../Renderer/model.h"
#include "../Renderer/shader.h"

/*
====================================================
Body
====================================================
*/
class Body {
public:
	Body();

	void Update(const float dt_sec);
	void ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse);
	void ApplyImpulseLinear(const Vec3& impulse);
	void ApplyImpulseAngular(const Vec3& impulse);
	Vec3 CenterOfMass() const;
	Mat3 CalculateInertiaTensor() const;
public:
	Vec3		m_position;
	Quat		m_orientation;
	Vec3		m_linearVelocity;
	Vec3        m_angularVelocity;

	float		m_radius;
	float		m_mass;
	float		m_elasticity;
	float		m_friction;
	Shape *		m_shape;
};