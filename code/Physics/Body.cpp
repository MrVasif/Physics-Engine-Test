//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
m_position( 0.0f ),
m_radius(0.0f),
m_orientation( 0.0f, 0.0f, 0.0f, 1.0f ),
m_shape( NULL ) {
}

void Body::Update(const float dt_sec)
{
	m_position += m_linearVelocity * dt_sec;

	Vec3 positionCM = CenterOfMass();
	Vec3 cmToPos = m_position - positionCM;

	Mat3 orientation = m_orientation.ToMat3();
	Mat3 inertiaTensor = orientation * m_shape->InertiaTensor() * orientation.Transpose();
	Vec3 alpha = inertiaTensor.Inverse() * (m_angularVelocity.Cross(inertiaTensor * m_angularVelocity));
	m_angularVelocity += alpha * dt_sec;

	Vec3 dAngle = m_angularVelocity * dt_sec;
	Quat dq = Quat(dAngle, dAngle.GetMagnitude());
	m_orientation = dq * m_orientation;
	m_orientation.Normalize();

	m_position = positionCM + dq.RotatePoint(cmToPos);
}

void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& impulse)
{
	if (m_mass == 0.0f)
		return;

	ApplyImpulseLinear(impulse);

	Vec3 position = CenterOfMass();
	Vec3 r = impulsePoint - position;
	Vec3 dL = r.Cross(impulse);
	ApplyImpulseAngular(dL);
}

void Body::ApplyImpulseLinear(const Vec3& impulse)
{
	if (m_mass == 0.0f)
		return;

	m_linearVelocity += impulse * m_mass;
}

void Body::ApplyImpulseAngular(const Vec3& impulse)
{
	if (m_mass == 0.0f)
		return;

	m_angularVelocity += CalculateInertiaTensor() * impulse;

	const float maxAngularSpeed = 30.0f;

	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed)
	{
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}

Vec3 Body::CenterOfMass() const
{
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}

Mat3 Body::CalculateInertiaTensor() const
{
	Mat3 orientation = m_orientation.ToMat3();
	Mat3 transposeOrient = orientation.Transpose();

	return orientation * m_shape->InertiaTensor() * transposeOrient * m_mass;
}
