//
//  Scene.cpp
//
#include "Scene.h"
#include <iostream>
/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize() {
	Body body;
	
	for (int x = 0; x < 8; x++)
	{
		for (int y = 0; y < 8; y++)
		{
			float radius = 0.5f;
			float xx = float(x - 1) * radius * 5.0f;
			float yy = float(y - 1) * radius * 5.0f;
			body.m_position = Vec3(xx, yy, 10.0f + yy);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity = Vec3(0, 0, 2);
			body.m_mass = 0.5f;
			body.m_radius = radius;
			body.m_elasticity = 0.3f;
			body.m_friction = 0.4f;
			body.m_shape = new ShapeSphere(body.m_radius);
			m_bodies.push_back(body);

		}
	}
	
	/*
	body.m_position = Vec3(0.0f, 0.0f, 5.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(1, 0, 0);
	body.m_mass = 1.0f;
	body.m_radius = 1.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.2f;
	body.m_shape = new ShapeSphere(body.m_radius);
	m_bodies.push_back(body);
	*/

	/*
	body.m_position = Vec3(0, 0, -1000);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_mass = 0.0f;
	body.m_radius = 1000.0f;
	body.m_elasticity = 0.3f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeSphere(body.m_radius);
	m_bodies.push_back(body);
	*/

	for (int x = 0; x < 3; x++)
	{
		for (int y = 0; y < 3; y++)
		{
			float radius = 80.0f;
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx, yy, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity = Vec3(0, 0, 0);
			body.m_mass = 0.0f;
			body.m_radius = 1000.0f;
			body.m_elasticity = 1.2f;
			body.m_friction = 0.6f;
			body.m_radius = radius;
			body.m_shape = new ShapeSphere(body.m_radius);
			m_bodies.push_back(body);
		}
	}
}

/*
====================================================
Scene::Update
====================================================
*/

bool RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float sphereRadius, float& t1, float& t2)
{
	const Vec3 m = sphereCenter - rayStart;
	const float a = rayDir.Dot(rayDir);
	const float b = m.Dot(rayDir);
	const float c = m.Dot(m) - sphereRadius * sphereRadius;

	const float delta = b * b - a * c;
	const float invA = 1.0f / a;

	if (delta < 0)
		return false;

	const float deltaRoot = sqrtf(delta);
	t1 = invA * (b - deltaRoot);
	t2 = invA * (b + deltaRoot);

	return true;
}

bool SphereSphereDynamic(const ShapeSphere* shapeA, const ShapeSphere* shapeB, const Vec3& posA, const Vec3& posB,
	const Vec3& velA, const Vec3& velB, const float dt, Vec3& ptOnA, Vec3& ptOnB, float& toi)
{
	const Vec3 relativeVelocity = velA - velB;

	const Vec3 startPointA = posA;
	const Vec3 endPointA = posA + relativeVelocity * dt;
	const Vec3 rayDir = endPointA - startPointA;

	float t0 = 0.0f;
	float t1 = 0.0f;

	if (rayDir.GetLengthSqr() < 0.001f * 0.001f)
	{
		Vec3 ab = posB - posA;
		float radius = shapeA->m_radius + shapeB->m_radius + 0.001f;
		if (ab.GetLengthSqr() > radius * radius)
			return false;
	}
	else if (!RaySphere(posA, rayDir, posB, shapeA->m_radius + shapeB->m_radius, t0, t1))
		return false;

	t0 *= dt;
	t1 *= dt;

	if (t1 < 0.0f)
		return false;

	toi = (t0 < 0.0f) ? 0.0f : t0;

	if (toi > dt)
		return false;

	Vec3 newPosA = posA + velA * toi;
	Vec3 newPosB = posB + velB * toi;
	Vec3 ab = newPosB - newPosA;
	ab.Normalize();

	ptOnA = newPosA + ab * shapeA->m_radius;
	ptOnB = newPosB - ab * shapeB->m_radius;

	return true;
}

inline bool IntersectTestSpheretoSphere(Body* bodyA,Body* bodyB,contact_t& contact,const float dt)
{
	if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE)
	{
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		Vec3 velocityA = bodyA->m_linearVelocity;
		Vec3 velocityB = bodyB->m_linearVelocity;



		if (SphereSphereDynamic(sphereA, sphereB, posA, posB, velocityA, velocityB, dt, contact.ptOnA_WorldSpace,
			contact.ptOnB_WorldSpace, contact.timeOfImpact))
		{
			contact.bodyA = bodyA;
			contact.bodyB = bodyB;
			
			bodyA->Update(contact.timeOfImpact);
			bodyB->Update(contact.timeOfImpact);

			contact.normal = bodyA->m_position - bodyB->m_position;
			contact.normal.Normalize();

			bodyA->Update(-contact.timeOfImpact);
			bodyB->Update(-contact.timeOfImpact);

			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			contact.seperationDistance = r;
			return true;
		}
	}
	return false;
}

void ResolveContact(contact_t & contact)
{
	Body* bodyA = contact.bodyA;
	Body* bodyB = contact.bodyB;

	if (bodyA->m_mass == 0.0f && bodyB->m_mass == 0.0f )
		return;


	const float invMassA = bodyA->m_mass;
	const float invMassB = bodyB->m_mass;

	const float elasticityA = bodyA->m_elasticity;
	const float elasticityB = bodyB->m_elasticity;
	const float elasticity = elasticityA * elasticityB;

	const Vec3 ra = contact.ptOnA_WorldSpace - bodyA->CenterOfMass();
	const Vec3 rb = contact.ptOnB_WorldSpace - bodyB->CenterOfMass();

	const Mat3 inertiaTensorA = bodyA->m_shape->InertiaTensor();
	const Mat3 inertiaTensorB = bodyB->m_shape->InertiaTensor();

	Mat3 invInvertiaTensorA = inertiaTensorA.Inverse() * invMassA;
	Mat3 invInvertiaTensorB = inertiaTensorB.Inverse() * invMassB;

	const Mat3 orientationA = bodyA->m_orientation.ToMat3();
	const Mat3 orientationB = bodyB->m_orientation.ToMat3();

	const Mat3 invWorldInertiaTensorA = orientationA * invInvertiaTensorA * orientationA.Transpose();
	const Mat3 invWorldInertiaTensorB = orientationB * invInvertiaTensorB * orientationB.Transpose();

	const Vec3& normal = contact.normal;

	const Vec3 angularJA = (invInvertiaTensorA * ra.Cross(normal)).Cross(ra);
	const Vec3 angularJB = (invInvertiaTensorB * rb.Cross(normal)).Cross(rb);
	const float angularFactor = (angularJA + angularJB).Dot(normal);

	const Vec3 velocityA = bodyA->m_linearVelocity + bodyA->m_angularVelocity.Cross(ra);
	const Vec3 velocityB = bodyB->m_linearVelocity + bodyB->m_angularVelocity.Cross(rb);

	const Vec3 vab = velocityA - velocityB;
	float impulse = (1.0f + elasticity) * vab.Dot(normal) / (invMassA + invMassB + angularFactor);
	Vec3 vectorImpulse = Vec3(0,0,impulse);
	

	bodyA->ApplyImpulse(contact.ptOnA_WorldSpace,vectorImpulse * -1.0f);
	bodyB->ApplyImpulse(contact.ptOnB_WorldSpace,vectorImpulse * 1.0f);

	const float frictionA = bodyA->m_friction;
	const float frictionB = bodyB->m_friction;
	const float friction = frictionA * frictionB;

	const Vec3 velNormal = normal * normal.Dot(vab);
	const Vec3 velTangent = vab - velNormal;

	Vec3 relativeVelTang = velTangent;
	relativeVelTang.Normalize();

	const Vec3 inertiaA = (invWorldInertiaTensorA * ra.Cross(relativeVelTang)).Cross(ra);
	const Vec3 inertiaB = (invWorldInertiaTensorB * rb.Cross(relativeVelTang)).Cross(rb);
	const float invInertia = (inertiaA + inertiaB).Dot(relativeVelTang);

	const float reducedMass = 1.0f / (bodyA->m_mass + bodyB->m_mass + invInertia);
	const Vec3 impulseFriction = velTangent * friction * reducedMass;
	
	bodyA->ApplyImpulse(contact.ptOnA_WorldSpace, impulseFriction * -1.0f);
	bodyB->ApplyImpulse(contact.ptOnB_WorldSpace, impulseFriction * 1.0f);
	
	
	if (contact.timeOfImpact)
	{
		const float tA = bodyA->m_mass / (bodyA->m_mass + bodyB->m_mass);
		const float tB = bodyB->m_mass / (bodyA->m_mass + bodyB->m_mass);
		const Vec3 dist = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
		bodyA->m_position += dist * tA;
		bodyB->m_position -= dist * tB;
	}
	
}

int CompareContacts(const void* p1, const void* p2)
{
	contact_t a = *(contact_t*)p1;
	contact_t b = *(contact_t*)p2;

	if (a.timeOfImpact < b.timeOfImpact)
		return -1;
	else if (a.timeOfImpact == b.timeOfImpact)
		return 0;
	else
		return 1;
}

int CompareSort(const void* a, const void* b)
{
	const psuedoBody_t* pA = (const psuedoBody_t*)a;
	const psuedoBody_t* pB = (const psuedoBody_t*)b;

	if (pA->value < pB->value)
		return -1;

	return 1;
}

void SortBodiesBounds(const Body* bodies, const int num, psuedoBody_t* sortedArray, const float dt_sec)
{
	Vec3 axis = Vec3(1, 1, 1);
	axis.Normalize();

	for (int i = 0; i < num; i++)
	{
		const Body& body = bodies[i];
		Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

		bounds.Expand(bounds.mins + body.m_linearVelocity * dt_sec);
		bounds.Expand(bounds.maxs + body.m_linearVelocity * dt_sec);

		const float epsilon = 0.01f;
		bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
		bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);

		sortedArray[i * 2 + 0].id = i;
		sortedArray[i * 2 + 0].value = axis.Dot(bounds.mins);
		sortedArray[i * 2 + 0].isMin = true;


		sortedArray[i * 2 + 1].id = i;
		sortedArray[i * 2 + 1].value = axis.Dot(bounds.maxs);
		sortedArray[i * 2 + 1].isMin = false;

	}

	qsort(sortedArray, num * 2, sizeof(psuedoBody_t), CompareSort);
}


void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const psuedoBody_t* sortedBodies, const int num)
{
	collisionPairs.clear();

	for (int i = 0; i < num * 2; i++)
	{
		const psuedoBody_t& a = sortedBodies[i];
		if (!a.isMin)
			continue;

		collisionPair_t pair;
		pair.a = a.id;

		for (int j = i + 1; j < num * 2; j++)
		{
			const psuedoBody_t& b = sortedBodies[j];

			if (b.id == a.id)
				break;


			if (!b.isMin)
				continue;

			pair.b = b.id;
			collisionPairs.push_back(pair);
		}
	}
}

void SweepAndPrune1D(const Body* bodies, const int num, std::vector < collisionPair_t>& collisionPairs,
	const float dt_Sec)
{
	psuedoBody_t* sortedBodies = (psuedoBody_t*)alloca(sizeof(psuedoBody_t) * num * 2);

	SortBodiesBounds(bodies, num, sortedBodies, dt_Sec);
	BuildPairs(collisionPairs, sortedBodies, num);
}

void BroadPhase(const Body* bodies, const int num, std::vector<collisionPair_t>& collisionPairs, const float dt_sec)
{
	collisionPairs.clear();

	SweepAndPrune1D(bodies, num, collisionPairs, dt_sec);
}

void Scene::Update( const float dt_sec ) {
	
	for (int i = 0; i < m_bodies.size(); i++)
	{
		Body* body = &m_bodies[i];

		float mass = 1.0f / body->m_mass;
		Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
		body->ApplyImpulseLinear(impulseGravity);
	}

	std::vector<collisionPair_t> collisionPairs;
	BroadPhase(m_bodies.data(), (int)m_bodies.size(), collisionPairs, dt_sec);

	int numberOfContacts = 0;
	const int maxContacts = m_bodies.size() * m_bodies.size();
	contact_t* contacts = (contact_t*)alloca(sizeof(contact_t) * maxContacts);

	for (int i = 0; i < collisionPairs.size(); i++)
	{
		const collisionPair_t& pair = collisionPairs[i];
		Body* bodyA = &m_bodies[pair.b];
		Body* bodyB = &m_bodies[pair.a];

		if (bodyA->m_mass == 0.0f && bodyB->m_mass == 0.0f)
			continue;

		contact_t contact;
		if (IntersectTestSpheretoSphere(bodyA, bodyB, contact, dt_sec))
		{
			//ResolveContact(contact);
			contacts[numberOfContacts] = contact;
			numberOfContacts++;
		}
	}

	if (numberOfContacts > 1)
		qsort(contacts, numberOfContacts, sizeof(contact_t), CompareContacts);

	float accumulatedTime = 0.0f;
	for (int i = 0; i < numberOfContacts; i++)
	{
		contact_t& contact = contacts[i];
		const float dt = contact.timeOfImpact - accumulatedTime;

		for (int j = 0; j < m_bodies.size(); j++)
			m_bodies[j].Update(dt);


		ResolveContact(contact);
		accumulatedTime += dt;
	}

	const float timeRemaining = dt_sec - accumulatedTime;
	if (timeRemaining > 0.0f)
	{
		for (int i = 0; i < m_bodies.size(); i++)
		{
			m_bodies[i].Update(timeRemaining);

		}
	}	
}