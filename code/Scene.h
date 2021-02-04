//
//  Scene.h
//
#pragma once
#include <vector>

#include "Physics/Shapes.h"
#include "Physics/Body.h"

/*
====================================================
Scene
====================================================
*/

struct psuedoBody_t
{
	int id;
	float value;
	bool isMin;
};

struct collisionPair_t
{
	int a;
	int b;

	bool operator == (const collisionPair_t& rhs) const
	{
		return (((a == rhs.a) && (b == rhs.b)) || ((a == rhs.b) && (b == rhs.a)));
	}

	bool operator != (const collisionPair_t& rhs) const
	{
		return !(*this == rhs);
	}
};

struct contact_t
{
	Vec3 ptOnA_WorldSpace;
	Vec3 ptOnB_WorldSpace;
	Vec3 ptOnA_LocalSpace;
	Vec3 ptOnB_LocalSpave;

	Vec3 normal;
	float seperationDistance;
	float timeOfImpact;

	Body* bodyA;
	Body* bodyB;
};

class Scene {
public:
	Scene() { m_bodies.reserve( 128 ); }
	~Scene();

	void Reset();
	void Initialize();
	void Update( const float dt_sec );	

	std::vector< Body > m_bodies;
};

