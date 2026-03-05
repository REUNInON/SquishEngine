#include "PhysicsEngine.h"

// ===========================
// Constructor and Destructor
// ===========================
PhysicsEngine::PhysicsEngine()
{
}

PhysicsEngine::~PhysicsEngine()
{
}

// ===========================
// Particle Management
// ===========================

uint32_t PhysicsEngine::AddParticle(float x, float y, float mass)
{
	return 0;
}

void PhysicsEngine::AddDistanceConstraint(uint32_t p1, uint32_t p2, float restLength, float stiffness)
{
}

void PhysicsEngine::AddAreaConstraint(const std::vector<uint32_t>& particleIndices, float restArea, float pressure)
{
}

// ===========================
// Simulation Control
// ===========================

void PhysicsEngine::StepSimulation(float deltaTime)
{
}

// ===========================
// Internal Simulation Steps
// ===========================

void PhysicsEngine::Integrate(float deltaTime)
{
}

void PhysicsEngine::HandleCollisions()
{
}

void PhysicsEngine::SolveConstraints()
{
}

void PhysicsEngine::UpdateVelocities(float deltaTime)
{
}


