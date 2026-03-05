#pragma once
#include <vector>
#include <cstdint>

/// <summary>
/// Represents a 2D particle with physical properties for simulation.
/// </summary>
struct Particle2D // 32-Byte Block
{
	// 16-Byte Block 1
	float position[2]; // 2 Dimensions (x, y)
	float velocity[2]; // 2 Dimensions (vx, vy), Direction and Speed

	// 16-Byte Block 2
	float force[2]; // 2 Dimensions (fx, fy), Force applied to the particle
	float mass; // Weight of the particle
	float inverseMass; // 1 / mass, Used for calculations to avoid division
};

/// <summary>
/// Defines a constraint that maintains a fixed distance between two particles, simulating a spring-like connection.
/// </summary>
struct DistanceConstraint // 16-Byte Block
{
	uint32_t p1; // Index of the first particle
	uint32_t p2; // Index of the second particle

	float restLength; // The desired distance between the two particles

	float stiffness; // How strongly the constraint is enforced (0: gel-like to 1: rigid)
};

class PhysicsEngine
{
};

