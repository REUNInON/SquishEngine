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

constexpr uint32_t MAX_AREA_PARTICLES = 16;
struct AreaConstraint // 80-Byte Block (76 Bytes used, 4 Bytes padding for alignment)
{
	uint32_t particleIndices[MAX_AREA_PARTICLES]; // Not a std::vector to avoid dynamic memory allocation, fixed size sent to GPU via HLSL

	uint32_t particleCount; // Number of particles in the area constraint
	float restArea; // The ideal area that the particles should maintain, calculated with Shoelace formula
	float pressure; // Internal pressure (0.8: deflated, 1.0: normal, 1.2: inflated)
};

// TODO: Physics Engine Class Implementation
class PhysicsEngine
{
public:
	// ===========================
	// Constructor and Destructor
	// ===========================
	PhysicsEngine();
	~PhysicsEngine();

	// ===========================
	// Particle Management
	// ===========================

	// Adds a new particle to the simulation with the specified position and mass, returning its index.
	uint32_t AddParticle(float x, float y, float mass);

	// Adds a connection between two particles with a specified rest length and stiffness, simulating a spring-like behavior.
	void AddDistanceConstraint(uint32_t p1, uint32_t p2, float restLength, float stiffness);

	// Adds an area constraint that maintains a specified area for a group of particles.
	void AddAreaConstraint(const std::vector<uint32_t>& particleIndices, float restArea, float pressure);

	// ===========================
	// Simulation Control
	// ===========================

	// Advances the simulation by deltaTime seconds, runs every frame.
	void StepSimulation(float deltaTime);

	// ===========================
	// Getter for Rendering
	// ===========================
	const std::vector<Particle2D>& GetParticles() const { return m_particles; }

private:
	uint32_t m_solverIterations = 10; // Default stubbornness of the system, more iterations = stiffer body, less iterations = softer body.
	// ===========================
	// Internal Simulation Steps
	// ===========================
	void Integrate(float deltaTime); // Updates particle positions and velocities based on current forces and velocities.
	void HandleCollisions(); // Detects and resolves collisions between particles and with boundaries.
	void SolveConstraints(); // Solves all distance and area constraints to maintain the physical properties of the system.
	void UpdateVelocities(float deltaTime); // Updates particle velocities based on the changes in position after constraint solving.

	// =============================
	// Memory Layout
	// =============================
	// Cache-friendly data structures for particles and constraints, optimized for GPU processing and minimizing cache misses.
	std::vector<Particle2D> m_particles; // Stores all particles in the simulation
	std::vector<DistanceConstraint> m_distanceConstraints; // Stores all distance constraints between particles
	std::vector<AreaConstraint> m_areaConstraints; // Stores all area constraints for groups of particles

};

