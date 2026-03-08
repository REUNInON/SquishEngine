#include "PhysicsEngine.h"
#include <cmath>

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

/// <summary>
/// Creates a new particle with the specified position and mass, initializes its velocity and force to zero, and returns its index in the particle list. The inverse mass is calculated for efficient physics calculations, handling the case of zero or negative mass appropriately.
/// </summary>
/// <param name="x"></param>
/// <param name="y"></param>
/// <param name="mass"></param>
/// <returns></returns>
uint32_t PhysicsEngine::AddParticle(float x, float y, float mass)
{
	Particle2D p;

	// 1. Initialize position
	p.position[0] = x; p.position[1] = y;

	// 2. Initialize velocity to zero
	p.velocity[0] = 0.0f; p.velocity[1] = 0.0f;

	// 3. Initialize applied force to zero
	p.force[0] = 0.0f; p.force[1] = 0.0f;

	// 4. Set mass
	p.mass = mass;
	p.inverseMass = (mass > 0.0f) * (1.0f / (mass + (mass <= 0.0f))); // mass <= 0 Case Handled: 0 * (1 / (mass + 1))

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

/// <summary>
/// Simulates the motion of particles by applying forces, updating velocities and positions using a symplectic Euler integration method, and applying a global damping factor to simulate energy loss. After integration, forces are reset for the next simulation step.
/// NON-MULTITHREADED: This function is designed to be called from a single thread.
/// </summary>
/// <param name="deltaTime">The time step (in seconds) over which to integrate the simulation.</param>
void PhysicsEngine::Integrate(float deltaTime)
{
	// World Forces
	const float gravity[2] = { 0.0f, -9.81f }; // Earth gravity, applied to all particles
	const float damping = 0.98f; // Global damping factor to simulate energy loss, applied to velocities

	// Apply forces and integrate motion for each particle
	for (auto& p : m_particles)
	{
		// 0. Skip if hardbody (infinite mass)
		if (p.inverseMass <= 0.0f) continue;

		// 1. Apply Newton's Second Law: F = m * a <=> a = F / m <=> a = F * inverseMass. Calculated: Acceleration
		float aX = p.force[0] * p.inverseMass + gravity[0]; // Total acceleration in X (force + gravity)
		float aY = p.force[1] * p.inverseMass + gravity[1]; // Total acceleration in Y (force + gravity)

		// 2. Symplectic Euler Step 1: Update velocity based on acceleration.
		p.velocity[0] += aX * deltaTime; // v = v + a * dt
		p.velocity[1] += aY * deltaTime; // v = v + a * dt

		// 3. Simulate energy loss
		p.velocity[0] *= damping; // v = v * damping
		p.velocity[1] *= damping; // v = v * damping

		// 4. Symplectic Euler Step 2: Update position based on velocity.
		p.position[0] += p.velocity[0] * deltaTime; // x = x + v * dt
		p.position[1] += p.velocity[1] * deltaTime; // y = y + v * dt

		// 5. Reset forces for the next frame
		p.force[0] = 0.0f;
		p.force[1] = 0.0f;
	}
}

void PhysicsEngine::HandleCollisions()
{
	// Floor of the simulation world at y = -0.5. Make it adjustable in the future for different environments.
	const float floorY = -0.5f;

	for (auto& p : m_particles)
	{
		// Check for collision with the floor
		if (p.position[1] < floorY)
		{
			// 1. Hard collision response: Move the particle back to the floor
			p.position[1] = floorY; // Particle is teleported to the floor

			// 2. Stop the partice
			p.velocity[1] = 0.0f; // Stop vertical movement

			// 3. Friction: Halve the horizontal velocity to simulate friction with the floor
			p.velocity[0] *= 0.5f; // Reduce horizontal velocity
		}
	}
}

void PhysicsEngine::SolveConstraints()
{
	// =============================
	// 0. MAIN SOLVER LOOP
	// =============================
	// m_solverIterations: less iterations = softer body, more iterations = stiffer body.
	for (uint32_t i = 0; i < m_solverIterations; ++i)
	{
		// =============================
		// 1. SOLVE DISTANCE CONSTRAINTS
		// =============================
		for (const auto& constraint : m_distanceConstraints)
		{
			
			// 1.0. Reference the two particles involved in the constraint.
			Particle2D& p1 = m_particles[constraint.p1];
			Particle2D& p2 = m_particles[constraint.p2];

			// 1.1. Calculate the current distance between the two particles.
			float deltaX = p2.position[0] - p1.position[0];
			float deltaY = p2.position[1] - p1.position[1];

			float currentDistance = std::sqrt(deltaX * deltaX + deltaY * deltaY); // Pythagorean Theorem

			if (currentDistance == 0.0f) continue; // Skip if particles are in the same position to avoid division by zero.

			// 1.2. Calculate how much the current distance deviates from the rest length.
			float error = currentDistance - constraint.restLength; // Positive if stretched, Negative if compressed

			// 1.3. Calculate the correction amount based on the stiffness of the constraint.
			float totalInverseMass = p1.inverseMass + p2.inverseMass;
			if (totalInverseMass <= 0.0f) continue; // Skip if both particles are immovable to avoid division by zero.
			float correctionAmount = (error / totalInverseMass) * constraint.stiffness; // If stiffness is 1.0, full correction is applied. If stiffness is 0.5, half of the correction is applied etc.


			// 1.4. Calculate the correction vector (direction and magnitude) to apply to each particle.
			float correctionX = (deltaX / currentDistance) * correctionAmount;
			float correctionY = (deltaY / currentDistance) * correctionAmount;

			// 1.5. Apply the correction: P1 moves towards P2, P2 moves towards P1.
			p1.position[0] += correctionX * p1.inverseMass;
			p1.position[1] += correctionY * p1.inverseMass;
			p2.position[0] -= correctionX * p2.inverseMass;
			p2.position[1] -= correctionY * p2.inverseMass;
		}

		// =============================
		// 2. SOLVE AREA CONSTRAINTS
		// =============================
		for (const auto& constraint : m_areaConstraints)
		{
			float currentArea = 0.0f;
			uint32_t count = constraint.particleCount;
			if (count < 3) continue;

			// 2.1. Calculate the current area using the Shoelace formula.
			for (uint32_t j = 0; j < count; ++j)
			{
				uint32_t idx1 = constraint.particleIndices[j];
				uint32_t idx2 = constraint.particleIndices[(j + 1) % count]; // Wrap around to the first particle

				const Particle2D& p1 = m_particles[idx1];
				const Particle2D& p2 = m_particles[idx2];

				// Cross Multiplication for Shoelace formula: (X1 * Y2 - X2 * Y1)
				currentArea += (p1.position[0] * p2.position[1]) - (p2.position[0] * p1.position[1]);
			}
			currentArea *= 0.5f;

			// 2.2. Calculate how much the current area deviates from the rest area.
			float targetArea = constraint.restArea * constraint.pressure; // (Pressure 1 = Normal Volume, 1.2 = Inflated, 0.8 Deflated)
			float error = currentArea - targetArea;
			// if (std::abs(error) < 0.0001f) continue;

			// 2.3. Calculate the gradients (push directions).
			float gradientsX[MAX_AREA_PARTICLES];
			float gradientsY[MAX_AREA_PARTICLES];

			float denominator = 0.0f; // Used to avoid division by zero when normalizing gradients

			for (uint32_t j = 0; j < count; ++j)
			{
				// Particle Indices
				uint32_t prevIdx = constraint.particleIndices[(j + count - 1) % count]; // Wrap around
				uint32_t currentIdx = constraint.particleIndices[j];
				uint32_t nextIdx = constraint.particleIndices[(j + 1) % count]; // Wrap around

				// Particle References
				const Particle2D& pPrev = m_particles[prevIdx];
				const Particle2D& pCurrent = m_particles[currentIdx];
				const Particle2D& pNext = m_particles[nextIdx];

				// Gradient Calculation: (Y_next - Y_prev, X_prev - X_next)
				float gradX = 0.5f * (pNext.position[1] - pPrev.position[1]);
				float gradY = 0.5f * (pPrev.position[0] - pNext.position[0]);

				// Store gradients to temporary arrays
				gradientsX[j] = gradX;
				gradientsY[j] = gradY;

				float weight = pCurrent.inverseMass;
				if (weight > 0.0f)
				{
					denominator += (gradX * gradX + gradY * gradY) * weight;
				}
			}
			// 2.4.
			if (denominator <= 0.0f) continue; // Skip if all particles are immovable to avoid division by zero.

			// PBD Multiplier (Lambda)
			float lambda = -error / denominator;

			for (uint32_t j = 0; j < count; ++j)
			{
				uint32_t idx = constraint.particleIndices[j];
				Particle2D& p = m_particles[idx];
				
				float weight = p.inverseMass;
				if (weight > 0.0f)
				{
					// Apply correction based on the gradient and lambda
					p.position[0] += (gradientsX[j] * lambda) * weight;
					p.position[1] += (gradientsY[j] * lambda) * weight;
				}
			}
			
		}

	}
}

void PhysicsEngine::UpdateVelocities(float deltaTime)
{
}


