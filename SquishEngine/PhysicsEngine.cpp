#include "PhysicsEngine.h"
#include <cmath>

// ===========================
// Simulation Control
// ===========================

/// <summary>
/// Main simulation step that advances the physics simulation by a specified time step (deltaTime).
/// </summary>
/// <param name="deltaTime">The time step (in seconds) over which to advance the simulation.</param>
void PhysicsEngine::StepSimulation(float deltaTime)
{
	Integrate(deltaTime);
	SolveConstraints();
	HandleCollisions();
	UpdateVelocities(deltaTime);
}

// ===========================
// Particle Management
// ===========================

/// <summary>
/// Creates a new particle with the specified position and mass, initializes its velocity and force to zero, and returns its index in the particle list.
/// The inverse mass is calculated for efficient physics calculations, handling the case of zero or negative mass appropriately.
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
	
	// 5. Add to particle list and return index
	m_particles.push_back(p);

	return static_cast<uint32_t>(m_particles.size() - 1);
}

/// <summary>
/// Adds a distance constraint between two particles identified by their indices, specifying the desired rest length and stiffness of the constraint.
/// The constraint is stored in a list for later processing during the simulation step to maintain the specified distance between the particles.
/// Stiffness controls how strongly the constraint is enforced, allowing for soft or rigid connections.
/// </summary>
/// <param name="p1"></param>
/// <param name="p2"></param>
/// <param name="restLength"></param>
/// <param name="stiffness"></param>
void PhysicsEngine::AddDistanceConstraint(uint32_t p1, uint32_t p2, float restLength, float stiffness)
{
	DistanceConstraint constraint;
	constraint.p1 = p1;
	constraint.p2 = p2;
	constraint.restLength = restLength;
	constraint.stiffness = stiffness;

	m_distanceConstraints.push_back(constraint);
}

/// <summary>
/// Adds an area constraint that maintains a specified area for a group of particles defined by their indices.
/// The rest area is the ideal area calculated using the Shoelace formula, and the pressure parameter controls how much the particles should resist changes to this area (0.8: deflated, 1.0: normal, 1.2: inflated).
/// The constraint is stored in a list for later processing during the simulation step to maintain the specified area among the particles.
/// The function checks that the number of particles is between 3 and a defined maximum to ensure valid area constraints.
/// </summary>
/// <param name="particleIndices"></param>
/// <param name="restArea"></param>
/// <param name="pressure"></param>
void PhysicsEngine::AddAreaConstraint(const std::vector<uint32_t>& particleIndices, float restArea, float pressure)
{
	if (particleIndices.size() < 3 || particleIndices.size() > MAX_AREA_PARTICLES) return; // Area constraint requires at least 3 particles and has a maximum limit
	AreaConstraint constraint;
	constraint.particleCount = static_cast<uint32_t>(particleIndices.size());
	constraint.restArea = restArea;
	constraint.pressure = pressure;
	// Copy particle indices to the fixed-size array in the constraint
	for (size_t i = 0; i < particleIndices.size(); ++i)
	{
		constraint.particleIndices[i] = particleIndices[i];
	}
	m_areaConstraints.push_back(constraint);
}

// ===========================
// Internal Simulation Steps
// ===========================

/// <summary>
/// Simulates the motion of particles by applying forces, updating velocities and positions using a symplectic Euler integration method, and applying a global damping factor to simulate energy loss.
/// After integration, forces are reset for the next simulation step.
/// NON-MULTITHREADED: This function is designed to be called from a single thread.
/// </summary>
/// <param name="deltaTime">The time step (in seconds) over which to integrate the simulation.</param>
void PhysicsEngine::Integrate(float deltaTime)
{
	// World Forces
	const float gravity[2] = { 0.0f, -9.81f }; // Earth gravity, applied to all particles
	const float damping = 0.99f; // Global damping factor to simulate energy loss, applied to velocities

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

		// Store previous position for velocity update after constraint solving.
		p.prevPosition[0] = p.position[0];
		p.prevPosition[1] = p.position[1];

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
	const float floorY = -1.0f;
	const float friction = 0.5f; // Friction: 1.0 means no slip, 0.0 means full slip.

	for (auto& p : m_particles)
	{
		if (p.position[1] < floorY)
		{
			// 1. Move the particle back to the floor level.
			p.position[1] = floorY;

			// 2. Stop vertical movement (prevent bouncing)
			// Since velocity is calculated as (pos - prevPos), we set prevPosition to the current position to zero out the vertical velocity.
			p.prevPosition[1] = floorY;

			// 3. Horizontal PBD friction (momentum stealing). Find how much horizontal movement we had in this frame (deltaX).
			float deltaX = p.position[0] - p.prevPosition[0];

			// Move the previous position towards the current position by a fraction determined by the friction.
			// This reduces the horizontal velocity when calculating the new velocity in UpdateVelocities.
			p.prevPosition[0] = p.position[0] - (deltaX * (1.0f - friction));
		}
	}
}

/// <summary>
/// Solves all distance and area constraints in the simulation iteratively to maintain the physical properties of the system.
/// The solver runs for a specified number of iterations, where each iteration applies corrections to particle positions based on the constraints.
/// Distance constraints ensure that pairs of particles maintain a specified rest length, while area constraints ensure that groups of particles maintain a specified area.
/// The stiffness and pressure parameters control how strongly the constraints are enforced, allowing for soft or rigid behavior.
/// This function is designed to be called from a single thread and modifies particle positions directly to satisfy the constraints.
/// </summary>
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


/// <summary>
///	Updates particle velocities based on the changes in position after constraint solving.
/// </summary>
/// <param name="deltaTime"></param>
void PhysicsEngine::UpdateVelocities(float deltaTime)
{
	if (deltaTime <= 0.0f) return;

	for (auto& p : m_particles)
	{
		if (p.inverseMass <= 0.0f) continue; // Skip immovable particles

		// Update velocity based on the change in position after constraint solving.
		p.velocity[0] = (p.position[0] - p.prevPosition[0]) / deltaTime;
		p.velocity[1] = (p.position[1] - p.prevPosition[1]) / deltaTime;
	}

}

void PhysicsEngine::CreateJellyBox(float startX, float startY, float size, float particleMass, float stiffness)
{
	// 4 CORNERS OF THE BOX
	uint32_t p0 = AddParticle(startX, startY, particleMass);               // Bottom Left
	uint32_t p1 = AddParticle(startX + size, startY, particleMass);        // Bottom Right
	uint32_t p2 = AddParticle(startX + size, startY + size, particleMass); // Top Right
	uint32_t p3 = AddParticle(startX, startY + size, particleMass);        // Top Left

	// 4 SIDES OF THE BOX
	AddDistanceConstraint(p0, p1, size, stiffness); // Bottom
	AddDistanceConstraint(p1, p2, size, stiffness); // Right
	AddDistanceConstraint(p2, p3, size, stiffness); // Top
	AddDistanceConstraint(p3, p0, size, stiffness); // Left

	// Connect the diagonals to prevent the box from collapsing on itself (Diagonal length: size * sqrt(2))
	float diagonal = size * 1.41421356f;
	AddDistanceConstraint(p0, p2, diagonal, stiffness); // Bottom Left -> Top Right
	AddDistanceConstraint(p1, p3, diagonal, stiffness); // Bottom Right -> Top Left
}

/// <summary>
/// Creates a soft-body circular jelly using a "Bicycle Wheel" topology.
/// A central core particle holds the structure together, connected to perimeter particles (spokes).
/// The perimeter particles are connected to each other to form the outer skin (rim).
/// An area constraint is applied to the perimeter to preserve the internal volume (pressure) and prevent inversion.
/// </summary>
/// <param name="centerX">X coordinate of the center.</param>
/// <param name="centerY">Y coordinate of the center.</param>
/// <param name="radius">Radius of the jelly ball.</param>
/// <param name="numParticles">Number of perimeter particles (Max 16 due to Area Constraint limits).</param>
/// <param name="particleMass">Mass of each individual particle.</param>
/// <param name="stiffness">Stiffness of the distance constraints (springs).</param>
void PhysicsEngine::CreateJellyBall(float centerX, float centerY, float radius, uint32_t numParticles, float particleMass, float stiffness)
{
	// ==========================================
	// 1. GUARD CLAUSES
	// ==========================================
	// An area constraint requires a minimum of 3 particles to form a valid polygon.
	if (numParticles < 3) return;

	// Clamp the number of particles to the maximum allowed by our fixed-size, GPU-friendly arrays.
	if (numParticles > MAX_AREA_PARTICLES)
	{
		numParticles = MAX_AREA_PARTICLES;
	}

	// ==========================================
	// 2. CREATE THE CORE (The Hub)
	// ==========================================
	// This central particle acts as the anchor for the spokes, preventing the circle from collapsing on itself.
	float coreMass = particleMass * (numParticles * 0.5f);
	uint32_t centerIndex = AddParticle(centerX, centerY, coreMass);

	// Array to store the indices of the outer skin particles for the area constraint.
	std::vector<uint32_t> perimeterIndices;
	perimeterIndices.reserve(numParticles);

	// ==========================================
	// 3. CREATE PERIMETER PARTICLES (The Outer Skin)
	// ==========================================
	// Distribute them evenly around the center using polar to cartesian coordinates.
	const float PI = 3.14159265f;
	float angleStep = (2.0f * PI) / static_cast<float>(numParticles);

	for (uint32_t i = 0; i < numParticles; ++i)
	{
		float angle = i * angleStep;
		float px = centerX + std::cos(angle) * radius;
		float py = centerY + std::sin(angle) * radius;

		uint32_t pIdx = AddParticle(px, py, particleMass);
		perimeterIndices.push_back(pIdx);
	}

	// ==========================================
	// 4. CONNECT THE DISTANCE CONSTRAINTS (Springs)
	// ==========================================
	// Calculate the exact straight-line distance between two neighboring particles on the perimeter.
	// Formula: distance = 2 * r * sin(theta / 2)
	float rimLength = 2.0f * radius * std::sin(angleStep * 0.5f);

	for (uint32_t i = 0; i < numParticles; ++i)
	{
		uint32_t currentParticle = perimeterIndices[i];

		// Use modulo to smoothly wrap the last particle back to the first one.
		uint32_t nextParticle = perimeterIndices[(i + 1) % numParticles];

		// A. The Rim: Connect neighbor to neighbor (Outer skin)
		AddDistanceConstraint(currentParticle, nextParticle, rimLength, stiffness);

		// B. The Spokes: Connect the perimeter particle to the center core
		AddDistanceConstraint(centerIndex, currentParticle, radius, stiffness);
	}

	// ==========================================
	// 5. THE MAGIC SAUCE: VOLUME / AREA PRESERVATION
	// ==========================================
	// Calculate the exact mathematical area of this regular polygon.
	// Area of a regular polygon = n * 0.5 * r^2 * sin(2 * PI / n)
	float restArea = numParticles * 0.5f * radius * radius * std::sin(angleStep);

	// Apply the Area Constraint to the perimeter. 
	// A pressure of 1.0f means it will try to perfectly maintain this calculated rest area.
	AddAreaConstraint(perimeterIndices, restArea, 1.0f);
}

void PhysicsEngine::CreateRealisticJiggle(float startX, float startY, float radius, float particleMass, float stiffness)
{
	const int res = 32; // 32 x 32 grid = 1024 particles per drop, which is a good balance between detail and performance for a jiggle effect. Adjust as needed.
	const float spacing = (radius * 2.0f) / static_cast<float>(res - 1);

	auto buildDrop = [this, res, spacing, radius, particleMass, stiffness](float cx, float cy, float tiltDir) -> std::vector<std::vector<int>>
		{
			std::vector<std::vector<int>> grid(res, std::vector<int>(res, -1));

			// ==========================================
			// 1. Create Circular Particle Arrangement (The Drop)
			// ==========================================
			for (int y = 0; y < res; ++y)
			{
				for (int x = 0; x < res; ++x)
				{
					float px = (cx - radius) + (x * spacing);
					float py = (cy - radius) + (y * spacing);

					float dx = px - cx;
					float dy = py - cy;
					if ((dx * dx + dy * dy) <= (radius * radius * 1.2f))
					{
						float mass = particleMass;

						if (y >= res - 2 && x > 0 && x < res - 1)
						{
							mass = 0.0f;
							py += radius * 0.6f;
							px += tiltDir * radius * 0.2f;
						}

						grid[y][x] = AddParticle(px, py, mass);
					}
				}
			}

			// ==========================================
			// 2. Connect the Springs (Distance Constraints)
			// ==========================================
			for (int y = 0; y < res; ++y)
			{
				for (int x = 0; x < res; ++x)
				{
					int p1 = grid[y][x];
					if (p1 == -1) continue;

					auto addSpring = [this, p1, stiffness](int p2) {
						if (p2 == -1) return;
						float dx = m_particles[p1].position[0] - m_particles[p2].position[0];
						float dy = m_particles[p1].position[1] - m_particles[p2].position[1];
						float dist = std::sqrt(dx * dx + dy * dy);
						AddDistanceConstraint(p1, p2, dist, stiffness);
						};

					if (x < res - 1) addSpring(grid[y][x + 1]);
					if (y < res - 1) addSpring(grid[y + 1][x]);

					if (x < res - 1 && y < res - 1) addSpring(grid[y + 1][x + 1]);
					if (x > 0 && y < res - 1)       addSpring(grid[y + 1][x - 1]);
				}
			}
			return grid;
		};

	// ==========================================
	// 3. Create Two Drops
	// ==========================================
	auto leftGrid = buildDrop(startX - radius * 0.95f, startY, 1.0f);
	auto rightGrid = buildDrop(startX + radius * 0.95f, startY, -1.0f);

	// ==========================================
	// 4. Connect the Two Drops with a Soft Spring
	// ==========================================
	int leftInner = leftGrid[res / 2][res - 1] != -1 ? leftGrid[res / 2][res - 1] : leftGrid[res / 2][res - 2];
	int rightInner = rightGrid[res / 2][0] != -1 ? rightGrid[res / 2][0] : rightGrid[res / 2][1];

	if (leftInner != -1 && rightInner != -1)
	{
		float dx = m_particles[leftInner].position[0] - m_particles[rightInner].position[0];
		float dy = m_particles[leftInner].position[1] - m_particles[rightInner].position[1];
		float dist = std::sqrt(dx * dx + dy * dy);

		AddDistanceConstraint(leftInner, rightInner, dist, stiffness * 0.3f);
	}
}