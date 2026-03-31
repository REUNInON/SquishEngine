#include "PhysicsEngine.h"
#include "SpatialHash.h"

// ===========================
// Constructor
// ===========================
PhysicsEngine::PhysicsEngine()
{	
	uint32_t maxParticles = 10000; // Maximum particle count in simulation.

	m_spatialHash = std::make_unique<SpatialHash>(PhysicsEngine::GLOBAL_GRID_SPACING, maxParticles);
}

PhysicsEngine::~PhysicsEngine() = default;

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

	if (m_spatialHash && !m_particles.empty())
	{
		m_spatialHash->BuildGrid(m_particles);
	}

	SolveConstraints();
	UpdateVelocities(deltaTime);
}

// ===========================
// Particle Management
// ===========================

/// <summary>
/// Creates a new particle with the specified position and mass, initializes its velocity and force to zero, and returns its index in the particle list.
/// The inverse mass is calculated for efficient physics calculations, handling the case of zero or negative mass appropriately.
/// </summary>
/// <param name="x">The x-coordinate of the particle's position.</param>
/// <param name="y">The y-coordinate of the particle's position.</param>
/// <param name="mass">The mass of the particle.</param>
/// <returns>The index of the newly added particle in the particle list.</returns>
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

	p.bodyId = m_currentBodyId;
	
	// 5. Add to particle list and return index
	m_particles.push_back(p);

	return static_cast<uint32_t>(m_particles.size() - 1);
}

/// <summary>
/// Adds a distance constraint between two particles identified by their indices, specifying the desired rest length and stiffness of the constraint.
/// The constraint is stored in a list for later processing during the simulation step to maintain the specified distance between the particles.
/// Stiffness controls how strongly the constraint is enforced, allowing for soft or rigid connections.
/// </summary>
/// <param name="p1">The index of the first particle in the constraint.</param>
/// <param name="p2">The index of the second particle in the constraint.</param>
/// <param name="restLength">The desired rest length between the two particles.</param>
/// <param name="stiffness">The stiffness of the constraint, controlling how strongly it is enforced.</param>
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
/// Calculated using the Shoelace formula, and the pressure parameter controls how much the particles should resist changes to this area (0.8: deflated, 1.0: normal, 1.2: inflated).
/// The constraint is stored in a vector for later processing during the simulation step to maintain the specified area among the particles.
/// </summary>
/// <param name="particleIndices">A vector of indices representing the particles involved in the area constraint.</param>
/// <param name="restArea">The desired rest area for the group of particles.</param>
/// <param name="pressure">The pressure parameter controlling how strongly the particles resist changes to the area.</param>
void PhysicsEngine::AddAreaConstraint(const std::vector<uint32_t>& particleIndices, float restArea, float pressure)
{
	if (particleIndices.size() < 3) return; // Area constraint requires at least 3 particles.

	AreaConstraint constraint;
	constraint.startIndex = static_cast<uint32_t>(m_areaIndices.size());
	constraint.particleCount = static_cast<uint32_t>(particleIndices.size());
	constraint.restArea = restArea;
	constraint.pressure = pressure;

	// Append particle indices to the global area indices list.
	m_areaIndices.insert(m_areaIndices.end(), particleIndices.begin(), particleIndices.end());

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
/// <param name="deltaTime">The time step (in seconds)</param>
void PhysicsEngine::Integrate(float deltaTime)
{
	// World Forces
	const float gravity[2] = { 0.0f, -9.81f }; // Earth gravity -9.81f, applied to all particles
	const float dragFactor = std::pow(0.5f, deltaTime); // Global damping factor to simulate energy loss, applied to velocities

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
		p.velocity[0] *= dragFactor; // v = v * damping
		p.velocity[1] *= dragFactor; // v = v * damping

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

/// <summary>
/// Solves all distance and area constraints in the simulation iteratively to maintain the physical properties of the system.
/// The solver runs for a specified number of iterations, where each iteration applies corrections to particle positions based on the constraints.
/// Distance constraints ensure that pairs of particles maintain a specified rest length, while area constraints ensure that groups of particles maintain a specified area.
/// The stiffness and pressure parameters control how strongly the constraints are enforced, allowing for soft or rigid behavior.
/// </summary>
void PhysicsEngine::SolveConstraints()
{
	const float floorY = -0.99f;
	const float friction = 0.5f; // Friction: 1.0 means no slip, 0.0 means full slip.

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

			// 1.1. Calculate the current distance between the two particles. (Pythagorean Theorem)
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
		std::vector<float> gradientsX;
		std::vector<float> gradientsY;

		for (const auto& constraint : m_areaConstraints)
		{
			// 2.0. Memory Allocation & Setup
			uint32_t count = constraint.particleCount;
			if (count < 3) continue;

			gradientsX.resize(count);
			gradientsY.resize(count);

			float currentArea = 0.0f;

			// 2.1. Area Calculation Using Shoelace Formula
			for (uint32_t j = 0; j < count; ++j)
			{
				uint32_t idx1 = m_areaIndices[constraint.startIndex + j];
				uint32_t idx2 = m_areaIndices[constraint.startIndex + ((j + 1) % count)]; // Wrap around to the first particle

				const Particle2D& p1 = m_particles[idx1];
				const Particle2D& p2 = m_particles[idx2];

				// Cross Multiplication for Shoelace formula: (X1 * Y2 - X2 * Y1)
				currentArea += (p1.position[0] * p2.position[1]) - (p2.position[0] * p1.position[1]);
			}
			currentArea *= 0.5f;

			// 2.2. Calculate Area Deviation and Target Area
			float targetArea = constraint.restArea * constraint.pressure; // (Pressure 1 = Normal Volume, 1.2 = Inflated, 0.8 Deflated)
			float error = currentArea - targetArea;

			// 2.3. Calculate Gradients (Pushing Direction)
			float denominator = 0.0f; // Avoid division by zero when all particles are immovable.

			for (uint32_t j = 0; j < count; ++j)
			{
				// Particle Indices
				uint32_t prevIdx = m_areaIndices[constraint.startIndex + ((j + count - 1) % count)];
				uint32_t currentIdx = m_areaIndices[constraint.startIndex + j];
				uint32_t nextIdx = m_areaIndices[constraint.startIndex + ((j + 1) % count)];

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

			// 2.4. Apply Position Based Dynamics (PBD) Correction
			if (denominator <= 0.0f) continue; // Skip if all particles are immovable to avoid division by zero.

			// PBD Multiplier (Lambda)
			float lambda = -error / denominator;

			for (uint32_t j = 0; j < count; ++j)
			{
				uint32_t idx = m_areaIndices[constraint.startIndex + j];
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

		// ======================================
		// 3. SOLVE PARTICLE COLLISIONS
		// ======================================
		if (m_spatialHash)
		{
			float minDistance = PhysicsEngine::GLOBAL_PARTICLE_RADIUS * 2.0f; // Minimum distance to avoid overlap (2 * radius)
			float minDistanceSq = minDistance * minDistance; // Squared Minimum Distance

			for (uint32_t i = 0; i < m_particles.size(); ++i)
			{
				Particle2D& p1 = m_particles[i];

				if (p1.inverseMass <= 0.0f) continue; // Skip immovable particles

				int xi = static_cast<int>(std::floor(p1.position[0] / PhysicsEngine::GLOBAL_GRID_SPACING));
				int yi = static_cast<int>(std::floor(p1.position[1] / PhysicsEngine::GLOBAL_GRID_SPACING));

				// 3 x 3 Neighborhood Search
				for (int dy = -1; dy <= 1; ++dy)
				{
					for (int dx = -1; dx <= 1; ++dx)
					{
						// Find neighboring cell's hash
						uint32_t hash = HashCoordinates(xi + dx, yi + dy, m_spatialHash->m_tableSize);

						// Get start and end indices of particles in this cell from the spatial hash
						uint32_t startIdx = m_spatialHash->m_cellStart[hash];
						uint32_t nextHash = hash + 1;
						uint32_t endIdx = (nextHash < m_spatialHash->m_cellStart.size()) 
										? m_spatialHash->m_cellStart[nextHash] 
										: static_cast<uint32_t>(m_particles.size());

						// Old version: Bad because it can cause out-of-bounds access when hash is the last index of m_cellStart
						// uint32_t endIdx = m_spatialHash->m_cellStart[hash + 1];

						// 3.3 Narrow Phase: Only check particles in the same or neighboring cells for collisions.
						for (uint32_t k = startIdx; k < endIdx; ++k)
						{
							uint32_t neighborIdx = m_spatialHash->m_sortedParticleIDs[k];

							// If neighbor index is less than or equal to current index, skip to avoid double checking pairs (A-B and B-A)
							if (neighborIdx <= i) continue;

							Particle2D& p2 = m_particles[neighborIdx];

							// Skip if same body.
							if (p1.bodyId == p2.bodyId) continue;

							// Calculate distance between p1 and p2
							float diffX = p1.position[0] - p2.position[0];
							float diffY = p1.position[1] - p2.position[1];

							// Pythagorean Theorem (c^2 = a^2 + b^2) - Squared distance
							float distSq = diffX * diffX + diffY * diffY;

							// If the squared distance is less than the squared minimum distance, we have a collision and need to resolve it.
							// (0.00001f check prevents division by zero if two particles are exactly on top of each other)
							if (distSq < minDistanceSq && distSq > 0.00001f)
							{
								// Square root (sqrt) operation only if there is a collision
								float dist = std::sqrt(distSq);

								// Calculate penetration (overlap)
								float penetration = minDistance - dist;

								// Find the push direction (normalized vector from p2 to p1)
								float normalX = diffX / dist;
								float normalY = diffY / dist;

								// Calculate how much to push each particle based on their inverse mass
								float totalInvMass = p1.inverseMass + p2.inverseMass;
								float correctionAmount = penetration / totalInvMass;

								// Push particles apart.
								// PBD RULE: P1 and P2 should be pushed to opposite directions.
								p1.position[0] += normalX * correctionAmount * p1.inverseMass;
								p1.position[1] += normalY * correctionAmount * p1.inverseMass;

								p2.position[0] -= normalX * correctionAmount * p2.inverseMass;
								p2.position[1] -= normalY * correctionAmount * p2.inverseMass;
							}
						}
					}
				}
			}
		
		}

		// =============================
		// 4. SOLVE GROUND COLLISION
		// =============================
		for (auto& p : m_particles)
		{
			if (p.inverseMass <= 0.0f) continue; // Skip rigid particles

			// If the particle is below the floor level, move it back up to the floor level.
			if (p.position[1] < floorY)
			{
				p.position[1] = floorY;

				// ADD FRICTION: DELETE IF BAD!!
				float deltaX = p.position[0] - p.prevPosition[0];
				p.prevPosition[0] = p.position[0] - (deltaX * (1.0f - friction));
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

// ----------------------------------------
// ========================================
// PROCEDURAL MESHES FOR TESTING THE ENGINE
// ========================================
// ----------------------------------------

void PhysicsEngine::CreateJellyBox(float centerX, float centerY, float size, float particleMass, float stiffness)
{
	m_currentBodyId++;

	int res = 16;
	float spacing = size / static_cast<float>(res - 1);

	float startX = centerX - size * 0.5f;
	float startY = centerY - size * 0.5f;

	std::vector<std::vector<int>> grid(res, std::vector<int>(res, -1));

	// ==========================================
	// 1. MESH
	// ==========================================
	for (int y = 0; y < res; ++y)
	{
		for (int x = 0; x < res; ++x)
		{
			float px = startX + x * spacing;
			float py = startY + y * spacing;
			grid[y][x] = AddParticle(px, py, particleMass);
		}
	}

	// ==========================================
	// 2. CONNECT THE SPRINGS
	// ==========================================
	auto connectSpring = [&](int p1, int p2, float customStiffness) {
		float dx = m_particles[p1].position[0] - m_particles[p2].position[0];
		float dy = m_particles[p1].position[1] - m_particles[p2].position[1];
		float dist = std::sqrt(dx * dx + dy * dy);
		AddDistanceConstraint(p1, p2, dist, customStiffness);
		};

	for (int y = 0; y < res; ++y)
	{
		for (int x = 0; x < res; ++x)
		{
			int p = grid[y][x];

			bool isTop = (y == res - 1);
			bool isBottom = (y == 0);
			bool isLeft = (x == 0);
			bool isRight = (x == res - 1);

			if (x < res - 1) {
				float s = (isTop || isBottom) ? 1.0f : stiffness;
				connectSpring(p, grid[y][x + 1], s);
			}
			if (y < res - 1) {
				float s = (isLeft || isRight) ? 1.0f : stiffness;
				connectSpring(p, grid[y + 1][x], s);
			}

			if (x < res - 1 && y < res - 1) connectSpring(p, grid[y + 1][x + 1], stiffness * 0.8f);
			if (x > 0 && y < res - 1)       connectSpring(p, grid[y + 1][x - 1], stiffness * 0.8f);
		}
	}

	// ==========================================
	// 3. PRESSURE
	// ==========================================
	for (int y = 0; y < res - 1; ++y)
	{
		for (int x = 0; x < res - 1; ++x)
		{
			int pBL = grid[y][x];
			int pBR = grid[y][x + 1];
			int pTR = grid[y + 1][x + 1];
			int pTL = grid[y + 1][x];

			auto addTriArea = [&](int i1, int i2, int i3) {
				std::vector<uint32_t> tri = { static_cast<uint32_t>(i1), static_cast<uint32_t>(i2), static_cast<uint32_t>(i3) };
				float area = 0.0f;
				for (int j = 0; j < 3; ++j) {
					const Particle2D& pA = m_particles[tri[j]];
					const Particle2D& pB = m_particles[tri[(j + 1) % 3]];
					area += (pA.position[0] * pB.position[1]) - (pB.position[0] * pA.position[1]);
				}

				AddAreaConstraint(tri, std::abs(area * 0.5f), 1.0f);
				};

			addTriArea(pBL, pBR, pTL);
			addTriArea(pBR, pTR, pTL);
		}
	}
}

void PhysicsEngine::CreateSoftBall(float centerX, float centerY, float radius, float particleMass, float stiffness)
{
	m_currentBodyId++;

	const float PI = 3.14159265f;

	// ==========================================
	// 1. OUTER AREA
	// ==========================================
	const int outerCount = 128;
	std::vector<uint32_t> outerIds;
	outerIds.reserve(outerCount);

	for (int i = 0; i < outerCount; ++i)
	{
		float angle = (2.0f * PI * i) / outerCount;
		float px = centerX + std::cos(angle) * radius;
		float py = centerY + std::sin(angle) * radius;
		outerIds.push_back(AddParticle(px, py, particleMass));
	}

	// ==========================================
	// 2. INNER AREA
	// ==========================================
	const int innerCount = 32;
	std::vector<uint32_t> innerIds;
	innerIds.reserve(innerCount);
	float innerRadius = radius * 0.70f; // Kabuğun %70'i kadar içeride

	for (int i = 0; i < innerCount; ++i)
	{
		float angle = (2.0f * PI * i) / innerCount;
		float px = centerX + std::cos(angle) * innerRadius;
		float py = centerY + std::sin(angle) * innerRadius;
		innerIds.push_back(AddParticle(px, py, particleMass));
	}

	// ==========================================
	// 3. CORE AREA
	// ==========================================
	const int coreCount = 8;
	std::vector<uint32_t> coreIds;
	coreIds.reserve(coreCount);
	float coreRadius = radius * 0.20f;

	for (int i = 0; i < coreCount; ++i)
	{
		float angle = (2.0f * PI * i) / coreCount;
		float px = centerX + std::cos(angle) * coreRadius;
		float py = centerY + std::sin(angle) * coreRadius;
		coreIds.push_back(AddParticle(px, py, particleMass));
	}

	// ==========================================
	// 4. CONNECT THE SPRINGS
	// ==========================================
	auto connectSpring = [&](uint32_t p1, uint32_t p2, float customStiffness) {
		float dx = m_particles[p1].position[0] - m_particles[p2].position[0];
		float dy = m_particles[p1].position[1] - m_particles[p2].position[1];
		float dist = std::sqrt(dx * dx + dy * dy);
		AddDistanceConstraint(p1, p2, dist, customStiffness);
		};

	// A. CONNECT OUTER AREA
	for (int i = 0; i < outerCount; ++i)
	{

		connectSpring(outerIds[i], outerIds[(i + 1) % outerCount], 1.0f);
		connectSpring(outerIds[i], outerIds[(i + 2) % outerCount], stiffness);
		int innerTarget = i / (outerCount / innerCount);
		connectSpring(outerIds[i], innerIds[innerTarget], stiffness * 0.5f);
	}

	// B. CONNECT INNER AREA
	for (int i = 0; i < innerCount; ++i)
	{
		connectSpring(innerIds[i], innerIds[(i + 1) % innerCount], stiffness * 0.5f);
		connectSpring(innerIds[i], innerIds[(i + 2) % innerCount], stiffness * 0.5f);

		int coreTarget = i / (innerCount / coreCount);
		connectSpring(innerIds[i], coreIds[coreTarget], stiffness * 0.2f);
	}

	// C. CONNECT CORE AREA
	for (int i = 0; i < coreCount; ++i)
	{
		connectSpring(coreIds[i], coreIds[(i + 1) % coreCount], stiffness);
		connectSpring(coreIds[i], coreIds[(i + 2) % coreCount], stiffness);
		connectSpring(coreIds[i], coreIds[(i + 3) % coreCount], stiffness);
	}

	// ==========================================
	// 5. AREA CONSTRAINT
	// ==========================================
	float restArea = PI * radius * radius;
	AddAreaConstraint(outerIds, restArea, 1.0f);
}

void PhysicsEngine::CreateHangingJelly(float centerX, float centerY, float radius, float particleMass, float stiffness)
{
	m_currentBodyId++;

	const int outerCount = 40;
	std::vector<uint32_t> boundIds;
	boundIds.reserve(outerCount);

	const float PI = 3.14159265f;

	// ==========================================
	// 1. THE TOP PART AND PINS
	// ==========================================
	for (int i = 0; i < outerCount; ++i)
	{
		float angle = (2.0f * PI * i) / outerCount;
		float px = centerX + std::cos(angle) * radius;
		float py = centerY + std::sin(angle) * radius;

		float mass = (py > centerY + (radius * 0.85f)) ? 0.0f : particleMass;
		boundIds.push_back(AddParticle(px, py, mass));
	}

	// ==========================================
	// 2. CONNECT THE SPRINGS
	// ==========================================
	auto connectSpring = [&](uint32_t p1, uint32_t p2, float customStiffness) {
		float dx = m_particles[p1].position[0] - m_particles[p2].position[0];
		float dy = m_particles[p1].position[1] - m_particles[p2].position[1];
		float dist = std::sqrt(dx * dx + dy * dy);
		AddDistanceConstraint(p1, p2, dist, customStiffness);
		};

	for (int i = 0; i < outerCount; ++i)
	{

		connectSpring(boundIds[i], boundIds[(i + 1) % outerCount], stiffness);

		connectSpring(boundIds[i], boundIds[(i + 2) % outerCount], stiffness);

		connectSpring(boundIds[i], boundIds[(i + 3) % outerCount], stiffness * 0.5f);
	}

	// ==========================================
	// 3. AREA CONSTRAINT
	// ==========================================
	float restArea = PI * radius * radius;
	AddAreaConstraint(boundIds, restArea, 1.0f);
}