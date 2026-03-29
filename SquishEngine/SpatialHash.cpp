#include "SpatialHash.h"
#include <cmath>
#include <algorithm>

/// <summary>
/// Constructor for the SpatialHash class.
/// </summary>
/// <param name="spacing">The spacing between grid cells.</param>
/// <param name="maxParticles">Maximum number of particles that can be stored in the spatial hash.</param>
SpatialHash::SpatialHash(float spacing, uint32_t maxParticles)
{
	m_spacing = spacing;
	m_inverseSpacing = 1.0f / spacing;

	m_tableSize = maxParticles * 2; // To minimize collisions, we can use a hash table size larger than the number of particles

	// INITIALIZE HASH TABLE ARRAYS
	m_cellStart.resize(m_tableSize + 1, 0); // +1 makes it easier to compute cell ranges
	m_particleCellIndices.resize(maxParticles, 0);
	m_sortedParticleIDs.resize(maxParticles, 0);
}

/// <summary>
/// Converts 2D coordinates (xi, yi) to a hash index for the spatial grid.
/// </summary>
/// <param name="xi">The x-coordinate in the grid.</param>
/// <param name="yi">The y-coordinate in the grid.</param>
/// <param name="tableSize">The size of the hash table.</param>
/// <returns>The hash index corresponding to the given coordinates.</returns>
inline uint32_t HashCoordinates(int xi, int yi, uint32_t tableSize)
{
	// XOR With Large Primes to Mix Bits
	uint32_t h = (xi * 92837111) ^ (yi * 689287499);

	// Modulo to Fit Into Table Size
	return h % tableSize; // TODO: Use bitwise AND if tableSize is a power of 2 for better performance
}

/// <summary>
/// Uses "Counting Sort" to organize particles into the spatial hash grid.
/// </summary>
/// <param name="particles">The list of particles to be hashed.</param>
void SpatialHash::BuildGrid(const std::vector<Particle2D>& particles)
{
    uint32_t numParticles = static_cast<uint32_t>(particles.size());
    if (numParticles == 0) return;

	// 1. CLEAR PREVIOUS GRID DATA
    std::fill(m_cellStart.begin(), m_cellStart.end(), 0);

	// 2. COUNT PARTICLES IN EACH CELL
    for (uint32_t i = 0; i < numParticles; ++i)
    {
		// 2.1 CONVERT PARTICLE POSITION TO GRID COORDINATES
        int xi = static_cast<int>(std::floor(particles[i].position[0] * m_inverseSpacing));
        int yi = static_cast<int>(std::floor(particles[i].position[1] * m_inverseSpacing));

        // 2.2 FIND THE HASH ID OF THE CELL
        uint32_t hash = HashCoordinates(xi, yi, m_tableSize);

		// 2.3 STORE THE CELL INDEX FOR THIS PARTICLE
        m_particleCellIndices[i] = hash;

        // 2.4 INCREMENT THE POPULATION OF THAT CELL
        m_cellStart[hash]++;
    }

    // 3. PREFIX SUM (DETERMINE STARTING INDICES)
    uint32_t start = 0;
    for (uint32_t i = 0; i < m_tableSize; ++i)
    {
        start += m_cellStart[i];
        m_cellStart[i] = start;
    }

	// 4. SORT PARTICLE IDS INTO THE HASH TABLE
    for (uint32_t i = 0; i < numParticles; ++i)
    {
        uint32_t hash = m_particleCellIndices[i];

        // Move the starting index of the target cell back by one
        m_cellStart[hash]--;

        // Place the particle ID in its correct position
        m_sortedParticleIDs[m_cellStart[hash]] = i;
    }
}