#pragma once
#include <vector>
#include <cstdint>
#include "PhysicsEngine.h"

/// <summary>
/// Converts 2D coordinates (xi, yi) to a hash index for the spatial grid.
/// </summary>
/// <param name="xi">The x-coordinate in the grid.</param>
/// <param name="yi">The y-coordinate in the grid.</param>
/// <param name="tableSize">The size of the hash table.</param>
/// <returns>The hash index corresponding to the given coordinates.</returns>
inline uint32_t HashCoordinates(int xi, int yi, uint32_t tableSize)
{

	// Cast to unsigned to ensure well-defined behavior for negative coordinates
	uint32_t uxi = static_cast<uint32_t>(xi);
	uint32_t uyi = static_cast<uint32_t>(yi);

	// XOR With Large Primes to Mix Bits
	uint32_t h = (uxi * 92837111) ^ (uyi * 689287499);

	// Modulo to Fit Into Table Size
	return h % tableSize; // TODO: Use bitwise AND if tableSize is a power of 2 for better performance
}

class SpatialHash
{
public:
	SpatialHash(float spacing, uint32_t maxParticles);

	void BuildGrid(const std::vector<Particle2D>& particles);


	uint32_t m_tableSize; // Size of the hash table (number of buckets)

	// ============================================================================
	// GPU Friendly Flat Arrays for Hash Table Storage
	// ============================================================================
	std::vector<uint32_t> m_cellStart;
	std::vector<uint32_t> m_particleCellIndices;
	std::vector<uint32_t> m_sortedParticleIDs;

private:
	float m_spacing; // Size of each grid cell
	float m_inverseSpacing; // 1 / spacing, Used for fast hashing (multiplication instead of division)
};

