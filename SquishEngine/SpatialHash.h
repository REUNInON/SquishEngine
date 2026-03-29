#pragma once
#include <vector>
#include <cstdint>
#include "PhysicsEngine.h"

class SpatialHash
{
public:
	SpatialHash(float spacing, uint32_t maxParticles);

	void BuildGrid(const std::vector<Particle2D>& particles);

private:
	float m_spacing; // Size of each grid cell
	float m_inverseSpacing; // 1 / spacing, Used for fast hashing (multiplication instead of division)

	uint32_t m_tableSize; // Size of the hash table (number of buckets)

	// ============================================================================
	// GPU Friendly Flat Arrays for Hash Table Storage
	// ============================================================================
	std::vector<uint32_t> m_cellStart;
	std::vector<uint32_t> m_particleCellIndices;
	std::vector<uint32_t> m_sortedParticleIDs;
};

