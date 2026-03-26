#pragma once

#include "Renderer.h"
#include "PhysicsEngine.h"

class ParticlePipeline
{
public:

	HRESULT Initialize(Renderer& renderer);

	HRESULT Render(Renderer& renderer, const std::vector<Particle2D>& particles, const std::vector<DistanceConstraint>& constraints);

	void Shutdown();

	// ============================
	// PIPELINE LIMITS
	// ============================
	static constexpr UINT MAX_PARTICLES = 10000;
	static constexpr UINT MAX_LINES = 20000;
	static constexpr UINT MAX_INDICES = MAX_LINES * 2;


private:

	// ============================
	// GPU Resources
	// ============================

	ComPtr<ID3D12RootSignature> m_rootSignature;
	ComPtr<ID3D12PipelineState> m_pso;

	ComPtr<ID3D12Resource> m_vb;
	D3D12_VERTEX_BUFFER_VIEW m_vbView;
	ComPtr<ID3D12Resource> m_ib;
	D3D12_INDEX_BUFFER_VIEW m_ibView;


	// =============================
	// INITIALIZATION HELPERS
	// =============================

	HRESULT CreateRootSignature_(Renderer& renderer);
	HRESULT CreatePSO_(Renderer& renderer);

	HRESULT CreateIndexBuffer_(Renderer& renderer);
	HRESULT CreateVertexBuffer_(Renderer& renderer);

	// =============================
	// SIMULATION MESH DATA STRUCTURES
	// =============================
	struct ParticleVertex { float x, y; };

};

