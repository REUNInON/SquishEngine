#include "ParticlePipeline.h"

#include "ParticleVS.h"
#include "ParticlePS.h"

// Helper macro for checking HRESULTs one by one.
// Usage: DX(your_dx_call());
#ifndef DX
#define DX(call) do { HRESULT _hr = (call); if (FAILED(_hr)) return _hr; } while(0)
#endif

/// <summary>
/// Creates an empty root signature that allows only input assembler input layout, since we will not use any textures or constant buffers in our particle shader.
/// </summary>
/// <param name="renderer">Renderer.</param>
/// <returns></returns>
HRESULT ParticlePipeline::CreateRootSignature_(Renderer& renderer)
{
	D3D12_ROOT_SIGNATURE_DESC desc = {0, nullptr, 0, nullptr}; // Empty root signature (We will not use textures or constant buffers.)

	// Flags: IA Only; We use vertex buffers but no textures or constant buffers.
    desc.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;

	// Serialize the Root Signature
	ComPtr<ID3DBlob> serializedSignature;
	ComPtr<ID3DBlob> errorBlob;

	// Convert the description to a root signature byte array
	DX(D3D12SerializeRootSignature(
		&desc, 
		D3D_ROOT_SIGNATURE_VERSION_1, 
		serializedSignature.GetAddressOf(), 
		errorBlob.GetAddressOf()));

	// Create the Root Signature on the GPU
	DX(renderer.GetDevice()->CreateRootSignature(
		0, // Single GPU
		serializedSignature->GetBufferPointer(),
		serializedSignature->GetBufferSize(),
		IID_PPV_ARGS(m_rootSignature.ReleaseAndGetAddressOf())));


    return S_OK;
}

HRESULT ParticlePipeline::CreatePSO_(Renderer& renderer)
{
	// INPUT LAYOUT: X, Y Position
	D3D12_INPUT_ELEMENT_DESC inputElementDescs[] =
	{
		{  "POSITION", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 }
	};

	// RASTERIZER (No Culling, Solid Fill)
	D3D12_RASTERIZER_DESC rasterizerDesc = {};
	rasterizerDesc.FillMode = D3D12_FILL_MODE_SOLID;
	rasterizerDesc.CullMode = D3D12_CULL_MODE_NONE; // No Depth Testing for 2D Particles
	rasterizerDesc.FrontCounterClockwise = FALSE;
	rasterizerDesc.DepthBias = D3D12_DEFAULT_DEPTH_BIAS;
	rasterizerDesc.DepthBiasClamp = D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
	rasterizerDesc.SlopeScaledDepthBias = D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
	rasterizerDesc.DepthClipEnable = TRUE;
	rasterizerDesc.MultisampleEnable = FALSE;
	rasterizerDesc.AntialiasedLineEnable = FALSE;
	rasterizerDesc.ForcedSampleCount = 0;
	rasterizerDesc.ConservativeRaster = D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;

	// BLEND STATE (Additive Blending for Particles)
	D3D12_BLEND_DESC blendDesc = {};
	blendDesc.AlphaToCoverageEnable = FALSE;
	blendDesc.IndependentBlendEnable = FALSE;
	blendDesc.RenderTarget[0].BlendEnable = FALSE;
	blendDesc.RenderTarget[0].LogicOpEnable = FALSE;
	blendDesc.RenderTarget[0].SrcBlend = D3D12_BLEND_ONE;
	blendDesc.RenderTarget[0].DestBlend = D3D12_BLEND_ZERO;
	blendDesc.RenderTarget[0].BlendOp = D3D12_BLEND_OP_ADD;
	blendDesc.RenderTarget[0].SrcBlendAlpha = D3D12_BLEND_ONE;
	blendDesc.RenderTarget[0].DestBlendAlpha = D3D12_BLEND_ZERO;
	blendDesc.RenderTarget[0].BlendOpAlpha = D3D12_BLEND_OP_ADD;
	blendDesc.RenderTarget[0].LogicOp = D3D12_LOGIC_OP_NOOP;
	blendDesc.RenderTarget[0].RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;

	// DEPTH-STENCIL STATE (No Depth Testing for 2D Particles)
	D3D12_DEPTH_STENCIL_DESC depthStencilDesc = {};
	depthStencilDesc.DepthEnable = FALSE;
	depthStencilDesc.StencilEnable = FALSE;

	// PSO DESCRIPTION
	D3D12_GRAPHICS_PIPELINE_STATE_DESC psoDesc = {};
	psoDesc.InputLayout = { inputElementDescs, _countof(inputElementDescs) };
	psoDesc.pRootSignature = m_rootSignature.Get();

	psoDesc.VS = { g_ParticleVS, sizeof(g_ParticleVS) };
	psoDesc.PS = { g_ParticlePS, sizeof(g_ParticlePS) };

	psoDesc.RasterizerState = rasterizerDesc;
	psoDesc.BlendState = blendDesc;
	psoDesc.DepthStencilState = depthStencilDesc;

	psoDesc.SampleMask = UINT_MAX;

	// WIREFRAME RENDERING (CAN CHANGE TO SOLID LATER)
	psoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE;

	// RENDER TARGET FORMATS
	psoDesc.NumRenderTargets = 1;
	psoDesc.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
	psoDesc.DSVFormat = DXGI_FORMAT_UNKNOWN;
	psoDesc.SampleDesc.Count = 1;

	// CREATE THE PIPELINE STATE OBJECT (PSO)
	DX(renderer.GetDevice()->CreateGraphicsPipelineState(&psoDesc, IID_PPV_ARGS(m_pso.ReleaseAndGetAddressOf())));

    return S_OK;
}

HRESULT ParticlePipeline::CreateIndexBuffer_(Renderer& renderer)
{
    return E_NOTIMPL;
}

HRESULT ParticlePipeline::CreateVertexBuffer_(Renderer& renderer)
{
    return E_NOTIMPL;
}
