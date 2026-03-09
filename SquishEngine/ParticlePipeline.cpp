#include "ParticlePipeline.h"

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

    return E_NOTIMPL;
}

HRESULT ParticlePipeline::CreateIndexBuffer_(Renderer& renderer)
{
    return E_NOTIMPL;
}

HRESULT ParticlePipeline::CreateVertexBuffer_(Renderer& renderer)
{
    return E_NOTIMPL;
}
