#include "Renderer.h"

// Helper macro for checking HRESULTs one by one.
// Usage: DX(your_dx_call());
#ifndef DX
#define DX(call) do { HRESULT _hr = (call); if (FAILED(_hr)) return _hr; } while(0)
#endif

HRESULT Renderer::Initialize(HWND hWnd, UINT width, UINT height)
{
	EnableDebugLayer_();

	m_hWnd = hWnd;
	m_width = width;
	m_height = height;

	if (FAILED(CreateDevice_(/*DXGI FLAGS*/))) return E_FAIL; // Create DXGI Factory and D3D12 Device 
	if (FAILED(CreateCommandQueue_())) return E_FAIL; // Create Command Queue and Command Allocator
	if (FAILED(CreateCommandList_())) return E_FAIL; // Create Command List
	if (FAILED(CreateFence_())) return E_FAIL; // Create Fence for GPU synchronization
	if (FAILED(CreateSwapChain_())) return E_FAIL; // Create Swap Chain and Render Target Views (RTVs)
	if (FAILED(CreateDepthBuffer_())) return E_FAIL; // Create Depth Stencil View (DSV) and Depth Texture

	InitTimestampQuery(); return S_OK;
}

HRESULT Renderer::CreateDevice_()
{
	// Create DXGI Factory
	DX(CreateDXGIFactory2
	(
		0 /*DXGI FLAGS: 0 = No Debug, 1 = Debug*/,
		IID_PPV_ARGS(m_factory.ReleaseAndGetAddressOf())
	));

	// Create D3D12 Device
	DX(D3D12CreateDevice
	(
		nullptr, // Default Adapter
		D3D_FEATURE_LEVEL_11_0, // Minimum Feature Level, Increase later.
		IID_PPV_ARGS(m_device.ReleaseAndGetAddressOf())
	));

	return S_OK;
}

HRESULT Renderer::CreateCommandQueue_()
{
	// Command Queue Description
	D3D12_COMMAND_QUEUE_DESC qDesc{};
	qDesc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT; // Alternative: COMPUTE, COPY
	qDesc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE; // Alternative: DISABLE_GPU_TIMEOUT

	// Create Command Queue
	DX(m_device->CreateCommandQueue(&qDesc, IID_PPV_ARGS(m_cmdQueue.ReleaseAndGetAddressOf())));

	// Create Command Allocators for each back buffer
	for (UINT i = 0; i < BACKBUFFER_COUNT; ++i)
	{
		DX(m_device->CreateCommandAllocator
		(
			D3D12_COMMAND_LIST_TYPE_DIRECT, // Same type as command queue
			IID_PPV_ARGS(m_cmdAllocator[i].ReleaseAndGetAddressOf())
		));

		m_fenceValues[i] = 0; // Per-frame fence counter initialization
	}

	return S_OK;
}

HRESULT Renderer::CreateCommandList_()
{
	// Create Command List
	DX(m_device->CreateCommandList
	(
		0, // Single GPU
		D3D12_COMMAND_LIST_TYPE_DIRECT, // Same type as command queue and command allocator
		m_cmdAllocator[0].Get(), // Initial Command Allocator (will change per frame)
		nullptr, // No initial pipeline state
		IID_PPV_ARGS(m_cmdList.ReleaseAndGetAddressOf())
	));

	// Command lists are created in the recording state. We need to close it before using.
	DX(m_cmdList->Close());

	m_frameIndex = 0; // Start from first back buffer

	return S_OK;
}

HRESULT Renderer::CreateFence_()
{
	// Create Fence Object
	DX(m_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(m_fence.ReleaseAndGetAddressOf()))); // Initial value 0, no special flags

	// Create Fence Event to signal CPU when GPU is done
	m_fenceEvent = CreateEvent(nullptr, FALSE, FALSE, nullptr); // Create an event handle for fence. This is used to wait on CPU.
	if (!m_fenceEvent) return HRESULT_FROM_WIN32(GetLastError());

	// Initialize fence values
	for (UINT i = 0; i < BACKBUFFER_COUNT; ++i) m_fenceValues[i] = 0;
	m_globalFenceCount = 0;
	return S_OK;
}

/// <summary>
/// Issues a resource state transition barrier on a Direct3D 12 command list.
/// </summary>
/// <param name="cmdList">The command list on which to record the barrier.</param>
/// <param name="pResource">The resource to transition.</param>
/// <param name="stateBefore">The state of the resource before the barrier.</param>
/// <param name="stateAfter">The state of the resource after the barrier.</param>
void Renderer::IssueBarrier(ID3D12GraphicsCommandList* cmdList, ID3D12Resource* pResource, D3D12_RESOURCE_STATES stateBefore, D3D12_RESOURCE_STATES stateAfter)
{
	// TODO: New alternative barrier API in D3D12? Check later. Ask RObert.

	D3D12_RESOURCE_BARRIER barrier = {};
	barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
	barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
	barrier.Transition.pResource = pResource;
	barrier.Transition.StateBefore = stateBefore;
	barrier.Transition.StateAfter = stateAfter;
	barrier.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES; // TODO: Apply to all subresources as default?

	cmdList->ResourceBarrier(1, &barrier);
}


// =============================================================
// Descriptors & Descriptor Heaps
// =============================================================
HRESULT Renderer::CreateDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE type, UINT numDescriptors, ComPtr<ID3D12DescriptorHeap>& heap)
{
	heap.Reset(); // Reset existing heap

	// Descriptor Heap Description
	D3D12_DESCRIPTOR_HEAP_DESC heapDesc{};
	heapDesc.Type = type; // RTV, DSV, CBV_SRV_UAV, SAMPLER
	heapDesc.NumDescriptors = numDescriptors; // Allocate this many descriptors in the memory
	heapDesc.NodeMask = 0; // Single GPU

	// Set Shader Visible flag for SRV/UAV/CBV heaps used in shaders
	if (type == D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV || type == D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER)
	{
		heapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	}
	else
	{
		heapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE;
	}

	// Create Descriptor Heap
	DX(m_device->CreateDescriptorHeap(&heapDesc, IID_PPV_ARGS(heap.ReleaseAndGetAddressOf())));

	return S_OK;
}

HRESULT Renderer::CreateSwapChain_()
{
	// Swap Chain Description
	DXGI_SWAP_CHAIN_DESC1 scDesc{};
	scDesc.BufferCount = BACKBUFFER_COUNT; // Double Buffering
	scDesc.Width = m_width;
	scDesc.Height = m_height;
	scDesc.SampleDesc.Count = 1; // 1 = No MSAA, 2 = 2x MSAA, 4 = 4x MSAA, 8 = 8x MSAA
	scDesc.SampleDesc.Quality = 0; // Standard quality level
	scDesc.Format = m_backBufferFormat; // 8-bit RGBA, not sRGB for clean measurements
	scDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT; // Render Target usage
	scDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD; // Modern flip model
	scDesc.Scaling = DXGI_SCALING_NONE; // No scaling
	scDesc.AlphaMode = DXGI_ALPHA_MODE_IGNORE; // Ignore alpha channel. Alternatives: PREMULTIPLIED, STRAIGHT, NONE
	scDesc.Flags = 0; // No special flags. Can add DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING for variable refresh rate displays.

	// Create Swap Chain
	ComPtr<IDXGISwapChain1> swapChain1; // Temporary Swap Chain 1 Interface
	DX(m_factory->CreateSwapChainForHwnd
	(
		m_cmdQueue.Get(), // Command Queue
		m_hWnd, // Window Handle
		&scDesc, // Swap Chain Description
		nullptr, // Fullscreen Desc (optional)
		nullptr, // Restrict to output (optional)
		swapChain1.ReleaseAndGetAddressOf()
	));

	// Get Swap Chain 4 Interface
	DX(swapChain1.As(&m_swapChain)); // Version up to IDXGISwapChain4

	// Disable Alt+Enter fullscreen toggle
	DX(m_factory->MakeWindowAssociation(m_hWnd, DXGI_MWA_NO_ALT_ENTER));

	// Create Render Target Views (RTVs) for each back buffer

	// Create RTV Descriptor Heap
	DX(CreateDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE_RTV, BACKBUFFER_COUNT, m_rtvHeap));
	m_rtvStride = m_device->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

	// Create RTV for each back buffer
	D3D12_CPU_DESCRIPTOR_HANDLE rtvHandle = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();

	// Back buffer RTV Creation Loop
	// TODO: TRY TO UNDERSTAND THIS PART CAREFULLY
	auto rtvStart = m_rtvHeap->GetCPUDescriptorHandleForHeapStart(); // Starting handle
	for (UINT i = 0; i < BACKBUFFER_COUNT; ++i)
	{
		// Get back buffer resource
		DX(m_swapChain->GetBuffer(i, IID_PPV_ARGS(m_renderTargets[i].ReleaseAndGetAddressOf())));
		D3D12_CPU_DESCRIPTOR_HANDLE h = rtvStart;
		h.ptr += SIZE_T(i) * m_rtvStride;
		// Create RTV for this back buffer
		m_device->CreateRenderTargetView(m_renderTargets[i].Get(), nullptr, h);
	}

	m_frameIndex = m_swapChain->GetCurrentBackBufferIndex(); // Get current back buffer index

	return S_OK;
}

HRESULT Renderer::CreateDepthBuffer_()
{
	// 1. Create DSV Descriptor Heap & Handle
	DX(CreateDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE_DSV, 1, m_dsvHeap));
	m_dsvHandle = m_dsvHeap->GetCPUDescriptorHandleForHeapStart();

	// 2. Create Depth Texture Resource Description
	D3D12_RESOURCE_DESC texture{};
	texture.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D;
	texture.Alignment = 0; // Default alignment
	texture.Width = m_width;
	texture.Height = m_height;
	texture.DepthOrArraySize = 1; // Single texture
	texture.MipLevels = 1; // KNOB: No mipmaps
	texture.Format = DEPTH_FORMAT; // 32-bit float depth format
	texture.SampleDesc.Count = 1; // KNOB: No MSAA
	texture.SampleDesc.Quality = 0; // KNOB: Standard quality level
	texture.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN; // KNOB: ?? Default layout ??
	texture.Flags = D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL; // IMPORTANT Allow depth stencil usage

	// 3. Create Optimized Clear Value for Depth Buffer
	D3D12_CLEAR_VALUE clearValue{};
	clearValue.Format = DEPTH_FORMAT;
	clearValue.DepthStencil.Depth = 1.0f; // KNOB: Default depth clear value
	clearValue.DepthStencil.Stencil = 0; // KNOB: Default stencil clear value

	DX(CreateTexture(texture, &clearValue, m_depth));

	// 4. Create Depth Stencil View (DSV)
	D3D12_DEPTH_STENCIL_VIEW_DESC view{};
	view.Format = DEPTH_FORMAT;
	view.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2D;
	view.Flags = D3D12_DSV_FLAG_NONE;

	m_device->CreateDepthStencilView(m_depth.Get(), &view, m_dsvHandle);

	// 5. Execute Barrier and Wait (COMMON -> DEPTH_WRITE)
	auto* cmd = BeginSetupCommands_(); if (!cmd) return E_FAIL;
	IssueBarrier(cmd, m_depth.Get(), D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_DEPTH_WRITE);

	ExecuteSetupCommandsAndWait_();

	return S_OK;
}

HRESULT Renderer::CreateBuffer(const D3D12_RESOURCE_DESC& desc, D3D12_HEAP_TYPE heapType, D3D12_RESOURCE_STATES initialState, ComPtr<ID3D12Resource>& buffer)
{
	buffer.Reset();

	D3D12_HEAP_PROPERTIES heapProps{};
	heapProps.Type = heapType;
	heapProps.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	heapProps.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;

	DX(m_device->CreateCommittedResource(
		&heapProps,
		D3D12_HEAP_FLAG_NONE,
		&desc,
		initialState,
		nullptr, // Buffers don't use optimized clear value
		IID_PPV_ARGS(buffer.ReleaseAndGetAddressOf())
	));

	return S_OK;
}

void Renderer::SetViewportAndScissor(ID3D12GraphicsCommandList* cmdList, float width, float height)
{
	D3D12_VIEWPORT viewport{};
	viewport.TopLeftX = 0.0f; viewport.TopLeftY = 0.0f;
	viewport.Width = width;
	viewport.Height = height;
	viewport.MinDepth = 0.0f; viewport.MaxDepth = 1.0f;

	D3D12_RECT scissorRect{};
	scissorRect.left = 0; scissorRect.top = 0;
	scissorRect.right = static_cast<LONG>(width);
	scissorRect.bottom = static_cast<LONG>(height);

	cmdList->RSSetViewports(1, &viewport);
	cmdList->RSSetScissorRects(1, &scissorRect);
}

HRESULT Renderer::CreateStaticBuffer(const void* data, UINT64 byteSize, ComPtr<ID3D12Resource>& gpuBuffer)
{
	if (byteSize == 0) return E_INVALIDARG; // Invalid size

	// Define resource description for buffer
	D3D12_RESOURCE_DESC bufferDesc{};
	bufferDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	bufferDesc.Width = byteSize;
	bufferDesc.Height = 1; // Buffers are 1D
	bufferDesc.DepthOrArraySize = 1; // Single buffer
	bufferDesc.MipLevels = 1; // No mipmaps
	bufferDesc.Format = DXGI_FORMAT_UNKNOWN; // Not a texture
	bufferDesc.SampleDesc.Count = 1; // No MSAA
	bufferDesc.SampleDesc.Quality = 0; // Standard quality level
	bufferDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR; // Row-major layout for buffers
	bufferDesc.Flags = D3D12_RESOURCE_FLAG_NONE; // No special flags

	// Create General Buffer
	DX(CreateBuffer(bufferDesc, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COPY_DEST, gpuBuffer));

	// If no data to upload, return here
	if (!data) return E_INVALIDARG;

	// Create Upload Buffer
	ComPtr<ID3D12Resource> uploadBuffer;
	DX(CreateBuffer(bufferDesc, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ, uploadBuffer));

	// Copy Data To Upload Buffer (Map, Copy, Unmap)
	void* mappedData = nullptr;
	DX(uploadBuffer->Map(0, nullptr, &mappedData));
	std::memcpy(mappedData, data, static_cast<size_t>(byteSize));
	uploadBuffer->Unmap(0, nullptr);

	// Copy Data From Upload Buffer To GPU Buffer
	auto* cmd = BeginSetupCommands_(); if (!cmd) return E_FAIL;

	// Copy from upload buffer to GPU buffer
	cmd->CopyBufferRegion(gpuBuffer.Get(), 0, uploadBuffer.Get(), 0, byteSize);

	// Barrier: COPY_DEST -> GENERIC_READ (maybe VERTEX_AND_CONSTANT_BUFFER would be better?)
	// TODO: Change to VERTEX_AND_CONSTANT_BUFFER if used only as vertex buffer
	IssueBarrier(cmd, gpuBuffer.Get(), D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_GENERIC_READ);

	ExecuteSetupCommandsAndWait_();

	return S_OK;
}

HRESULT Renderer::CreateTexture(const D3D12_RESOURCE_DESC& desc, const D3D12_CLEAR_VALUE* clearValue, ComPtr<ID3D12Resource>& texture)
{
	// 0. Reset output texture for safety and validate parameters
	texture.Reset();
	if (desc.Width == 0 || desc.Height == 0 || desc.DepthOrArraySize == 0) return E_INVALIDARG;

	// 1. Define heap properties for default (GPU) heap
	D3D12_HEAP_PROPERTIES heapProps{};
	heapProps.Type = D3D12_HEAP_TYPE_DEFAULT; // Default GPU heap
	heapProps.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	heapProps.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	heapProps.CreationNodeMask = 0; // Single GPU
	heapProps.VisibleNodeMask = 0; // Single GPU

	// 2. Create the texture resource
	DX(m_device->CreateCommittedResource(
		&heapProps,
		D3D12_HEAP_FLAG_NONE,
		&desc, // Texture description from parameter
		D3D12_RESOURCE_STATE_COMMON,
		clearValue, // Optional optimized clear value for RTV/DSV
		IID_PPV_ARGS(texture.ReleaseAndGetAddressOf())
	));

	return S_OK;
}

ID3D12GraphicsCommandList* Renderer::BeginSetupCommands_()
{
	// Reset command allocator and command list for setup commands
	HRESULT hr;
	hr = m_cmdAllocator[m_frameIndex]->Reset();
	if (FAILED(hr)) return nullptr;

	hr = m_cmdList->Reset(m_cmdAllocator[m_frameIndex].Get(), nullptr);
	if (FAILED(hr)) return nullptr;

	// Return command list for recording setup commands
	return m_cmdList.Get();
}

void Renderer::ExecuteSetupCommandsAndWait_()
{
	// Close command list
	HRESULT hr = m_cmdList->Close();
	if (FAILED(hr))
	{
		return;
	}

	ID3D12CommandList* cmdLists[] = { m_cmdList.Get() };
	m_cmdQueue->ExecuteCommandLists(1, cmdLists);

	// Wait for GPU to finish
	WaitForGPU_();
}

void Renderer::WaitForGPU_()
{
	// Wait for GPU to finish
	++m_globalFenceCount; // Increment global fence count
	m_cmdQueue->Signal(m_fence.Get(), m_globalFenceCount); // Tell GPU to signal when it reaches this point

	// Wait until GPU reaches the fence value
	if (m_fence->GetCompletedValue() < m_globalFenceCount)
	{
		m_fence->SetEventOnCompletion(m_globalFenceCount, m_fenceEvent);
		WaitForSingleObject(m_fenceEvent, INFINITE);
	}
}

void Renderer::BeginFrame()
{
	// Wait for previous frame
	const UINT64 fenceToWait = m_fenceValues[m_frameIndex];
	if (this->m_fence->GetCompletedValue() < fenceToWait)
	{
		m_fence->SetEventOnCompletion(fenceToWait, m_fenceEvent);
		WaitForSingleObject(m_fenceEvent, INFINITE);
	}

	// Reset command allocator and command list for current frame
	m_cmdAllocator[m_frameIndex]->Reset();
	m_cmdList->Reset(m_cmdAllocator[m_frameIndex].Get(), nullptr);

	// Set viewport and scissor according to window size
	SetViewportAndScissor(m_cmdList.Get(), static_cast<float>(m_width), static_cast<float>(m_height));

	// Barrier: Present -> Render Target
	IssueBarrier(m_cmdList.Get(), m_renderTargets[m_frameIndex].Get(), D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET);

	// Start Timer Query
	m_cmdList->EndQuery(m_queryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 0);

	// Set Render Target and Depth Stencil
	// TODO: Check if this is needed, can be written with helper or not
	D3D12_CPU_DESCRIPTOR_HANDLE rtvStart = m_rtvHeap->GetCPUDescriptorHandleForHeapStart();
	D3D12_CPU_DESCRIPTOR_HANDLE currenRTV = rtvStart;

	currenRTV.ptr += SIZE_T(m_frameIndex) * m_rtvStride;

	m_cmdList->OMSetRenderTargets(1, &currenRTV, FALSE, &m_dsvHandle);

	// Clear Render Target and Depth Stencil
	// TODO: Make clear color configurable
	const FLOAT clearColor[4] = { 0.0f, 0.01f, 0.05f, 1.0f }; // Clear color (black)
	m_cmdList->ClearRenderTargetView(currenRTV, clearColor, 0, nullptr); // Clear RTV
	m_cmdList->ClearDepthStencilView(m_dsvHandle, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr); // Clear DSV
}

void Renderer::EndFrame()
{
	// End Timer Query
	m_cmdList->EndQuery(m_queryHeap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, 1);
	ResolveQueryData(m_cmdList.Get()); // Resolve query data to readback buffer

	// Barrier: Render Target -> Present
	IssueBarrier(m_cmdList.Get(), m_renderTargets[m_frameIndex].Get(), D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT);

	// Close command list
	m_cmdList->Close();

	// Execute command list
	ID3D12CommandList* cmdLists[] = { m_cmdList.Get() };
	m_cmdQueue->ExecuteCommandLists(1, cmdLists);

	// Present the frame
	m_swapChain->Present(1, 0); // VSync enabled (1), Allow tearing

	++m_globalFenceCount; // Global sayacı artır (örn: 5 idi, 6 oldu)
	m_cmdQueue->Signal(m_fence.Get(), m_globalFenceCount); // GPU'ya "işin bitince Fence'i 6 yap" de.

	m_fenceValues[m_frameIndex] = m_globalFenceCount; // TODO: COMMENT GLOBAL FENCE COUNT
	// Update frame index
	m_frameIndex = m_swapChain->GetCurrentBackBufferIndex();
}

void Renderer::Shutdown()
{
	WaitForGPU_();

	if (m_fenceEvent) { CloseHandle(m_fenceEvent); m_fenceEvent = nullptr; }
	// ComPtrs get released automatically. But wait for GPU to finish first.
}

void Renderer::EnableDebugLayer_()
{
#if defined(_DEBUG)
	ComPtr<ID3D12Debug> debugController;
	if (SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debugController))))
	{
		debugController->EnableDebugLayer();
	}
	// Additional debug layers for GPU-based validation can be enabled here.
#endif
}

// =========================================================
// TIMESTAMP QUERY IMPLEMENTATION
// =========================================================

void Renderer::InitTimestampQuery()
{
	// 1. Create Query Heap
	D3D12_QUERY_HEAP_DESC heapDesc = {};
	heapDesc.Count = MAX_QUERY_COUNT;
	heapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
	heapDesc.NodeMask = 0;

	m_device->CreateQueryHeap(&heapDesc, IID_PPV_ARGS(m_queryHeap.ReleaseAndGetAddressOf()));

	// 2. Create Readback Buffer
	D3D12_RESOURCE_DESC bufferDesc = {};
	bufferDesc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	bufferDesc.Width = MAX_QUERY_COUNT * sizeof(uint64_t); // 2 queries * 8 bytes each
	bufferDesc.Height = 1;
	bufferDesc.DepthOrArraySize = 1;
	bufferDesc.MipLevels = 1;
	bufferDesc.Format = DXGI_FORMAT_UNKNOWN;
	bufferDesc.SampleDesc.Count = 1;
	bufferDesc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	bufferDesc.Flags = D3D12_RESOURCE_FLAG_NONE;

	// Heap Type: READBACK (GPU -> CPU)
	D3D12_HEAP_PROPERTIES heapProps = {};
	heapProps.Type = D3D12_HEAP_TYPE_READBACK;

	m_device->CreateCommittedResource(
		&heapProps,
		D3D12_HEAP_FLAG_NONE,
		&bufferDesc,
		D3D12_RESOURCE_STATE_COPY_DEST,
		nullptr,
		IID_PPV_ARGS(m_queryReadbackBuffer.ReleaseAndGetAddressOf()));
}

void Renderer::ResolveQueryData(ID3D12GraphicsCommandList* cmdList)
{
	// Write query data to readback buffer
	cmdList->ResolveQueryData(
		m_queryHeap.Get(),
		D3D12_QUERY_TYPE_TIMESTAMP,
		0,                  // Starting index
		MAX_QUERY_COUNT,    // How many to resolve? (2)
		m_queryReadbackBuffer.Get(),
		0                   // Buffer offset
	);
}

double Renderer::GetLastFrameDuration()
{


	// 1. Get GPU timestamp frequency
	uint64_t frequency = 0;
	if (FAILED(m_cmdQueue->GetTimestampFrequency(&frequency))) return 0.0;

	// 2. Read data from buffer
	uint64_t* data = nullptr;
	D3D12_RANGE readRange = { 0, MAX_QUERY_COUNT * sizeof(uint64_t) };

	// Map: Map buffer to CPU address space
	if (SUCCEEDED(m_queryReadbackBuffer->Map(0, &readRange, reinterpret_cast<void**>(&data))))
	{
		uint64_t start = data[0];
		uint64_t end = data[1];

		// Unmap
		m_queryReadbackBuffer->Unmap(0, nullptr);

		// 3. Calculate duration in milliseconds
		// Delta Tick / Frequency = Seconds
		// For milliseconds * 1000.0
		if (end < start) return 0.0; // Overflow protection

		return (static_cast<double>(end - start) / static_cast<double>(frequency)) * 1000.0;
	}

	return 0.0;
}


// =======================================================================================================
// GLITCH EFFECT METHODS AND MEMBERS
// =======================================================================================================

HRESULT Renderer::CreateGlitchHeap(UINT64 size, ComPtr<ID3D12Heap>& heap)
{
	heap.Reset();

	D3D12_HEAP_DESC heapDesc = {};
	heapDesc.SizeInBytes = size; // Size of the heap in bytes
	heapDesc.Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT; // For memory aliasing, use default alignment
	heapDesc.Properties.Type = D3D12_HEAP_TYPE_DEFAULT; // Default GPU heap, not UPLOAD
	heapDesc.Properties.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN; // Default CPU page property
	heapDesc.Properties.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN; // Default memory pool
	heapDesc.Flags = D3D12_HEAP_FLAG_ALLOW_ALL_BUFFERS_AND_TEXTURES; // Allow all resource types

	return m_device->CreateHeap(&heapDesc, IID_PPV_ARGS(heap.ReleaseAndGetAddressOf()));
}

HRESULT Renderer::CreatePlacedResource(ID3D12Heap* heap, UINT64 heapOffset, const D3D12_RESOURCE_DESC& desc, D3D12_RESOURCE_STATES initialState, const D3D12_CLEAR_VALUE* clearValue, ComPtr<ID3D12Resource>& resource)
{
	resource.Reset();

	return m_device->CreatePlacedResource(
		heap,
		heapOffset,
		&desc,
		initialState,
		clearValue,
		IID_PPV_ARGS(resource.ReleaseAndGetAddressOf())
	);
}