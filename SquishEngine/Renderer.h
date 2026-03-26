#pragma once

// DirectX 12 Includes:
#include <d3d12.h>
#include <dxgi1_6.h>
#include <wrl.h>
#include <stdexcept> // For runtime error throwing
#include <iostream>  // Add this for std::cout and std::endl
#include "d3dcompiler.h" // Used for shader compilation

#pragma comment(lib, "d3dcompiler.lib") // Add library to Linker
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")


// Include Windows
#include <Windows.h>

#include <cstdint> // For uint32_t
#include <cstring> // For memcpy

// Namespaces:
// Alias for Microsoft::WRL::ComPtr. Smart pointer for COM objects.
template <typename T>
using ComPtr = Microsoft::WRL::ComPtr<T>;

/// <summary>
/// Renderer Configuration Structure
/// </summary>
struct RendererConfig
{
	// I will leave these here for future use
	// The following will be set during initialization for each benchmark from this config
	// Right now they are fixed in the Renderer class
	UINT width; UINT height;
	UINT BUFFER_COUNT = 2;
	DXGI_FORMAT backBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
	D3D12_COMMAND_LIST_TYPE commandListType; // Direct, Bundle, Compute, Copy
	UINT sampleCount; // MSAA
};

class Renderer
{
public:
	HRESULT Initialize(HWND hWnd, UINT width, UINT height);
	void BeginFrame();
	void EndFrame();
	void Shutdown();


	ID3D12GraphicsCommandList* BeginSetupCommands_(); // Begin Setup Commands (Used in RTV, DSV, Buffer Creation)
	void ExecuteSetupCommandsAndWait_(); // Execute Setup Commands and Wait

	// Getters
	ID3D12Device* GetDevice() const { return m_device.Get(); }
	ID3D12CommandQueue* GetCommandQueue() const { return m_cmdQueue.Get(); }
	ID3D12GraphicsCommandList* GetCommandList() const { return m_cmdList.Get(); }
	IDXGISwapChain4* GetSwapChain() const { return m_swapChain.Get(); }
	D3D12_CPU_DESCRIPTOR_HANDLE GetDSVHandle() const { return m_dsvHandle; }

	// BARRIER
	// Helper: Records a resource transition barrier into the command list.
	void IssueBarrier(ID3D12GraphicsCommandList* cmdList, ID3D12Resource* pResource, D3D12_RESOURCE_STATES stateBefore, D3D12_RESOURCE_STATES stateAfter);

	// Memory Resource Management Helpers

	// NOTE:
	// - Only use during initialization / setup, not in the middle of per-frame rendering.
	// - Uses internal command list / allocator and waits for GPU.'
	HRESULT CreateBuffer(const D3D12_RESOURCE_DESC& desc, D3D12_HEAP_TYPE heapType, D3D12_RESOURCE_STATES initialState, ComPtr<ID3D12Resource>& buffer); // Create Generic Buffer
	HRESULT CreateStaticBuffer(const void* data, UINT64 byteSize, ComPtr<ID3D12Resource>& gpuBuffer); // Create Vertex Buffer
	HRESULT CreateTexture(const D3D12_RESOURCE_DESC& desc, const D3D12_CLEAR_VALUE* clearValue, ComPtr<ID3D12Resource>& texture); // Create Texture Resource (for UAV/SRV)

	// Descriptor Heap Creation Helper
	// SHADER_VISIBLE should be true for SRV/UAV/CBV heaps used in shaders
	HRESULT CreateDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE type, UINT numDescriptors, ComPtr<ID3D12DescriptorHeap>& heap); // Create Descriptor Heap

	UINT Width() const { return m_width; }
	UINT Height() const { return m_height; }
	UINT FrameIndex() const { return m_frameIndex; }

	// Viewport ve Scissor Rect Setup in a Single Call
	void SetViewportAndScissor(ID3D12GraphicsCommandList* cmdList, float width, float height);

	// TIMESTAMP SYSTEM FOR GPU PROFILING
	void InitTimestampQuery();
	void ResolveQueryData(ID3D12GraphicsCommandList* cmdList);
	double GetLastFrameDuration(); // In milliseconds

private:
	// Debug Layer
	void EnableDebugLayer_();

	// Window Parameters
	HWND m_hWnd = nullptr;

	UINT m_width{ 800 };
	UINT m_height{ 800 };

	// DirectX 12 Core Components (DXGI/D3D12)

	// Core Constants
	static constexpr UINT BACKBUFFER_COUNT = 2; // Double Buffering (Alternative: Triple Buffering)
	static constexpr DXGI_FORMAT DEPTH_FORMAT = DXGI_FORMAT_D32_FLOAT; // 32-bit float depth format

	// Device
	ComPtr<IDXGIFactory7> m_factory; // 6 or 7 for modern features
	ComPtr<ID3D12Device> m_device;

	// Command Context: Queue, Allocator, List, Fence
	ComPtr<ID3D12CommandQueue> m_cmdQueue;
	ComPtr<ID3D12CommandAllocator> m_cmdAllocator[BACKBUFFER_COUNT];
	ComPtr<ID3D12GraphicsCommandList> m_cmdList;

	ComPtr<ID3D12Fence> m_fence; HANDLE m_fenceEvent = nullptr; UINT64 m_fenceValues[BACKBUFFER_COUNT] = {};

	// Swap Chain
	ComPtr<IDXGISwapChain4> m_swapChain; // Version 4 for modern features, Version 3 is also ok
	DXGI_FORMAT m_backBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM; // 8-bit RGBA format
	UINT m_frameIndex = 0;

	// Render Target Views (RTVs) and Render Targets
	ComPtr<ID3D12DescriptorHeap> m_rtvHeap;
	ComPtr<ID3D12Resource> m_renderTargets[BACKBUFFER_COUNT];
	UINT m_rtvStride = 0; // Size of RTV Descriptor

	// Depth Stencil View (DSV) and Depth Texture
	ComPtr<ID3D12DescriptorHeap> m_dsvHeap;
	ComPtr<ID3D12Resource> m_depth;
	D3D12_CPU_DESCRIPTOR_HANDLE m_dsvHandle{};

	// Viewport and Scissor Rect
	D3D12_VIEWPORT m_viewport{};
	D3D12_RECT     m_scissor{};
	// TODO: COMMENT
	UINT64 m_globalFenceCount = 0; // Continuous global fence counter

	// Creation Methods

	HRESULT CreateDevice_(); // Create DXGI Factory and D3D12 Device 
	HRESULT CreateCommandQueue_(); // Create Command Queue and Command Allocator
	HRESULT CreateCommandList_(); // Create Command List
	HRESULT CreateFence_(); // Create Fence for GPU synchronization
	HRESULT CreateSwapChain_(); // Create Swap Chain and Render Target Views (RTVs)

	HRESULT CreateDepthBuffer_(); // Create Depth Stencil View (DSV) and Depth Texture


	// TODO: Maybe add END Setup Commands function. (Used in RTV, DSV, Buffer Creation)
	// TODO: Maybe add Resize function.
	// TODO: Maybe seperate Command Queue and Command Allocator creation for more flexibility.

	void WaitForGPU_(); // Wait for GPU to finish


	// TIMESTAMP SYSTEM FOR GPU PROFILING

	ComPtr<ID3D12QueryHeap> m_queryHeap; // Pool of queries
	ComPtr<ID3D12Resource> m_queryReadbackBuffer; // GPU -> CPU data transfer buffer

	static constexpr UINT MAX_QUERY_COUNT = 2; // Start and End timestamps


	// =======================================================================================================
	// GLITCH EFFECT METHODS AND MEMBERS
	// =======================================================================================================
public:
	// Raw Heap Creation Helper for Memory Aliasing
	// NOTE: Only use during initialization / setup, not in the middle of per-frame rendering.
	HRESULT CreateGlitchHeap(UINT64 size, ComPtr<ID3D12Heap>& heap);

	// Adds a placed resource to the given heap at the specified offset.
	HRESULT CreatePlacedResource(ID3D12Heap* heap, UINT64 heapOffset, const D3D12_RESOURCE_DESC& desc, D3D12_RESOURCE_STATES initialState, const D3D12_CLEAR_VALUE* clearValue, ComPtr<ID3D12Resource>& resource);
};