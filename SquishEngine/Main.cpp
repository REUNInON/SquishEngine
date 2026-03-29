#include "Renderer.h"
#include "PhysicsEngine.h"
#include "ParticlePipeline.h"
#include <windows.h>

#include <fstream>
#include <string>
#include <vector>

// GLOBAL VARIABLES
Renderer g_renderer;
PhysicsEngine g_physics;
ParticlePipeline g_pipeline;


// Window Creation Helper
HWND SetupWindow(HINSTANCE hInstance, int width, int height);


// ============================================================================
// MAIN ENTRY POINT
// ============================================================================
int WINAPI wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR lpCmdLine, _In_ int nCmdShow)
{

    // 0. UNUSED PARAMETERS
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // AllocConsole(); // Console for debugging
    FILE* stream; freopen_s(&stream, "CONOUT$", "w", stdout);

    // 1. OPEN WINDOW
    const UINT WIDTH = 750;
    const UINT HEIGHT = 750;
    HWND hWnd = SetupWindow(hInstance, WIDTH, HEIGHT);

    if (!hWnd) return -1;

    // 2. INITIALIZE RENDERER
    if (FAILED(g_renderer.Initialize(hWnd, WIDTH, HEIGHT)))
    {
        MessageBox(hWnd, L"Failed to initialize Renderer!", L"Error", MB_OK | MB_ICONERROR);
        return -1;
    }

	// 3. INITIALIZE PARTICLE PIPELINE
    if (FAILED(g_pipeline.Initialize(g_renderer)))
    {
        MessageBox(hWnd, L"Failed to initialize Particle Pipeline!", L"Error", MB_OK | MB_ICONERROR);
        return -1;
	}

    //g_physics.CreateJellyBox(-0.3f, 1.0f, 0.6f, 1.0f, 0.0075f); // Example Jelly Box

    //g_physics.CreateJellyBall(0, 5, 0.4f, 16, 1.0f, 0.08f);

    g_physics.CreateRealisticJiggle(0.0f, 0.5f, 0.25f, 1.0f, 0.2f);

    // 5. GAME LOOP

    MSG msg = {};
    while (msg.message != WM_QUIT)
    {
        if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        else
        {
			// ===========================================================================
			// SIMULATION STEP
			// ============================================================================
			g_physics.StepSimulation(0.016f); // Simulate at 60 FPS, fixed time step

            // ===========================================================================
            // RENDERING CODE
            // ===========================================================================
            g_renderer.BeginFrame();
			g_pipeline.Render(g_renderer, g_physics.GetParticles(), g_physics.GetDistanceConstraints());
            g_renderer.EndFrame();
        }
    }

    g_renderer.Shutdown();

    return (int)msg.wParam;
}

// ============================================================================
// WINDOW CREATION HELPER (Standard Windows)
// ============================================================================
LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_KEYDOWN:
        if (wParam == VK_ESCAPE) PostQuitMessage(0);
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProc(hWnd, message, wParam, lParam);
}

HWND SetupWindow(HINSTANCE hInstance, int width, int height)
{
    const wchar_t* CLASS_NAME = L"BenchmarkWindow";
    WNDCLASSEX wc = {};
    wc.cbSize = sizeof(WNDCLASSEX);
    wc.style = CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = CLASS_NAME;
    RegisterClassEx(&wc);

    RECT rc = { 0, 0, (LONG)width, (LONG)height };
    AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);

    HWND hWnd = CreateWindow(CLASS_NAME, L"BenchmarkDX12",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left, rc.bottom - rc.top,
        nullptr, nullptr, hInstance, nullptr);

    return hWnd;
}