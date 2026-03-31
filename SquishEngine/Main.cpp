#include "Renderer.h"
#include "PhysicsEngine.h"
#include "ParticlePipeline.h"
#include <windows.h>

#include <chrono>

// =====================================================================================================
// PLEASE GO TO THE "SIMULATIONS" REGION IN THE MAIN FUNCTION TO SWITCH BETWEEN DIFFERENT SIMULATIONS!!
// =====================================================================================================

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

#pragma region Simulations

	// ============================================================================
	int sim = 2; // Change this to switch between different simulations!!
	// NOTE: To change simulation overall softness, adjust the solver iterations with g_physics.SetWorldSoftness(iterations) (More iterations = stiffer body, Less iterations = softer body)
    // ============================================================================

	if (sim == 0)
    {
        // ============================================================================
        // JELLY BOX
        // ============================================================================
        g_physics.SetWorldSoftness(16);
        g_physics.CreateJellyBox(0.0f, 0.75f, 0.75f, 1.0f, 0.5f);
    }
    else if (sim == 1)
    {
        // ============================================================================
        // SMALL & BIG JELLY BALLS
        // ============================================================================
        g_physics.SetWorldSoftness(10);
        g_physics.CreateSoftBall(0.0f, 0.0f, 0.1f, 0.2f, 0.5f);
        g_physics.CreateSoftBall(0.0f, -0.6f, 0.25f, 1.0f, 0.01f);

    }
    else if (sim == 2)
    {
        // ============================================================================
		// SMALL JELLY BALL PLAYGROUND
        // ============================================================================
        g_physics.SetWorldSoftness(16);
        g_physics.CreateSoftBall(0.0f, 0.5f, 0.1f, 0.2f, 0.25f);
        g_physics.CreateSoftBall(0.1f, 0.25f, 0.1f, 0.2f, 0.25f);
        g_physics.CreateSoftBall(0.1f, 0.0f, 0.1f, 0.4f, 0.25f);
        g_physics.CreateSoftBall(-0.3f, 0.0f, 0.1f, 0.2f, 0.25f);
        g_physics.CreateSoftBall(0.25f, -0.8f, 0.1f, 0.2f, 0.75f);
        g_physics.CreateSoftBall(0.0f, -0.8f, 0.1f, 0.5f, 0.25f);
        g_physics.CreateSoftBall(-0.25f, -0.8f, 0.1f, 0.2f, 0.2f);
    }
    else if (sim == 3)
    {
        // ============================================================================
        // COLLIDING JELLY BALLS
		// ============================================================================
        g_physics.SetWorldSoftness(16);
        g_physics.CreateSoftBall(0.0f, 0.0f, 0.25f, 1.0f, 0.25f);
        g_physics.CreateSoftBall(0.0f, 0.5f, 0.2f, 1.0f, 0.1f);
    }
    else if (sim == 4)
    {
        // ============================================================================
        // JELLY BALL
        // ============================================================================
        g_physics.SetWorldSoftness(8);
        g_physics.CreateSoftBall(0.0f, 2.0f, 0.25f, 1.0f, 0.75f);
    }
	else if (sim == 5)
    {
        // ============================================================================
        // SOFT JELLY BALL
        // ============================================================================
		g_physics.SetWorldSoftness(5);
        g_physics.CreateSoftBall(0.0f, 0.0f, 0.45f, 1.0f, 0.01f);
    }
    else if (sim == 6)
    {
        // ============================================================================
		// HANGING JELLY
        // ============================================================================
        g_physics.CreateHangingJelly(0.0f, 0.25f, 0.5f, 1.0f, 0.1f);
    }
    else if (sim == 7)
    {
        // ============================================================================
		// HANGING JELLY & SOFT BALL
        // ============================================================================
        g_physics.CreateHangingJelly(0.0f, 0.0f, 0.5f, 1.0f, 0.01f);
        g_physics.CreateSoftBall(0.0f, -0.8f, 0.1f, 1.0f, 0.25f);
    }
    else
    {
        // ============================================================================
		// SOFT HANGING JELLY
        // ============================================================================
        g_physics.CreateHangingJelly(0.0f, 0.0f, 0.5f, 1.0f, 0.01f);
    }

#pragma endregion

    // 5. GAME LOOP

    MSG msg = {};

	auto prevTime = std::chrono::high_resolution_clock::now();
	float accumulator = 0.0f;

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
			// FIXED TIMESTEP SIMULATION
            // ===========================================================================
			auto currentTime = std::chrono::high_resolution_clock::now();
			float frameTime = std::chrono::duration<float>(currentTime - prevTime).count();
			prevTime = currentTime; if (frameTime > 0.25f) frameTime = 0.25f; // Avoid deadly long frames

			accumulator += frameTime;

			// ===========================================================================
			// SIMULATION STEP
			// ============================================================================
			const float fixedDeltaTime = 0.016f; // 60 FPS

            while (accumulator >= fixedDeltaTime)
            {
                g_physics.StepSimulation(fixedDeltaTime); // Simulate at 60 FPS, fixed time step 0.016 seconds (16 ms)
                accumulator -= fixedDeltaTime;
            }

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
    const wchar_t* CLASS_NAME = L"SimulationWindow";
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

    HWND hWnd = CreateWindow(CLASS_NAME, L"Squish Engine",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT,
        rc.right - rc.left, rc.bottom - rc.top,
        nullptr, nullptr, hInstance, nullptr);

    return hWnd;
}