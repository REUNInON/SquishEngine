# 2D Softbody Physics Engine (Position Based Dynamics)

Real-Time soft-body physics engine that uses Position Based Dynamics (PBD) built with DirectX 12 and C++.

## Mathematical Foundations & Methods

### 1. `PhysicsEngine::Integrate` (Time Step & Acceleration)
Particle acceleration, velocity, and position updates.

*   **Newton's 2nd Law of Motion:**
    $$a = F \cdot w$$
    *(Where* $w = \frac{1}{m}$ *, Inverse Mass)*
*   **Frame-Rate Independent Damping (Drag):**
    $$v_{t+1} = v_t \cdot \text{drag}^{\Delta t}$$
*   **Symplectic Euler Integration:**
    $$v_{t+1} = v_t + a \cdot \Delta t$$
    $$x_{t+1} = x_t + v_{t+1} \cdot \Delta t$$

### 2. `PhysicsEngine::SolveConstraints` -> Distance Constraints
Spring/distance constraint solver between two particles (Hooke's Law variation).

*   **Euclidean Distance:**
    $$d = \sqrt{\Delta x^2 + \Delta y^2}$$
*   **Constraint Error Function:**
    $$C(x_1, x_2) = d - d_{rest}$$
*   **PBD Position Correction:**
    Correction factor proportional to inverse mass ($w_1, w_2$) and spring stiffness ($k$).
    $$\Delta \vec{p_1} = -\frac{w_1}{w_1 + w_2} C(x_1, x_2) \frac{\Delta \vec{p}}{d} \cdot k$$
    $$\Delta \vec{p_2} = +\frac{w_2}{w_1 + w_2} C(x_1, x_2) \frac{\Delta \vec{p}}{d} \cdot k$$

### 3. `PhysicsEngine::SolveConstraints` -> Area Constraints
Internal volume/area preservation solver for closed polygons (jelly/balloon).

*   **Shoelace Formula (Polygon Area):**
    $$A = \frac{1}{2} \sum_{i=1}^{n} (x_i y_{i+1} - x_{i+1} y_i)$$
*   **Area Error Function:**
    $$C(x) = A_{current} - (A_{rest} \cdot \text{pressure})$$
*   **Gradients (Push Directions):**
    $$\nabla_{x_i} C = \frac{1}{2}(y_{i+1} - y_{i-1})$$
    $$\nabla_{y_i} C = \frac{1}{2}(x_{i-1} - x_{i+1})$$
*   **Lagrange Multiplier ($\lambda$):**
    $$\lambda = \frac{-C(x)}{\sum (|\nabla C_i|^2 \cdot w_i)}$$
*   **Final Position Update:**
    $$\Delta p_i = \lambda \cdot w_i \cdot \nabla C_i$$

### 4. `PhysicsEngine::SolveConstraints` -> Particle Collisions
$O(N)$ collision resolution using Spatial Hash Grid.

*   **Distance Check (Broad & Narrow Phase):**
    $$d^2 < (2r)^2$$
*   **Penetration Depth:**
    $$p = 2r - d$$
*   **Collision Normal:**
    $$\vec{n} = \frac{\Delta \vec{p}}{d}$$
*   **Impulse / Correction Response:**
    $$\Delta \vec{p_1} = \vec{n} \cdot \frac{p}{w_1 + w_2} \cdot w_1$$
