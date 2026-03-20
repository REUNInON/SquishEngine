struct PS_INPUT
{
    float4 Pos : SV_POSITION;
};

// PIXEL SHADER
// Color the pixel with a bright neon green color.
float4 PSMain(PS_INPUT input) : SV_TARGET
{
    // R, G, B, A (Neon Green)
    return float4(0.2f, 1.0f, 0.2f, 1.0f);
}