struct PS_INPUT
{
    float4 Pos : SV_POSITION;
};

// PIXEL SHADER
// Color the pixel with a bright neon green color.
float4 PSMain(PS_INPUT input) : SV_TARGET
{
    // R, G, B, A
    return float4(0.8f, 0.0f, 1.0f, 1.0f);
}