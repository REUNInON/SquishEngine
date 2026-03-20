struct VS_INPUT
{
    float2 Pos : POSITION;
};

struct PS_INPUT
{
    float4 Pos : SV_POSITION;
};

// VERTEX SHADER
// Output the position of the vertex in clip space.
PS_INPUT VSMain(VS_INPUT input)
{
    PS_INPUT output;
    output.Pos = float4(input.Pos.x, input.Pos.y, 0.0f, 1.0f);
    return output;
}