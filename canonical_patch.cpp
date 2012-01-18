#include <ecv/canonical_patch.hpp>

using namespace ecv;

Image<float> ecv::make_disc(int size)
{
    Image<float> disc(size,size);
    float c = (size-1)*0.5f;
    float inner_R = c*c;
    float outer_R = (c+1)*(c+1);

    for (int i=0; i<size; ++i)
    {
        float dy2 = (i-c) * (i-c);
        float *o = disc[i];
        for (int j=0; j<size; ++j) {
            float r2 = dy2 + (j-c) * (j-c);
            if (r2 > outer_R) {
                o[j] = 0;
            } else if (r2 < inner_R) {
                o[j] = 1;
            } else {
                float t = sqrtf(r2) - c;
                o[j] = 1 - t;
            } 
        }
    }
    return disc;
}

Image<float> ecv::make_gaussian_envelope(int size, float sigmas_at_size)
{
    Image<float> g(size,size);
    float c = (size-1)*0.5f;
    float factor = -0.5 * latl::sq(sigmas_at_size / c);    

    for (int i=0; i<size; ++i)
    {
        float dy2 = (i-c) * (i-c);
        float *o = g[i];
        for (int j=0; j<size; ++j) {
            float r2 = dy2 + (j-c) * (j-c);
            o[j] = expf(r2 * factor);
        }
    }
    return g;
}
