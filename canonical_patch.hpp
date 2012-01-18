#ifndef ECV_CANONICAL_PATCH_HPP
#define ECV_CANONICAL_PATCH_HPP

#include <ecv/image.hpp>
#include <ecv/transform.hpp>
#include <cmath>
#include <ecv/scale_space.hpp>

namespace ecv {

    Image<float> make_disc(int size);
    Image<float> make_gaussian_envelope(int size, float sigmas_at_size);
    
    template <class T>
    int find_orientations(const Image<T>& patch,
                          const Image<float>& weight,
                          float min_frac_of_max,
                          float orientations[],
                          int max_ori)
    {
        const int BINS = 32;
        const float rads_to_bin = BINS / (2 * M_PI);
        const float bin_to_rads = 1/rads_to_bin;
        
        float hist[BINS];
        
        for (int i=0; i<BINS; ++i)
            hist[i] = 0;


        assert(patch.size() == weight.size() + 2);

        const float beta = 0.5f;
        const int s = patch.stride();
        for (int i=1; i+1<patch.height(); ++i) {            
            const T* p = patch[i];
            const float *w = weight[i-1];
            for (int j=1; j+1<patch.width(); ++j) {
                float gx = p[j+1] - p[j-1];
                float gy = p[j+s] - p[j-s];
                
                float theta = atan2f(gy, gx);
                float mag = w[j-1] * sqrtf(gx*gx + gy*gy);

                float bin = theta * rads_to_bin;
                int lo = (int)bin;
                float t = bin - lo;
                lo = (lo + BINS) % BINS;
                int hi = (lo + 1) % BINS;
                int hi2 = (lo + 2) % BINS;
                int lo2 = (lo + (BINS-1)) % BINS;

                float ylo = (1-t)*mag;
                float yhi = t*mag;
                
                hist[lo] += ylo + beta * yhi;
                hist[hi] += yhi + beta * ylo;
                hist[lo2] += beta * ylo;
                hist[hi2] += beta * yhi;
            }
        }
        
        float minval;
        {
            int argmax = 0;
            for (int i=1; i<BINS; ++i) {
                if (hist[i] > hist[argmax])
                    argmax = i;
            }
            minval = min_frac_of_max * hist[argmax];
        }

        float peaks[BINS/2];
        float peakvals[BINS/2];
        int count = 0;
        float prev = hist[BINS-1];
        
        for (int i=0; i<BINS; ++i)
        {
            float mid = hist[i];
            float next = hist[(i+1) % BINS];
            if (mid >= minval && mid > prev && mid > next) {
                float A = 0.5f * (prev + next) - mid;
                float B = 0.5f * (next - prev);
                float x = -B / (2*A);
                peaks[count] = x + i;
                peakvals[count] = mid + x*(B + x*A);
                ++count;
            }
            prev = mid;
        }

        int k = count < max_ori ? count : max_ori;
        for (int i=0; i<k; ++i) {
            int argmax = i;
            for (int j=i+1; j<count; ++j)
                if (peakvals[j] > peakvals[argmax])
                    argmax = j;
            orientations[i] = peaks[argmax] * bin_to_rads;
            peaks[argmax] = peaks[i];
            peakvals[argmax] = peakvals[i];
        }
        return k;
    }

    template <class T>
    bool sample_scaled_rotated_patch(const Image<T>& src,
                                     const latl::Vector<2,float>& center,
                                     float scale,
                                     float theta,
                                     Image<T>& patch)
    {
        using namespace latl;
        
        float ct = cosf(theta);
        float st = sinf(theta);
        Matrix<2,2,float> sR;
        sR(0,0) = scale * ct;
        sR(0,1) = scale * -st;
        sR(1,0) = scale * st;
        sR(1,1) = scale * ct;

        Vector<2,float> patch_center;
        patch_center[0] = 0.5f * (patch.width()-1);
        patch_center[1] = 0.5f * (patch.height()-1);
        
        return warp_affine(src, center, patch_center, sR, patch);
    }


    struct CanonicalPatchExtractor
    {
        struct Options {
            int patch_size;
            int max_orientations;
            Options() {
                patch_size = 15;
                max_orientations = 4;
            }
        };
            
        const Options options;
        Image<float> ori_weight;
        float oris[16];

        CanonicalPatchExtractor(const Options& opt = Options())
            : options(opt)
        {                
            ori_weight = make_disc(options.patch_size-2);
        }
            
        template <class T, class Processor>
        void extract_canonical_patches(const scale_space::Pyramid<T>& pyr,
                                       const std::vector<scale_space::Point>& points,
                                       Processor& processor)
        {
            using namespace latl;

            Image<T> patch(options.patch_size, options.patch_size);

            for (size_t i=0; i<points.size(); ++i)
            {
                const scale_space::Point& p = points[i];
                const Image<T>& im = pyr.level[p.level];
                const Vector<2,float> where = makeVector(p.level_x, p.level_y);
                if (!sample_scaled_rotated_patch(im,
                                                 where,
                                                 p.level_scale,
                                                 0.f,
                                                 patch))
                {
                    continue;
                }

                int num_ori = find_orientations(patch,
                                                ori_weight,
                                                0.8f,
                                                oris,
                                                options.max_orientations);

                for (int j=0; j<num_ori; ++j) {
                    if (!sample_scaled_rotated_patch(im,
                                                     where,
                                                     p.level_scale,
                                                     oris[j],
                                                     patch))
                    {
                        continue;
                    }

                    processor(p, i, j, oris[j], patch);
                }
            }
        }
    };
    
}

#endif
