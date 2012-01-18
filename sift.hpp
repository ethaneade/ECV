#ifndef ECV_SIFT_HPP
#define ECV_SIFT_HPP

#include <vector>
#include <cmath>
#include <stdint.h>
#include <ecv/scale_space.hpp>
#include <ecv/canonical_patch.hpp>

#if SAVE_PATCHES
#include <fstream>
#include <ecv/io_pgm.hpp>
#include <ecv/convert.hpp>
#endif

namespace ecv {

    namespace sift {

        struct HistogramEntry {
            int ylo, xlo, dx, dy;
            float weight[4];
        };
        
        void build_histogram_entries(int bins, int angles, int patch_size, double sigmas_at_edge,
                                     std::vector<HistogramEntry>& entries);
        
        template <int Bins, int Angles>
        class DescriptorExtractor
        {
        public:
            DescriptorExtractor(int patch_size_, double sigmas_at_edge)
            {
                size = patch_size_-2;
                build_histogram_entries(Bins, Angles, patch_size_, sigmas_at_edge,
                                        entries);
            }

            int patch_size() const { return size + 2; }

            template <class T>
            void extract(const T* patch, int stride,
                         float desc[Bins*Bins*Angles]) const
            {
                const HistogramEntry* e = &entries[0];
                const float rads_to_bin = Angles / (2*M_PI);

                const int N = Bins*Bins*Angles;
                for (int i=0; i<N; ++i)
                    desc[i] = 0;
            
                for (int i=0; i<size; ++i) {
                    patch += stride;
                    for (int j=1; j<=size; ++j, ++e) {
                        float gx = patch[j+1] - patch[j-1];
                        float gy = patch[j+stride] - patch[j-stride];

                        float theta = atan2f(gy, gx);
                        float mag = sqrtf(gx*gx + gy*gy);
                    
                        float tbin = rads_to_bin * (theta + M_PI);
                        int lo = (int)floorf(tbin);
                        float t = tbin - lo;
                        lo = (lo + Angles) % Angles;
                        int hi = (lo + 1) % Angles;

                        float wlo = (1-t) * mag;
                        float whi = t * mag;

                        float *d = &desc[(e->ylo*Bins + e->xlo)*Angles];
                    
                        d[lo] += e->weight[0] * wlo;
                        d[hi] += e->weight[0] * whi;
                    
                        d[e->dx + lo] += e->weight[1] * wlo;
                        d[e->dx + hi] += e->weight[1] * whi;

                        d += e->dy;
                    
                        d[lo] += e->weight[2] * wlo;                    
                        d[hi] += e->weight[2] * whi;
                    
                        d[e->dx + lo] += e->weight[3] * wlo;
                        d[e->dx + hi] += e->weight[3] * whi;
                    }
                }
                
                float sum_sq = 0;
                for (int i=0; i<N; ++i)
                    sum_sq += desc[i] * desc[i];
            
                float factor = 1.0f/sqrtf(sum_sq);
                sum_sq = 0;
                for (int i=0; i<N; ++i) {
                    float y = desc[i] * factor;
                    if (y > 0.2)
                        y = 0.2;
                    desc[i] = y;
                    sum_sq += y*y;
                }
            
                factor = 1.0f/sqrtf(sum_sq);
                for (int i=0; i<N; ++i)
                    desc[i] *= factor;
            }
        
        private:
            int size;
            std::vector<HistogramEntry> entries;        
        };


        template <int Bins, int Angles>
        struct Descriptor {
            enum {Size = Bins*Bins*Angles};
            unsigned int index;
            float orientation;
            unsigned long sum_sq;
            uint8_t d[Size];
        };

        template <int Bins, int Angles>
        struct Extractor
        {
            struct Options : CanonicalPatchExtractor::Options {
                double sigmas_at_edge;
                Options() {
                    sigmas_at_edge = 2.0;
                }
            };
            
            enum {DESC_SIZE = Bins*Bins*Angles};

            const Options options;
            CanonicalPatchExtractor cpe;
            DescriptorExtractor<Bins,Angles> desc_extractor;
            
            float descf[DESC_SIZE];

            Extractor(const Options& opt = Options())
                : options(opt), cpe(opt), desc_extractor(opt.patch_size, opt.sigmas_at_edge)
            {
            }
            
            struct PatchProcessor
            {
                std::vector<Descriptor<Bins,Angles> > descs;
                
                void operator()(const scale_space::Point& point,
                                size_t index, int ori_index, float ori,
                                const Image<float>& patch)
                {
#if SAVE_PATCHES
                    {
                        static int counter = 0;
                        char buf[50];
                        sprintf(buf, "patch_%06d.pgm", counter);
                        std::ofstream out(buf);
                        pgm_write(convert<uint8_t>(patch), out);
                        ++counter;
                    }
#endif
                    
                    desc_extractor.extract(patch.data(), patch.stride(), descf);
                    descs.resize(descs.size() + 1);
                    Descriptor<Bins,Angles>& desc = descs.back();
                    desc.index = index;
                    desc.orientation = ori;
                    unsigned long sum_sq = 0;
                    for (int k=0; k<DESC_SIZE; ++k) {
                        int y = (int)(descf[k] * 512 + 0.5f);
                        if (y > 255)
                            y = 255;
                        //std::cerr << descf[k] << " ";
                        sum_sq += y*y;
                        desc.d[k] = y;
                    }
                    //std::cerr << "\n";
                    desc.sum_sq = sum_sq;
                    //std::cerr << sum_sq << "\n";
                }
            };
            
            template <class T>
            void extract_descriptors(const scale_space::Pyramid<T>& pyr,
                                     const std::vector<scale_space::Point>& points,
                                     std::vector<Descriptor<Bins,Angles> >& descs)
            {
                PatchProcessor processor;
                cpe.extract_canonical_patches(pyr, points, processor);
                descs.swap(processor.descs);
            }
        };
    }        
}

#endif
