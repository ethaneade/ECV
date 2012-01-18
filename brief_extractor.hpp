#ifndef ECV_BRIEF_EXTRACTOR_HPP
#define ECV_BRIEF_EXTRACTOR_HPP

#include <ecv/brief.hpp>
#include <ecv/brief_7_128.hpp>
#include <ecv/canonical_patch.hpp>
#include <vector>

namespace ecv {
    namespace brief {

        template <int Bits>
        struct Descriptor
        {
            unsigned int index;
            float orientation;
            BRIEF_Descriptor<Bits> desc;

            struct GetBit {
                bool operator()(const Descriptor& d, unsigned int i) const
                {
                    return d.desc.x[i/32] & (1u << (i%32));
                }
            };
            
        };        

        template <int Bits>
        struct Extractor
        {
            struct Processor
            {
                std::vector<Descriptor<Bits> > descs;
            
                template <class T>
                void operator()(const scale_space::Point &p, size_t i, int j, float ori, const Image<T> &patch)
                {
                    descs.resize(descs.size()+1);
                    descs.back().index = i;
                    descs.back().orientation = ori;
                    BRIEF_7_128_extract(patch[patch.height()/2] + patch.width()/2, patch.stride(), descs.back().desc.x);
                }
            };

            CanonicalPatchExtractor cpe;
            Extractor(const CanonicalPatchExtractor::Options& opts) : cpe(opts) {}

            template <class T>
            void extract_descriptors(const scale_space::Pyramid<T>& pyr,
                                     const std::vector<scale_space::Point>& points,
                                     std::vector<Descriptor<Bits> >& descs)
            {
                Processor processor;
                cpe.extract_canonical_patches(pyr, points, processor);
                descs.swap(processor.descs);
            }
        };
    }        
}

#endif

