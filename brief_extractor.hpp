// Copyright 2011 Ethan Eade. All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//    1. Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.
// 
//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY ETHAN EADE ``AS IS'' AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL ETHAN EADE OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
// 
// The views and conclusions contained in the software and
// documentation are those of the authors and should not be
// interpreted as representing official policies, either expressed or
// implied, of Ethan Eade.
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

