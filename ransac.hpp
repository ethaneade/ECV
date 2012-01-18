// Copyright 2011 Ethan Eade. All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:

//    1. Redistributions of source code must retain the above
//       copyright notice, this list of conditions and the following
//       disclaimer.

//    2. Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials
//       provided with the distribution.

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

// The views and conclusions contained in the software and
// documentation are those of the authors and should not be
// interpreted as representing official policies, either expressed or
// implied, of Ethan Eade.

#ifndef ECV_RANSAC_HPP
#define ECV_RANSAC_HPP

#include <cassert>
#include <vector>
#include <algorithm>
#include <cstdlib>

namespace ecv {

    struct RNG_erand48
    {
        unsigned short xsub[3];
        RNG_erand48(int seed = 47) {
            xsub[0] = seed;
            xsub[1] = seed >> 16;
            xsub[2] = 1;
        }
        size_t operator()(size_t n) {
            return (size_t)(n * erand48(xsub));
        }
    };
    
    template <int K, class RNG>
    void choose_subset(size_t n, RNG& rng, size_t which[])
    {
        for (int i=0; i<K; ++i) {
        gen_r:
            size_t r = rng(n);
            for (int j=0; j<i; ++j)
                if (r == which[j])
                    goto gen_r;
            which[i] = r;
        }
    }

    
    template <int K, class X, class Model, class Generator, class RNG>
    void generate_hypotheses_uniform(const std::vector<X>& x,
                                     const Generator& gen,
                                     size_t samples,
                                     std::vector<Model>& models,
                                     RNG& rng)
    {
        models.clear();
        
        if (K > (int)x.size())
            return;

        size_t which[K];
        X const * sample[K];
        
        for (size_t i=0; i<samples; ++i)
        {
            choose_subset<K>(x.size(), rng, which);
            for (int k=0; k<K; ++k)
                sample[k] = &x[which[k]];
            gen(sample, models);
        }
    }

    template <int K, class X, class Model, class Generator, class RNG>
    void generate_hypotheses_prosac(const std::vector<X>& x,
                                    const Generator& gen,
                                    size_t samples, size_t cutoff,
                                    std::vector<Model>& models,
                                    RNG& rng)
    {
        models.clear();
        if (x.size() < cutoff)
            cutoff = x.size();
        
        if (K > (int)cutoff)
            return;

        size_t which[K];
        X const * sample[K];
        
        for (size_t i=0; i<samples; ++i)
        {
            choose_subset<K>(std::min((size_t)(i+K), cutoff), rng, which);
            for (int k=0; k<K; ++k)
                sample[k] = &x[which[k]];
            gen(sample, models);
        }
    }


    template <class X, class Model, class Test>
    size_t test_hypotheses(const std::vector<X>& x,
                           const std::vector<Model>& models,
                           const Test& test,
                           size_t& argbest,
                           std::vector<bool>& inlier)
    {
        argbest = 0;
        size_t most_inliers = 0;

        inlier.resize(x.size());
        std::vector<bool> inlier_for_this(x.size());
        
        for (size_t i=0; i<models.size(); ++i) {
            size_t num_inliers = 0;
            for (size_t j=0; j<x.size(); ++j) {
                if (test(models[i], x[j])) {
                    ++num_inliers;
                    inlier_for_this[j] = true;
                } else { 
                    if (most_inliers >= num_inliers + x.size() - (i+1))
                        break;
                    inlier_for_this[j] = false; 
                }
            }
            if (num_inliers > most_inliers) {
                most_inliers = num_inliers;
                inlier.swap(inlier_for_this);
                argbest = i;
            }
        }
        return most_inliers;
    }

    template <class X, class Model, class Test>
    size_t test_hypotheses_preemptive(const std::vector<X>& x,
                                      const std::vector<Model>& models,
                                      const Test& test,
                                      size_t block_size,
                                      size_t& argbest,
                                      std::vector<bool>& inlier)
    {
        std::vector<std::pair<size_t,size_t> > votes(models.size());
        for (size_t i=0; i<models.size(); ++i) {
            votes[i].first = 0;
            votes[i].second = i;
        }

        for (size_t j=0; j < x.size();)
        {
            const size_t end = std::min(x.size(), j + block_size);

            for (size_t i=0; i<votes.size(); ++i) {
                const Model& model  = models[votes[i].second];
                size_t count = 0;
                for (size_t k=j; k<j+end; ++k) {
                    if (test(model, x[k]))
                        ++count;
                }
                votes[i].first += count;
            }

            size_t cutoff = votes.size() / 2;
            std::nth_element(votes.begin(), votes.begin() + cutoff, votes.end(), std::greater<std::pair<size_t,size_t> >());
            votes.resize(cutoff);
            if (votes.size() == 1)
                break;

            j = end;
        }
        argbest = std::max_element(votes.begin(), votes.end())->second;

        inlier.resize(x.size());
        size_t num_inliers = 0;
        for (size_t j=0; j<x.size(); ++j) {
            if (test(models[argbest], x[j])) {
                ++num_inliers;
                inlier[j] = true;
            } else
                inlier[j] = false;
        }
        return num_inliers;
    }
}

#endif
