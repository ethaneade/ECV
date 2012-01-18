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
#ifndef ECV_ARCTAN_CAMERA_MODEL_HPP
#define ECV_ARCTAN_CAMERA_MODEL_HPP

#include <ecv/camera_model.hpp>

namespace ecv {

    class ArctanCameraModel : public CameraModel
    {
    public:
        ArctanCameraModel();

        enum { NUM_PARAMS = 5 };
        
        const char* name() const { return "arctan"; }
        int num_params() const { return NUM_PARAMS; }
        
        latl::Vector<-1,float> get_params() const;
        void set_params(const latl::Vector<-1,float>& p);        
        void update_params(const latl::Vector<-1,float>& dp);
                
    protected:
        latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                      latl::Matrix<2,2,float>* J_xy,
                                      latl::Matrix<2,-1,float>* J_model) const;
        
        latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv,
                                        latl::Matrix<2,2,float>* J_uv) const;

        void update_internal();

        latl::Vector<2,float> focal, inv_focal;
        latl::Vector<2,float> uv0;
        float radial;
    };
    
}

#endif
