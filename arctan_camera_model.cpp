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
#include <ecv/arctan_camera_model.hpp>

using namespace latl;
using namespace ecv;

ecv::ArctanCameraModel::ArctanCameraModel()
{
    fill(uv0, 0.5f);
    fill(focal, 1.0f);
    radial = 0.01f;
    update_internal();
}
        
Vector<-1,float> ecv::ArctanCameraModel::get_params() const
{
    Vector<-1,float> p(5);
    slice<0,2>(p) = uv0;
    slice<2,2>(p) = focal;
    p[4] = radial;
    return p;
}

void ecv::ArctanCameraModel::update_internal()
{
    inv_focal[0] = 1/focal[0];
    inv_focal[1] = 1/focal[1];
}

void ecv::ArctanCameraModel::set_params(const Vector<-1,float>& p)
{
    assert(p.size() == 5);
    uv0 = slice<0,2>(p);
    focal = slice<2,2>(p);
    radial = p[4];
    update_internal();
}


void ecv::ArctanCameraModel::update_params(const Vector<-1,float>& dp)
{
    assert(dp.size() == 5);
    uv0 += slice<0,2>(dp);
    focal += slice<2,2>(dp);
    radial *= latl::exp(dp[4]);
    update_internal();
}
                
static const float third = 1.0f/3;
static const float fifth = 0.2f;    

Vector<2,float> ecv::ArctanCameraModel::project(const Vector<2,float>& xy,
                                                Matrix<2,2,float>* J_xy,
                                                Matrix<2,-1,float>* J_model) const
{
    const float R = norm_sq(xy);
    const float r = latl::sqrt(R);
    const float A = radial * r;
    const float Asq = A*A;
    
    float D, diffD, diffDxy;
    if (Asq < Constants<float>::sqrt_epsilon()) {
        D = 1 - Asq*(third - Asq*fifth);
        diffDxy = -radial * (2*third - Asq*(4*fifth));
        diffD = r * diffDxy;
    } else {
        float invA = 1/A;
        D = latl::atan(A) * invA;
        diffD = invA * (1/(Asq + 1) - D);
        diffDxy = diffD * (invA * radial);
    }

    Vector<2,float> d = D * xy;
    Vector<2,float> uv = diagmult(focal, d) + uv0;

    if (J_xy) {
        float dx = diffDxy * xy[0];
        float dy = diffDxy * xy[1];
        (*J_xy)(0,0) = focal[0] * (D + dx*xy[0]);
        (*J_xy)(0,1) = focal[0] * (dx*xy[1]);
        (*J_xy)(1,0) = focal[1] * (dy*xy[0]);
        (*J_xy)(1,1) = focal[1] * (D + dy*xy[1]);
    }

    if (J_model) {
        (*J_model)(0,0) = (*J_model)(1,1) = 1;
        (*J_model)(0,1) = (*J_model)(1,0) = 0;
        (*J_model)(0,2) = d[0];
        (*J_model)(1,2) = 0;
        (*J_model)(0,3) = 0;
        (*J_model)(1,3) = d[1];
        (*J_model)(0,4) = focal[0] * (diffD * A * xy[0]);
        (*J_model)(1,4) = focal[1] * (diffD * A * xy[1]);
    }

    return uv;
}
        
Vector<2,float> ecv::ArctanCameraModel::unproject(const Vector<2,float>& uv,
                                                  Matrix<2,2,float>* J_uv) const
{
    Vector<2,float> d = diagmult(inv_focal, uv - uv0);
    float r = latl::sqrt(norm_sq(d));
    float A = radial * r;
    float invD, diff_times_a_over_r;
    if (A < Constants<float>::sqrt_epsilon()) {
        float Asq = A*A;
        invD = 1 + Asq*third*(1 - Asq*2*fifth);
        diff_times_a_over_r = 2*third*(radial*radial)*(1 + 4*fifth*Asq);
    } else {
        float invA = 1 / A;
        float secA = 1/latl::cos(A);
        invD = latl::tan(A) * invA;
        diff_times_a_over_r = latl::sq(radial*invA) * (secA*secA - invD);
    }
    
    if (J_uv) {
        float a = d[0] * diff_times_a_over_r;
        float b = d[1] * diff_times_a_over_r;
        (*J_uv)(0,0) = inv_focal[0] * (invD + d[0] * a);
        (*J_uv)(0,1) = inv_focal[1] * (d[1] * a);
        (*J_uv)(1,0) = inv_focal[0] * (d[0] * b);
        (*J_uv)(1,1) = inv_focal[1] * (invD + d[1] * b);
    }
    
    return d * invD;
}


#if 0
#include <latl/io.hpp>
using namespace std;

int main()
{
    ArctanCameraModel cm;
    Vector<-1,float> p = cm.get_params();
    p[3] = 0.5f;
    p[4] = 1.0f;
    cm.set_params(p);
    
    const CameraModel& acm = cm;
    
    Vector<2,float> xy;
    xy[0] = 0.1;
    xy[1] = 0.4;

    const int NP = acm.num_params();
    Matrix<2,2,float> J, Jnum;
    Matrix<2,-1,float> J_model(NP), J_model_num(NP);
    Vector<2,float> uv = acm.project(xy, J, J_model);
    
    cerr << uv << endl << endl;
    cerr << J << endl;
    cerr << J_model << endl;

    const float eps = 1e-4f;
    for (int i=0; i<2; ++i) {
        Vector<2,float> z = xy;
        z[i] = xy[i] + eps;
        Vector<2,float> hi = acm.project(z);
        z[i] = xy[i] - eps;
        Vector<2,float> lo = acm.project(z);
        Jnum.T()[i] = (hi - lo)  / (2*eps);
    }

    Vector<-1,float> ref_params = cm.get_params();
    for (int i=0; i<NP; ++i) {
        Vector<-1,float> p(NP, 0.0f);
        p[i] = eps;
        cm.update_params(p);
        Vector<2,float> hi = acm.project(xy);
        cm.set_params(ref_params);
        p[i] = -eps;
        cm.update_params(p);
        Vector<2,float> lo = acm.project(xy);
        cm.set_params(ref_params);
        
        J_model_num.T()[i] = (hi-lo) / (2*eps);
    }
    
    cerr.precision(10);
    cerr << J - Jnum << endl;
    cerr << J_model - J_model_num << endl;

    Matrix<2,2,float> J_up, J_up_num;

    for (int i=0; i<2; ++i) {
        Vector<2,float> z = uv;
        z[i] = uv[i] + eps;
        Vector<2,float> hi = acm.unproject(z);
        z[i] = uv[i] - eps;
        Vector<2,float> lo = acm.unproject(z);
        J_up_num.T()[i] = (hi - lo)  / (2*eps);
    }
       
    cerr << acm.unproject(uv, J_up) - xy << endl << endl;

    cerr << inverse(Jnum) - J_up_num << endl;
    cerr << J_up- J_up_num << endl;
    
    return 0;
}

#endif
