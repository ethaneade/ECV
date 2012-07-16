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


bool ecv::ArctanCameraModel::can_project(const latl::Vector<3,float> &xyz) const
{
    return true;
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

ecv::ArctanCTCameraModel::ArctanCTCameraModel()
{
    fill(uv0, 0.5f);
    fill(focal, 1.0f);
    radial = 1.0f;
    zero(tangential);
    zero(cr);
    update_internal();
}
        
Vector<-1,float> ecv::ArctanCTCameraModel::get_params() const
{
    Vector<-1,float> p(9);
    slice<0,2>(p) = uv0;
    slice<2,2>(p) = focal;
    p[4] = radial;
    slice<5,2>(p) = tangential;
    slice<7,2>(p) = cr;
    return p;
}

void ecv::ArctanCTCameraModel::update_internal()
{
    inv_focal[0] = 1/focal[0];
    inv_focal[1] = 1/focal[1];
    inv_radial = 1/radial;
}

void ecv::ArctanCTCameraModel::set_params(const Vector<-1,float>& p)
{
    assert(p.size() == 9);
    uv0 = slice<0,2>(p);
    focal = slice<2,2>(p);
    radial = p[4];
    tangential = slice<5,2>(p);
    cr = slice<7,2>(p);
    update_internal();
}


void ecv::ArctanCTCameraModel::update_params(const Vector<-1,float>& dp)
{
    assert(dp.size() == 9);
    uv0 += slice<0,2>(dp);
    focal += slice<2,2>(dp);
    radial *= latl::exp(dp[4]);
    tangential += slice<5,2>(dp);
    cr += slice<7,2>(dp);
    
    update_internal();
}


bool ecv::ArctanCTCameraModel::can_project(const latl::Vector<3,float> &xyz) const
{
    return true;
}

Vector<2,float> ecv::ArctanCTCameraModel::project(const Vector<2,float>& xy,
                                                Matrix<2,2,float>* J_xy,
                                                Matrix<2,-1,float>* J_model) const
{
    const Vector<2,float> d = xy - cr;
    float R = norm_sq(d);
    float Asq = R * inv_radial * inv_radial;
    float c, dD;
    float A_factor;
    if (Asq < Constants<float>::sqrt_epsilon()) {
        c = 1.f - Asq*(third - Asq*fifth);
        dD = 0.f;
        A_factor = 1.f;
    } else {            
        float r = latl::sqrt(R);
        float A = r * inv_radial;
        float inv_r = 1.f/r;
        float inv_A = radial * inv_r;
        float inv_R = inv_r * inv_r;
        A_factor = 1.f/(1.f+Asq);
        c = latl::atan(A) * inv_A;
        dD = (A_factor - c) * inv_R;
    }

    float D = c + 2 * (d * tangential);
    Vector<2,float> e = D * d + R * tangential + cr;
    const Vector<2,float> uv = diagmult(focal, e) + uv0;

    Matrix<2,2,float> Jxy;
    if (J_xy || J_model) {
        float offdiag = 2 * (d[0] * tangential[1] + d[1] * tangential[0]) + dD * d[0] * d[1];
        Jxy(0,0) = focal[0] * (D + d[0] * (4 * tangential[0] + dD * d[0]));
        Jxy(0,1) = focal[0] * offdiag;
        Jxy(1,0) = focal[1] * offdiag;        
        Jxy(1,1) = focal[1] * (D + d[1] * (4 * tangential[1] + dD * d[1]));

        if (J_xy)
            *J_xy = Jxy;
    }
    

    if (J_model) {
        Matrix<2,-1,float> &J_cm = *J_model;
        zero(J_cm);
        const Vector<2,float> f_d = diagmult(focal, d);
        slice<0,7,2,2>(J_cm) = -Jxy;
        J_cm(0,7) += focal[0];
        J_cm(1,8) += focal[1];
        
        J_cm(0,0) = J_cm(1,1) = 1.f;
        J_cm(0,2) = e[0];
        J_cm(1,3) = e[1];
    
        J_cm.T()[4] = (c - A_factor) * f_d;
        
        float off_diag = 2*d[0]*d[1];
        J_cm(0,5) = focal[0] * (2*d[0]*d[0] + R);
        J_cm(0,6) = focal[0] * off_diag;
        J_cm(1,5) = focal[1] * off_diag;
        J_cm(1,6) = focal[1] * (2*d[1]*d[1] + R);
    }
    return uv;
}
        
Vector<2,float> ecv::ArctanCTCameraModel::unproject(const Vector<2,float>& uv,
                                                  Matrix<2,2,float>* J_uv) const
{    
    Vector<2,float> e = diagmult(inv_focal, uv - uv0) - cr;
    
    // Guess for tangential distortion
    Vector<2,float> d = e - norm_sq(e) * tangential;

    float r = latl::sqrt(norm_sq(d));
    float A = radial * r;
    float invD;
    if (A < Constants<float>::sqrt_epsilon()) {
        float Asq = A*A;
        invD = 1 + Asq*third*(1 - Asq*2*fifth);
    } else {
        float invA = 1 / A;
        invD = latl::tan(A) * invA;
    }

    Vector<2,float> xy = d*invD + cr;
    Matrix<2,2,float> J;
    for (int i=0; i<4; ++i) {
        Vector<2,float> v = uv - CameraModel::project(xy, J);
        J = inverse(J);
        xy += J * v;
    }

    if (J_uv) {
        *J_uv = J;
    }
    return xy;
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
