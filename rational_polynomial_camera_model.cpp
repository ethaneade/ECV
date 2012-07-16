#include <ecv/rational_polynomial_camera_model.hpp>

using namespace latl;
using namespace ecv;

ecv::RationalPolynomialCameraModel::RationalPolynomialCameraModel()
{
    fill(uv0, 0.5f);
    fill(focal, 1.0f);
    pnum[0] = 1.f;
    pnum[1] = 0.f;
    pnum[2] = 0.f;;
    pden[0] = 0.f;
    pden[1] = 0.f;
    cr[0] = 0.f;
    cr[1] = 0.f;
    update_internal();
}

void ecv::RationalPolynomialCameraModel::update_internal()
{
    inv_focal[0] = 1/focal[0];
    inv_focal[1] = 1/focal[1];
}

Vector<-1,float> ecv::RationalPolynomialCameraModel::get_params() const
{
    Vector<-1,float> p(NUM_PARAMS);
    slice<0,2>(p) = uv0;
    slice<2,2>(p) = focal;
    slice<4,3>(p) = pnum;
    slice<7,2>(p) = pden;
    slice<9,2>(p) = cr;
    return p;
}

void ecv::RationalPolynomialCameraModel::set_params(const Vector<-1,float>& p)
{
    assert(p.size() == NUM_PARAMS);
    uv0 = slice<0,2>(p);
    focal = slice<2,2>(p);
    pnum = slice<4,3>(p);
    pden = slice<7,2>(p);
    cr = slice<9,2>(p);
    update_internal();
}


void ecv::RationalPolynomialCameraModel::update_params(const Vector<-1,float>& dp)
{
    assert(dp.size() == NUM_PARAMS);
    uv0 += slice<0,2>(dp);
    focal += slice<2,2>(dp);
    pnum += slice<4,3>(dp);
    pden += slice<7,2>(dp);
    cr += slice<9,2>(dp);
    update_internal();
}


bool ecv::RationalPolynomialCameraModel::can_project(const latl::Vector<3,float> &xyz) const
{
    return true;
}

static const float third = 1.0f/3;
static const float fifth = 0.2f;    

Vector<2,float> ecv::RationalPolynomialCameraModel::project(const Vector<2,float>& xy,
                                                Matrix<2,2,float>* J_xy,
                                                Matrix<2,-1,float>* J_model) const
{
    Vector<2,float> d = xy - cr;
    const float R = norm_sq(d);
    
    float num = pnum[0] + R*(pnum[1] + R*pnum[2]);
    float den = 1.f + R*(pden[0] + R*pden[1]);
    
    float inv_den = 1.f / den;
    float D = num * inv_den;
    
    Vector<2,float> e = D * d + cr;
    Vector<2,float> uv = diagmult(focal, e) + uv0;

    float dnum_dR = pnum[1] + 2.f*R*pnum[2];
    float dden_dR = pden[0] + 2.f*R*pden[1];
    float dD_dR = inv_den * (dnum_dR - D*dden_dR);
    float dR_dx = 2.f*d[0];
    float dR_dy = 2.f*d[1];    
    
    if (J_xy) {
        float dx = d[0]*dD_dR;
        float dy = d[1]*dD_dR;
        (*J_xy)(0,0) = focal[0] * (D + dx*dR_dx);
        (*J_xy)(0,1) = focal[0] * (dx*dR_dy);
        (*J_xy)(1,0) = focal[1] * (dy*dR_dx);
        (*J_xy)(1,1) = focal[1] * (D + dy*dR_dy);
    }

    if (J_model) {
        float dua = focal[0] * inv_den * d[0];
        float dva = focal[1] * inv_den * d[1];
        float dub = focal[0] * -D*inv_den * d[0];
        float dvb = focal[1] * -D*inv_den * d[1];
        float R2 = R*R;
        (*J_model)(0,0) = (*J_model)(1,1) = 1.f;
        (*J_model)(0,1) = (*J_model)(1,0) = 0.f;
        
        (*J_model)(0,2) = e[0];
        (*J_model)(1,2) = 0.f;
        (*J_model)(0,3) = 0.f;
        (*J_model)(1,3) = e[1];

        (*J_model)(0,4) = dua;
        (*J_model)(0,5) = dua * R;
        (*J_model)(0,6) = dua * R2;
        (*J_model)(0,7) = dub * R;
        (*J_model)(0,8) = dub * R2;
        (*J_model)(1,4) = dva;
        (*J_model)(1,5) = dva * R;
        (*J_model)(1,6) = dva * R2;
        (*J_model)(1,7) = dvb * R;
        (*J_model)(1,8) = dvb * R2;

        float dx = d[0]*dD_dR;
        float dy = d[1]*dD_dR;
        (*J_model)(0,9) = focal[0]*(1.f - (D + dx*dR_dx));
        (*J_model)(0,10) = -focal[0]*(dx*dR_dy);
        (*J_model)(1,9) = -focal[1]*(dy*dR_dx);
        (*J_model)(1,10) = focal[1]*(1.f - (D + dy*dR_dy));        
    }

    return uv;
}
        
Vector<2,float> ecv::RationalPolynomialCameraModel::unproject(const Vector<2,float>& uv,
                                                              Matrix<2,2,float>* J_uv) const
{
    Vector<2,float> d = diagmult(inv_focal, uv - uv0) - cr;
    float R = d*d;

    float y = 1;
    for (int pass=0; pass<6; ++pass) {
        float fy = 1.f - y*(pnum[0] - y*R*(pden[0] - y*(pnum[1] - y*R*(pden[1] - y*pnum[2]))));
        float dy = -pnum[0] + R*y*(2.f*pden[0] - y*(3.f*pnum[1] - y*R*(4.f*pden[1] - 5.f*y*pnum[2])));
        y -= fy/dy;
    }

    Vector<2,float> xy = d*y + cr;
    Matrix<2,2,float> J;
    
    for (int pass=0; pass<4; ++pass) {
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
    RationalPolynomialCameraModel cm;
    Vector<-1,float> p = cm.get_params();
    p[4] = 1.0f;
    p[5] = -0.1f;
    p[7] = 0.1f;
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
    cerr << acm.unproject(uv) << endl<<endl;
    cerr << J << endl;
    cerr << J_model << endl;

    const float eps = 1e-2f;
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
