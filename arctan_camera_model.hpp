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
