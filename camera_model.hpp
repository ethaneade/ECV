#ifndef ECV_CAMERA_MODEL_HPP
#define ECV_CAMERA_MODEL_HPP

#include <latl/latl.hpp>

namespace ecv {

    class CameraModel
    {
    public:
        virtual ~CameraModel() {}

        virtual const char* name() const = 0;
        
        virtual int num_params() const = 0;
        virtual latl::Vector<-1,float> get_params() const = 0;
        virtual void set_params(const latl::Vector<-1,float>& p) = 0;
        virtual void update_params(const latl::Vector<-1,float>& dp) = 0;
        
        latl::Vector<2,float> project(const latl::Vector<2,float>& xy) const
        {
            return project(xy, 0, 0);
        }

        latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                      latl::Matrix<2,2,float>& J_xy) const
        {
            return project(xy, &J_xy, 0);
        }

        latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                      latl::Matrix<2,2,float>& J_xy,
                                      latl::Matrix<2,-1,float>& J_model) const
        {
            return project(xy, &J_xy, &J_model);
        }

        latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv) const
        {
            return unproject(uv, 0);
        }

        latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv,
                                        latl::Matrix<2,2,float>& J_uv) const
        {
            return unproject(uv, &J_uv);
        }
        
    protected:
        virtual latl::Vector<2,float> project(const latl::Vector<2,float>& xy,
                                              latl::Matrix<2,2,float>* J_xy,
                                              latl::Matrix<2,-1,float>* J_model) const = 0;

        virtual latl::Vector<2,float> unproject(const latl::Vector<2,float>& uv,
                                                latl::Matrix<2,2,float>* J_uv) const = 0;
    };
    
}

#endif
