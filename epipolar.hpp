#ifndef ECV_EPIPOLAR_HPP
#define ECV_EPIPOLAR_HPP

#include <latl/se3.hpp>
#include <ecv/correspondence.hpp>
#include <latl/ldlt.hpp>

namespace ecv {

    template <class S>
    S epipolar_error(const latl::Vector<2,S>& a,
                     const latl::Vector<2,S>& b,
                     const latl::SE3<S>& motion);
    
    template <class S>
    S epipolar_error(const latl::Vector<2,S>& a,
                     const latl::Vector<2,S>& b,
                     const latl::SE3<S>& motion,
                     latl::Vector<2,S> &Ja,
                     latl::Vector<2,S> &Jb,
                     latl::Vector<6,S> &Jmotion);
    

    struct EpipolarTest
    {
        float r;
        EpipolarTest(float radius) : r(radius) {}
        template <class Scalar>
        bool operator()(const latl::SE3<Scalar>& motion, const ScaledMatch2D<Scalar>& m) const
        {
            using namespace latl;
            Vector<3,Scalar> line = motion.translation() ^ (motion.rotation() * unproject(m.a));
            Scalar den_sq = line[0]*line[0] + line[1]*line[1];
            return (unproject(m.b) * line) < latl::sq(r*m.scale) * den_sq;
        }
    };


    template <class Scalar>
    bool refine_epipolar(const std::vector<ScaledMatch2D<Scalar> >& corr,
                         const std::vector<bool>& inlier,
                         latl::SE3<Scalar>& trans)
    {
        using namespace latl;

        normalize(trans.translation());
                
        Matrix<6,6,Scalar> A(Scalar(0));
        Vector<6,Scalar> b(Scalar(0));

        outer_product_upper(trans.translation(), Scalar(1e2),
                            ops::Add(), slice<0,0,3,3>(A).ref());
        
        for (size_t i=0; i<corr.size(); ++i)
        {
            if (!inlier[i])
                continue;

            Vector<2,Scalar> Ja, Jb;
            Vector<6,Scalar> Jtrans;
            Scalar v = epipolar_error(corr[i].a, corr[i].b, trans, Ja, Jb, Jtrans);
            Scalar Rinv = 1 / sq(corr[i].scale);

            outer_product_upper(Jtrans, Rinv, ops::Add(), A);
            b -= Jtrans * (Rinv*v);
        }
                
        LDLT<6,Scalar> ldlt(A);
        if (!ldlt.is_full_rank())
            return false;
        
        Vector<6,Scalar> update = ldlt.inverse_times(b);
        trans = SE3<Scalar>::exp(update) * trans;
        normalize(trans.translation());
        
        return true;
    }
    
    
}

#endif
