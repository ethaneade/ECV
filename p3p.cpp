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

#include <ecv/p3p.hpp>
#include <mathbits/quartic.h>

using namespace latl;

namespace ecv {

    // Find the rigid transformation that maps x to y,
    // assuming that such a transformation exists.
    
    template <class Scalar>
    void absolute_pose(const Vector<3,Scalar> x[3],
                       const Vector<3,Scalar> y[3],
                       SE3<Scalar>& pose)
    {
        Matrix<3,3,Scalar> X, Y;

        Vector<3,Scalar> d0 = unit(x[1] - x[0]);
        Vector<3,Scalar> d2 = unit(d0 ^ (x[2] - x[0]));
        X[0] = d0;
        X[1] = d2 ^ d0;
        X[2] = d2;

        Vector<3,Scalar> e0 = unit(y[1] - y[0]);
        Vector<3,Scalar> e2 = unit(e0 ^ (y[2] - y[0]));
        Y.T()[0] = e0;
        Y.T()[1] = e2 ^ e0;
        Y.T()[2] = e2;

        pose.rotation() = SO3<Scalar>::bless(Y * X);
    
        static const double third = 1.0/3;
        Vector<3,Scalar> sumx = x[0] + x[1] + x[2];
        Vector<3,Scalar> sumy = y[0] + y[1] + y[2];
        pose.translation() = third * (sumy - pose.rotation() * sumx);
    }

    // Three point pose problem, a la Fischer and Bolles, 1981.
    
    int p3p(const Vector<3> x[3], const Vector<2> z[3],
            SE3<> poses[])
    {
        double L01 = norm_sq(x[1] - x[0]);
        double L02 = norm_sq(x[2] - x[0]);
        double L12 = norm_sq(x[2] - x[1]);

        Vector<3> unit_z[3];
        for (int i=0; i<3; ++i)
            unit_z[i] = unit(unproject(z[i]));

        // Compute cosines of inter-ray angles
        double c01 = unit_z[0] * unit_z[1];
        double c02 = unit_z[0] * unit_z[2];
        double c12 = unit_z[1] * unit_z[2];

        // Roughly cos(0.5 degrees)
        const double cos_min_theta = 0.99996;

        // Check for collinear points
        if (c01 > cos_min_theta ||
            c02 > cos_min_theta ||
            c12 > cos_min_theta)
            return -1;

        // Construct the quartic coefficients
        double K1 = L12 / L02;
        double K2 = L12 / L01;

        double K12 = K1*K2;
        double K12m1m2 = K12 - K1 - K2;
        double K12p1m2 = K12 + K1 - K2;
        double K12m1p2 = K12 - K1 + K2;
    
        double G4 = sq(K12m1m2) - 4*K12*sq(c12);
    
        double G3 = 4*(K12m1m2*K2*(1 - K1)*c01
                       + K1*c12*(K12m1p2*c02
                                 + 2*K2*c01*c12
                                 )
                       );
    
        double G2 = (sq(2*K2*(1 - K1)*c01)
                     + 2*K12p1m2*K12m1m2
                     + 4*K1*((K1 - K2)*sq(c12)
                             + (1 - K2)*K1*sq(c02)
                             - 2*K2*(1 + K1)*c01*c02*c12
                             )
                     );
    
        double G1 = 4*(K12p1m2*K2*(1 - K1)*c01
                       + K1*(K12m1p2 *c02*c12
                             + 2*K12*c01*sq(c02)
                             )
                       );
    
        double G0 = sq(K12p1m2) - 4*sq(K1*c02)*K2;
    
    
        double invG4 = 1/G4;
        double roots[4];
        int nr = real_quartic_roots(invG4*G3, invG4*G2, invG4*G1, invG4*G0, roots);

        int np = 0;
        for (int i=0; i<nr; ++i) {
            double r = roots[i];
            if (r <= 0)
                continue;

            double a, b, c;
            double a_sq = L01 / (r*(r - 2*c01) + 1);
            if (a_sq <= 0)
                continue;
        
            a = sqrt(a_sq);
            b = a*r;

            double r2 = r*r;
            double q = r2 - K1;
            double q1 = r2*(1-K2) + 2*K2*r*c01 - K2;
            double den = (1-K1)*q1 - q;
            double num = 2*(-r*c12*q - (K1*c02 - r*c12)*q1);

            if (num*den < 0 || num == 0)
                continue;

            if (fabs(den) <= 1e-12 * fabs(num)) {
                // Special case:
                // Fraction num/den is degenerate.
                // Solve for y = c/a directly.
            
                double a2 = a*a;
                double b2 = b*b;
                double disc = sq(c02) + (L02 - a2) / a2;
                if (disc < 0)
                    continue;
                double rd = sqrt(disc);
                double c1 = (c02 + rd) * a;
                double c2 = (c02 - rd) * a;
                double err1 = sq(b2 - L12 + c1*(c1 - 2*b*c12));
                double err2 = sq(b2 - L12 + c2*(c2 - 2*b*c12));
                double err;
                if (c1 < 0) {
                    if (c2 < 0)
                        continue;
                    c = c2;
                    err = err2;
                } else if (c2 < 0 || err1 < err2) {
                    c = c1;
                    err = err1;
                } else {
                    c = c2;
                    err = err2;
                }
                if (err > 1e-12)
                    continue;
            } else {
                double s = num / den;
                c = a*s;
            }
        
            Vector<3> p[3] = { a*unit_z[0],
                               b*unit_z[1],
                               c*unit_z[2] };
        
            absolute_pose(x, p, poses[np++]);
        }

        return np;
    }

}

#if 0

// Some silly test code

#include <latl/se3_io.hpp>
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace ecv;

int main()
{
    SE3<> pose = SE3<>::exp(Vector<6>(CenteredRandom()));

    cerr << pose << endl;
    
    Vector<3> x[3];
    Vector<2> z[3];
    Vector<3> unit_z[3];
    double scale = 0.5;
    
    for (int i=0; i<3; ++i) {
        x[i] = Vector<3>(CenteredRandom(scale));
        x[i][2] += 1;
        z[i] = project(pose * x[i]);
        unit_z[i] = unit(unproject(z[i]));
    }

    SE3<> poses[4];
    int np = p3p(x, z, poses);

    if (np < 0) {
        cerr << "Error" << endl;
        return 1;
    }
    
    for (int i=0; i<np; ++i) {
        cerr << poses[i] << endl;
        for (int j=0; j<3; ++j) {
            cerr << norm(z[j] - project(poses[i] * x[j])) << endl;
        }
    }
    return 0;
}

#endif
