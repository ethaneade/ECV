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

#ifndef ECV_P3P_HPP
#define ECV_P3P_HPP

#include <latl/se3.hpp>

namespace ecv {

    // Solve the perspective three point pose problem.
    // Up to four poses are stored in poses[].
    // The number of poses is returned, or -1 is returned
    // in the case of error (such as when collinear
    // or nearly collinear points are specified in z[]).

    // Each computed pose C satisfies, for i=0,1,2:
    //       project(C*x_i) == z_i
    // and 
    //       (C*x_i) * (0 0 1)' > 0
    // (points are in front of the camera).
    
    int p3p(const latl::Vector<3> x[3], const latl::Vector<2> z[3],
            latl::SE3<double> poses[]);
}

#endif
