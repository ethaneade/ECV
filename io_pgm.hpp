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

#ifndef ECV_IO_PGM_HPP
#define ECV_IO_PGM_HPP

#include <ecv/image.hpp>

#include <stdint.h>
#include <iosfwd>

namespace ecv {
    
    bool pgm_read(std::istream& in, Image<uint8_t>& im);
    bool pgm_read(std::istream& in, Image<uint16_t>& im);
    
    bool pgm_load(const char *filename, Image<uint8_t>& im);
    bool pgm_load(const char *filename, Image<uint16_t>& im);

    void pgm_write(const Image<uint8_t>& im, std::ostream& out);
    void pgm_write(const Image<uint16_t>& im, std::ostream& out);

    bool pgm_save(const Image<uint8_t>& im, const char *filename);
    bool pgm_save(const Image<uint16_t>& im, const char *filename);
}

#endif
