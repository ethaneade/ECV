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

#ifndef ECV_GL_HELPERS_HPP
#define ECV_GL_HELPERS_HPP

#include <ecv/image.hpp>
#include <ecv/rgb.hpp>
#include <GL/gl.h>

namespace ecv {

    template <class T>
    struct GLPixelTraits;

    template <>
    struct GLPixelTraits<uint8_t> {
        enum { format = GL_LUMINANCE,
               type = GL_UNSIGNED_BYTE };
    };

    template <>
    struct GLPixelTraits<int16_t> {
        enum { format = GL_LUMINANCE,
               type = GL_SHORT };
    };

    template <>
    struct GLPixelTraits<uint16_t> {
        enum { format = GL_LUMINANCE,
               type = GL_UNSIGNED_SHORT };
    };

    template <>
    struct GLPixelTraits<float> {
        enum { format = GL_LUMINANCE,
               type = GL_FLOAT };
    };

    template <>
    struct GLPixelTraits<Rgb8> {
        enum { format = GL_RGB,
               type = GL_UNSIGNED_BYTE };
    };

    template <class T>
    void glDrawPixels(const Image<T>& im)
    {
        typedef GLPixelTraits<T> traits;
        ::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        ::glPixelStorei(GL_UNPACK_ROW_LENGTH, im.stride());
        ::glDrawPixels(im.width(), im.height(), traits::format, traits::type, im.data());
        ::glPixelStorei(GL_UNPACK_ALIGNMENT, 0);
    }

    template <class T>
    void glTexImage2D(const Image<T>& im)
    {
        typedef GLPixelTraits<T> traits;
        ::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        ::glPixelStorei(GL_UNPACK_ROW_LENGTH, im.stride());
        ::glTexImage2D(GL_TEXTURE_2D, 0,
                       traits::format,
                       im.width(), im.height(), 0,
                       traits::format, traits::type,
                       im.data());
        ::glPixelStorei(GL_UNPACK_ALIGNMENT, 0);
    }
    
    
}

#endif
