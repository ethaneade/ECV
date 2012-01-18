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
#include <latl/latl.hpp>
#include <latl/ldlt.hpp>

#include <GL/gl.h>

namespace ecv {

    namespace gl {
        
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
        void glTexImage2D(const Image<T>& im, GLint fmt=GLPixelTraits<T>::format)
        {
            typedef GLPixelTraits<T> traits;
            ::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            ::glPixelStorei(GL_UNPACK_ROW_LENGTH, im.stride());
            ::glTexImage2D(GL_TEXTURE_2D, 0,
                           fmt,
                           im.width(), im.height(), 0,
                           fmt, traits::type,
                           im.data());
            ::glPixelStorei(GL_UNPACK_ALIGNMENT, 0);
        }

        struct GLPushMatrix {
            GLPushMatrix() { glPushMatrix(); }
            ~GLPushMatrix() { glPopMatrix(); }
        };

        void glOrthoPixels(int w, int h);
        
        class GLTexture {

        private:        
            GLuint id;
            bool valid;

            GLTexture(const GLTexture& other);
        
        public:        
            GLTexture() { valid = false; }
            ~GLTexture() {
                if (valid)
                    glDeleteTextures(1, &id);
            }

            GLuint get_id() {
                maybe_create();
                return id;
            }

            void maybe_create() {
                if (!valid) {
                    glGenTextures(1, &id);
                    valid = true;
                }
            }
        
            template <class T>
            void load(const Image<T>& im, int fmt=GLPixelTraits<T>::format) {
                bind();
                glTexImage2D(im, fmt);
            }

            void bind() {
                glEnable(GL_TEXTURE_2D);
                maybe_create();
                glBindTexture(GL_TEXTURE_2D, id);
            }

            static void set_env_mode(GLint mode) {
                glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, mode);
            }
        
            static void set_filter(GLint filter) {
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, filter);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, filter);        
            }
        };

        inline
        void glTexturedBox(double x, double y, double w, double h)
        {
            glBegin(GL_QUADS);
            glTexCoord2f(0,0);
            glVertex2d(x,y);
            glTexCoord2f(1,0);
            glVertex2d(x+w, y);
            glTexCoord2f(1,1);
            glVertex2d(x+w, y+h);
            glTexCoord2f(0,1);
            glVertex2d(x, y+h);
            glEnd();
        }


        template <class V>
        void glVertex(const latl::AbstractVector<V>& v)
        {
            assert(v.size() == 2 || v.size() == 3);
            if (v.size() == 2) {
                glVertex2d(v[0], v[1]);
            } else {
                glVertex3d(v[0], v[1], v[2]);
            }
        }

        inline
        void glLine(double x0, double y0, double x1, double y1)
        {
            glBegin(GL_LINES);
            glVertex2d(x0,y0);
            glVertex2d(x1,y1);
            glEnd();            
        }

        template <class V1, class V2>
        void glQuadLine(const latl::FixedVector<2,V1>& a, const latl::FixedVector<2,V2>& b, double width)
        {
            latl::Vector<2> d = b - a;
            d = perp(unit(d)) * 0.5*width;

            glBegin(GL_QUADS);
            glVertex(a - d);
            glVertex(b - d);
            glVertex(b + d);
            glVertex(a + d);
            glEnd();
        }

        template <class V1, class V2>
        void glLine(const latl::FixedVector<2,V1>& a, const latl::FixedVector<2,V2>& b)
        {
            glLine(a[0], a[1], b[0], b[1]);
        }
    
        inline
        void glCross(double x, double y, double radius)
        {
            glBegin(GL_LINES);
            glVertex2d(x-radius, y);
            glVertex2d(x+radius, y);
            glVertex2d(x, y-radius);
            glVertex2d(x, y+radius);
            glEnd();
        }
    
        template <class V>
        void glCross(const latl::FixedVector<2,V>& center, double radius)
        {
            glCross(center[0], center[1], radius);
        }

        inline
        void glBox(double x, double y, double w, double h)
        {
            glBegin(GL_QUADS);
            glVertex2d(x,y);
            glVertex2d(x+w, y);
            glVertex2d(x+w, y+h);
            glVertex2d(x, y+h);
            glEnd();
        }
        
        inline
        void glUnitCircle(int segs=20)
        {
            double dtheta = 2 * M_PI / segs;
            double c = cos(dtheta);
            double s = sin(dtheta);
            double x = 1;
            double y = 0;

            glBegin(GL_LINE_STRIP);
            glVertex2d(1,0);        
            for (int i=1; i<segs; ++i) {
                double x1 = x*c - y*s;
                y = x*s + y*c;
                x = x1;
                glVertex2d(x,y);
            }
            glVertex2d(1,0);
            glEnd();
        }

        inline
        void glCircle(double x, double y, double r, int segs=20)
        {
            GLPushMatrix pm;
            glTranslated(x,y,0);
            glScaled(r,r,1);
            glUnitCircle(segs);        
        }

        template <class V, class Mat>
        void glEllipse(const latl::FixedMatrix<2,2,Mat>& C, const latl::FixedVector<2,V>& x, int segs=20)
        {
            latl::LDLT<2,double> ldlt(C);
            latl::Matrix<4> L(0.0);
            ldlt.get_Cholesky(latl::slice<0,0,2,2>(L).instance());
            L(2,2) = L(3,3) = 1;

            GLPushMatrix pm;
            glTranslated(x[0], x[1], 0);
            glMultMatrixd(L.data());
            glUnitCircle();
        }
    }
}


#endif
