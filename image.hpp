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

#ifndef ECV_IMAGE_HPP
#define ECV_IMAGE_HPP

#include <cassert>

namespace ecv {

    struct ImageSize {
        int x, y;
        ImageSize() : x(0), y(0) {}
        ImageSize(int w, int h) : x(w), y(h) {}
        
        ImageSize operator+(const ImageSize& s) const {
            return ImageSize(x + s.x, y + s.y);
        }
        
        ImageSize operator+(int s) const {
            return ImageSize(x + s, y + s);
        }

        ImageSize operator-(int s) const {
            return ImageSize(x - s, y - s);
        }
        
        bool operator==(const ImageSize& s) const {
            return x == s.x && y == s.y;            
        }

        int area() const { return x*y; }
    };
    
    template <class T>
    class Image {
    private:

        struct Buffer {
            char *data;
            int ref_count;
        };
        
        struct Block {
            T *data;
            Buffer *buffer;
            int stride;

            Block() {
                data = 0;
                buffer = 0;
                stride = 0;
            }
           
            void release() {
                if (buffer && --buffer->ref_count == 0) {
                    delete[] buffer->data;
                    delete buffer;
                    buffer = 0;
                }
            }
        };

        Block _block;
        ImageSize _size;

        static void alloc(int w, int h, unsigned short align,
                          Block& block)
        {
            int a = sizeof(T);
            int b = align;
            while (b) {
                int r = a % b;
                if (r == 0)
                    break;
                a = b;
                b = r;
            }
            int chunk = a / sizeof(T);
            int chunks = (w + chunk-1) / chunk;
            block.stride = chunks * chunk;
            block.buffer = new Buffer;
            block.buffer->data = new char[sizeof(T)*block.stride*h + align-1];
            block.buffer->ref_count = 1;
            int start = block.buffer->data - (char*)0;
            int over = start % align;
            int offset = over ? align - over : 0;
            block.data = (T*)(block.buffer->data + offset);
        }

    public:
        Image() {}
        
        Image(int w, int h, int align=16)
        {
            assert(w>=0 && h>=0);
            
            alloc(w, h, align, _block);
            _size = ImageSize(w,h);
        }

        Image(const ImageSize& size_, int align=16)
        {
            assert(size_.x >=0 && size_.y >= 0);
            _size = size_;
            alloc(_size.x, _size.y, align, _block);
        }
        

        Image(const T *data, int w, int h, int stride, int offset=-1)
        {
            assert(w >= 0 && h >= 0 && stride >= w);
            _block.data = const_cast<T*>(data);
            _block.stride = stride;
            if (offset >= 0) {
                _block.buffer = new Buffer();
                _block.buffer->data = (char*)data - offset;
                _block.buffer->ref_count = 1;
            } else {
                _block.buffer = 0;
            }
            _size = ImageSize(w,h);
        }

        Image(const Image& other)
        {
            *this = other;
        }
        
        ~Image()
        {
            _block.release();
        }

        Image& operator=(const Image& other)
        {
            _block.release();
            _block = other._block;
            if (_block.buffer)
                ++_block.buffer->ref_count;
            _size = other._size;
            return *this;
        }

        void resize(int w, int h, int align=16) {
            assert(w >= 0 && h >= 0);
            if (w == width() && h == height())
                return;
            
            _block.release();
            alloc(w, h, align, _block);
            _size = ImageSize(w,h);
        }

        void resize(const ImageSize& s, int align=16) {
            resize(s.x, s.y, align);
        }
        
        const ImageSize& size() const { return _size; }

        const T* data() const { return _block.data; }
        T* data() { return _block.data; }
        
        int width() const { return _size.x; }
        int height() const { return _size.y; }
        int stride() const { return _block.stride; }

        const T* operator[](int i) const { return _block.data + stride() * i; }
        T* operator[](int i) { return _block.data + stride() * i; }

        T operator()(int i, int j) const { return _block.data[stride()*i + j]; }
        T& operator()(int i, int j) { return _block.data[stride()*i + j]; }

        void fill(T val) {
            for (int i=0; i<height(); ++i) {
                T *row = (*this)[i];
                for (int j=0; j<width(); ++j)
                    row[j] = val;
            }                
        }
        
        void copy_data(const Image& im) {
            assert(size() == im.size());
            for (int i=0; i<height(); ++i) {
                T* a = (*this)[i];
                const T* b = im[i];
                for (int j=0; j<width(); ++j)
                    a[j] = b[j];
            }
        }

        void swap(Image& im) {
            {
                Block tmp = _block;
                _block = im._block;
                im._block = tmp;
            }
            {
                ImageSize tmp = _size;
                _size = im._size;
                im._size = tmp;
            }
        }
        
        Image copy() const
        {
            Image tmp(size());
            tmp.copy_data(*this);
            return tmp;
        }

        bool is_unique() const {
            return _block.buffer && _block.buffer->ref_count == 1;
        }
        
        void make_unique() {
            if (is_unique())
                return;
            
            Image<T> im = copy();
            swap(im);
        }

        Image sub_image(int x, int y, int w, int h) const
        {
            assert(x >= 0 && y >= 0 &&
                   w >= 0 && h >= 0 &&
                   x < width() && y < height() &&
                   x+w <= width() && y+h <= height());
            
            Image im(*this);            
            im._block.data = const_cast<T*>((*this)[y] + x);
            im._size = ImageSize(w,h);
            return im;
        }
        
    };
    
}

#endif
