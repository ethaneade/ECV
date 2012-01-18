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

#ifndef ECV_IO_PNM_HPP
#define ECV_IO_PNM_HPP

#include <iostream>
#include <ctype.h>

namespace ecv {

    namespace pnm_io {

        void skip_space(std::istream& in)
        {            
            while (in.good() && isspace(in.get()));
            in.unget();            
        }

        bool skip_comment(std::istream& in)
        {
            char c = in.get();
            if (c != '#') {
                in.unget();
                return false;
            }
            while (in.good() && in.get() != '\n');
            return true;
        }
        
        bool read_num(std::istream& in, int &n)
        {
            char c = in.get();
            if (!isdigit(c))
                return false;
            n = c - '0';
            c = in.get();
            
            while (in.good() && isdigit(c)) {
                n = n*10 + (c - '0');
                c = in.get();
            }
            in.unget();
            return true;
        }

        struct Header {
            int fmt;
            int width, height, maxval;
        };

        bool read_header(std::istream& in,
                         Header &header)
        {
            char P = in.get();
            if (P != 'P') {
                in.unget();
                return false;
            }
        
            char fmt = in.get();
            switch (fmt) {
            case '2':
            case '3':
            case '5':
            case '6':
                break;
            default:
                in.unget();
                return false;
            }

            skip_space(in);
            
            header.fmt = fmt;
        
            int num[3];

            for (int i=0; i<3; ++i) {
                while (in.good() && skip_comment(in)) {}            
                skip_space(in);
                if (!read_num(in, num[i]))
                    return false;
            }
            header.width = num[0];
            header.height = num[1];
            header.maxval = num[2];

            if (!isspace(in.get()))
                return false;
            return true;
        }
    }
}

#endif
