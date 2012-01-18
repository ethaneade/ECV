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
#ifndef ECV_GL_WINDOW_HPP
#define ECV_GL_WINDOW_HPP

namespace ecv {
    
    namespace gl {
        
        class Window
        {
        public:

            struct Event {
                enum Buttons { BTN_L=1, BTN_M=2, BTN_R=4, BTN_W=8 };
                enum Type { KEY_DOWN, KEY_UP, BTN_DOWN, BTN_UP, MOVE, RESIZE };
            };

            struct EventHandler {
                virtual ~EventHandler() {}
                virtual bool on_key_down(Window& win, int key) { return false; }
                virtual bool on_key_up(Window& win, int key) { return false; }
                virtual bool on_button_down(Window& win, int btn, int state, int x, int y) { return false; }
                virtual bool on_button_up(Window& win, int btn, int state, int x, int y) { return false; }
                virtual bool on_mouse_move(Window& win, int state, int x, int y) { return false; }
                virtual bool on_mouse_wheel(Window& win, int state, int x, int y, int dx, int dy) { return false; }
                virtual bool on_resize(Window& win, int x, int y, int w, int h) { return false; }
            };


            Window(int w, int h, const char* title);
            ~Window();
            
            int width() const;
            int height() const;
            bool visible() const;
            bool alive() const;
            
            void make_current();
            void swap_buffers();
            void resize(int w, int h);
            void set_title(const char* title);
            void set_handler(EventHandler* handler);

            void set_ortho(double lx, double ly, double hx, double hy);
            void set_ortho() { set_ortho(0,0, width(), height()); }

            static void update(int ms=0);            
            
        protected:
            struct State;
            State *_state;
        };
    }
}

#endif
