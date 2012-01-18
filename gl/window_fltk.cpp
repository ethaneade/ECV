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
#include <ecv/gl/window.hpp>

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/gl.h>
#include <FL/Fl_Gl_Window.H>
#include <ecv/gl/helpers.hpp>

#include <iostream>

namespace ecv {
    
    int from_fltk_button(int btn)
    {
        switch (btn) {
        case 1: return gl::Window::Event::BTN_L;
        case 2: return gl::Window::Event::BTN_M;
        case 3: return gl::Window::Event::BTN_R;
        }
        return 0;
    }

    int from_fltk_buttons(int btns)
    {
        int out=0;
        if (btns & FL_BUTTON1) out |= gl::Window::Event::BTN_L;
        if (btns & FL_BUTTON2) out |= gl::Window::Event::BTN_M;
        if (btns & FL_BUTTON3) out |= gl::Window::Event::BTN_R;
        return out;
    }
    
    class GLWindow : public Fl_Gl_Window {
    protected:
        void draw() {
        }

        int handle(int event) {
            if (event == FL_FOCUS)
                return 1;

            if (event == FL_ENTER)
                return 1;
            
            if (handler) {
                int x = Fl::event_x();
                int y = Fl::event_y();
                int dx = Fl::event_dx();
                int dy = Fl::event_dy();

                int btn = from_fltk_button(Fl::event_button());
                int state = from_fltk_buttons(Fl::event_buttons());
                
                switch (event) {
                case FL_PUSH:
                    if (handler->on_button_down(*owner, btn, state, x, y))
                        return 1;
                    break;
                    
                case FL_RELEASE:
                    if (handler->on_button_up(*owner, btn, state, x, y))
                        return 1;
                    break;
                    
                case FL_MOVE:
                    if (handler->on_mouse_move(*owner, state, x, y))
                        return 1;
                    break;
                    
                case FL_MOUSEWHEEL:
                    if (handler->on_mouse_wheel(*owner, state, x, y, dx, dy))
                        return 1;
                    break;
                    
                case FL_KEYDOWN:                  
                    if (handler->on_key_down(*owner, Fl::event_key()))
                        return 1;
                    break;
                    
                case FL_KEYUP:
                    if (handler->on_key_up(*owner, Fl::event_key()))
                        return 1;
                    break;
                    
                default:
                    break;
                }
            }
            return Fl_Gl_Window::handle(event);
        }
        
    public:
        gl::Window *owner;
        gl::Window::EventHandler *handler;

        void resize(int X, int Y, int W, int H)
        {
            Fl_Gl_Window::resize(X,Y,W,H);
            if (handler) {
                handler->on_resize(*owner, X, Y, W, H);
            }
        }
        
        GLWindow(int X, int Y, int W, int H, const char *L=0)
            : Fl_Gl_Window(X, Y, W, H, L)
        {
            mode(FL_DOUBLE | FL_ALPHA | FL_DEPTH | FL_RGB8);
            handler = 0;
        }

        void setup() {
            make_current();
            gl::glOrthoPixels(w(), h());
            
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_LINE_SMOOTH);
            glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);            
        }
        
    };


    namespace gl {

        struct Window::State {
            State(int w, int h, const char *title, Window& win)
                : window(0,0,w,h, title)
            {
                glwin = new GLWindow(0,0, w,h);
                glwin->owner = &win;
                window.add(glwin);
                window.resizable(glwin);
                window.show();

                glwin->setup();
            }

            Fl_Window window;
            GLWindow *glwin;
        };
        
        Window::Window(int width, int height, const char *title)
        {
            _state = new State(width, height, title, *this);
        }
        
        Window::~Window()
        {
            delete _state;
        }
            
        int Window::width() const
        {
            return _state->window.w();
        }
        
        int Window::height() const
        {
            return _state->window.h();
        }

        bool Window::visible() const
        {
            return _state->window.visible();
        }
        
        bool Window::alive() const
        {
            return _state->window.shown();
        }

        void Window::make_current()
        {
            _state->glwin->make_current();
        }

        void Window::set_ortho(double lx, double ly, double hx, double hy)
        {
            make_current();
            glViewport(0,0,_state->glwin->w(),_state->glwin->h());

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(lx, hx, hy, ly, -1, 1);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glPixelZoom(1,-1);            
        }
        
        void Window::swap_buffers()
        {
            _state->glwin->swap_buffers();
            _state->window.redraw();
        }
        
        void Window::update(int ms)
        {
            Fl::wait(ms * 1e-3);
        }
        
        void Window::resize(int w, int h)
        {
            _state->window.resize(_state->window.x(), _state->window.y(), w,h);
        }

        
        void Window::set_title(const char* title)
        {
            _state->window.copy_label(title);
        }

        void Window::set_handler(EventHandler* handler)
        {
            _state->glwin->handler = handler;
        }

    }
}
