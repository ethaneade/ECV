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
