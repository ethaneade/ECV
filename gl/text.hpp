#pragma once

namespace ecv {
    namespace gl {

        void glTextBounds(const char *text, int xywh[4]);
        
        void glText(double x, double y, const char *text,
                    double xywh[4]=0);
        
    }
}
