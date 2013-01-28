#include <ecv/gl/text.hpp>
#include <ecv/gl/helpers.hpp>
#include <GL/gl.h>
#include <vector>
#include "font.inc"
#include <algorithm>

static GLuint tex_id = (unsigned int)-1;

static void init_font()
{
    if (tex_id != (unsigned int)-1)
        return;
    
    int w = font_img_width;
    int h = font_img_height;

    std::vector<unsigned char> data(w*h*2);
    unpack_rle(font_luminance_rle, font_alpha_rle, w*h, &data[0]);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &tex_id);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    ::glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    ::glPixelStorei(GL_UNPACK_ROW_LENGTH, w);
    ::glTexImage2D(GL_TEXTURE_2D, 0,
                   GL_LUMINANCE_ALPHA,
                   w, h, 0,
                   GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE,
                   &data[0]);
    ::glPixelStorei(GL_UNPACK_ALIGNMENT, 0);

    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);        
}

void ecv::gl::glTextBounds(const char *text, int xywh[4])
{
    int min_y = 1000000;
    int max_y = -1000000;
    int min_x = 0;
    int x = 0, y = 0;
    int max_x = 0;
    while (*text) {
        int code = (unsigned char)*text++;
        if (code == '\n') {
            x = 0;
            y += font_size;
            continue;            
        }
        if (code == '\r') {
            x = 0;
            continue;
        }
        if (glyphs[code].code == -1)
            code = '?';
        const Glyph &g = glyphs[code];
        if (g.w > 0) {
            int y0 = y - g.oy + font_ascent - g.h;
            
            if (y0 < min_y)
                min_y = y0;
            if (y0 + g.h > max_y)
                max_y = y0 + g.h;
        }
        min_x = std::min(min_x, x + g.ox);
        max_x = std::max(max_x, x + g.ox + g.w);
        x += g.advance;
    }
    xywh[0] = 0;
    xywh[1] = min_y;
    xywh[2] = max_x;
    xywh[3] = max_y - min_y;   
}

void ecv::gl::glText(double x, double y, const char *text,
                     double xywh[4])
{
    init_font();

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex_id);
    const double xstart = x;
    double max_x = x;

    glPushAttrib(GL_TRANSFORM_BIT | GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glLoadIdentity();
    glScalef(1.f / (float)font_img_width, 1.f / (float)font_img_height, 1.f);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    double min_y = 1e100;
    double max_y = -1e100;
    glBegin(GL_QUADS);
    while (*text) {
        int code = (unsigned char)*text++;
        if (code == '\n') {
            x = xstart;
            y += font_size;
            continue;            
        }
        if (code == '\r') {
            x = xstart;
            continue;
        }
        
        if (glyphs[code].code == -1)
            code = '?';
        const Glyph &g = glyphs[code];
        if (g.w > 0) {
            double x0 = x + g.ox;
            double y0 = y - g.oy + font_ascent - g.h;
            if (y0 < min_y)
                min_y = y0;
            if (y0 + g.h > max_y)
                max_y = y0 + g.h;
            
            glTexCoord2i(g.x, g.y);
            glVertex2d(x0, y0);
            
            glTexCoord2i(g.x + g.w, g.y);
            glVertex2d(x0 + g.w, y0);

            glTexCoord2i(g.x + g.w, g.y + g.h);
            glVertex2d(x0 + g.w, y0 + g.h);

            glTexCoord2i(g.x, g.y + g.h);
            glVertex2d(x0, y0 + g.h);
        }
        max_x = std::max(max_x, x + g.ox + g.w);
        x += g.advance;
    }
    glEnd();
    glPopMatrix();
    glPopAttrib();

    if (xywh) {
        xywh[0] = xstart;
        xywh[1] = min_y;
        xywh[2] = max_x - xstart;
        xywh[3] = max_y - min_y;
    }
}
