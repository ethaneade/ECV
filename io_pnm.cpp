#include <ecv/io_pnm.hpp>
#include <iostream>
#include <ctype.h>

using namespace std;
using namespace ecv;

static void skip_space(std::istream& in)
{            
    while (in.good() && isspace(in.get()));
    in.unget();            
}

static bool skip_comment(std::istream& in)
{
    char c = in.get();
    if (c != '#') {
        in.unget();
        return false;
    }
    while (in.good() && in.get() != '\n');
    return true;
}
        
static bool read_num(std::istream& in, int &n)
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

bool ecv::pnm_io::read_header(std::istream& in,
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

