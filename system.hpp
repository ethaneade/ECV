#ifndef ECV_SYSTEM_HPP
#define ECV_SYSTEM_HPP

#include <string>
#include <vector>

namespace ecv {

    bool list_directory(const std::string &dir, std::vector<std::string> &filenames);
    bool expand_file_expression(const std::string &pattern,  std::vector<std::string> &filenames);
}

#endif
