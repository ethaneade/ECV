#include <ecv/system.hpp>
#include <windows.h>
#include <tchar.h>
#include <stdio.h>

bool ecv::expand_file_expression(const std::string &pattern,  std::vector<std::string> &filenames)
{
    filenames.clear();

    std::string prefix;
    size_t last_slash = pattern.find_last_of("/\\", pattern.length(), 2);
    if (last_slash !=std::string::npos) {
        prefix = pattern.substr(0, last_slash+1);
    }
    
    WIN32_FIND_DATA ffd;
    HANDLE hFind = FindFirstFile(pattern.c_str(), &ffd);
    
    if (INVALID_HANDLE_VALUE == hFind) {
        DWORD err = GetLastError();
        if (err == ERROR_NO_MORE_FILES)
            return true;
        
        return false;
    }

    do {
        filenames.push_back(prefix + std::string(ffd.cFileName));
    } while (FindNextFile(hFind, &ffd) != 0);
 
    DWORD err = GetLastError();
    FindClose(hFind);
    
    if (err != ERROR_NO_MORE_FILES)
        return false;
    
    return true;
}

bool ecv::list_directory(const std::string &dirname, std::vector<std::string> &filenames)
{
    std::string pattern = dirname;
    if (!pattern.empty() && pattern[pattern.length()-1] == '\\')
        pattern += "*";
    else
        pattern += std::string("\\*");
    
    return expand_file_expression(pattern, filenames);
}
