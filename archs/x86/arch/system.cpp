#include <ecv/system.hpp>

#include <dirent.h>
#include <glob.h>

bool ecv::expand_file_expression(const std::string &pattern,  std::vector<std::string> &filenames)
{
    filenames.clear();

    glob_t g;
    int err = glob(pattern.c_str(), GLOB_TILDE_CHECK | GLOB_BRACE, 0, &g);
    if (err != 0) {
        globfree(&g);
        if (err == GLOB_NOMATCH) {
            return true;
        }
        return false;
    }

    filenames.resize(g.gl_pathc);
    for (size_t i=0; i<filenames.size(); ++i) {
        filenames[i] = g.gl_pathv[i];
    }
    
    globfree(&g);
    return true;
}

bool ecv::list_directory(const std::string &dirname, std::vector<std::string> &filenames)
{
    DIR *dir = opendir(dirname.c_str());
    if (!dir)
        return false;

    filenames.clear();
    dirent *de = readdir(dir);    
    while (de != NULL) {
        filenames.push_back(de->d_name);
        de = readdir(dir);
    }
    closedir(dir);

    return true;
}
