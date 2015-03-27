#ifndef _MISC_H_
#define _MISC_H_

#include <sstream>
#include <vector>
#include <string>

namespace misc {
    // Converts a string to something
    template <typename T>
    T strTo (std::string const& str) {
        std::istringstream ss(str);
        T res;

        ss >> res;

        return res;
    }

    // Split a string on a delimiter
    inline std::vector<std::string> split (const std::string &s, char delim) {
        std::vector<std::string> elems;

        std::stringstream ss(s);
        std::string item;

        while (std::getline(ss, item, delim)) {
            elems.push_back(item);
        }

        return elems;
    }
}

#endif // _MISC_H_

