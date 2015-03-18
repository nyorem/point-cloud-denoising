#ifndef _CONST_HPP_
#define _CONST_HPP_

#include <boost/random.hpp>

namespace consts {
    // Dimensions of the main window
    static const int width_window = 800;
    static const int height_window = 600;

    // Dimensions of the view (left side of the window)
    static const int width_view = width_window - 200;
    static const int height_view = height_window;

    // Global random number generator
    static boost::mt19937 g_eng;
};

#endif

