#ifndef _MISC_HPP_
#define _MISC_HPP_

namespace misc {
    // Clamps a value between a minimum and a maxmimum.
    template <typename T>
    static T clamp (T const& val, T const& m, T const& M) {
        if (val <= m) {
            return m;
        } else if (val >= M) {
            return M;
        } else {
            return val;
        }
    }

    // Colour ramp from green to red.
    static void ramp (double val,
                      double &r,
                      double &g,
                      double &b) {
        val = clamp(val, 0.0, 1.0);

        b = 0;

        if (val <= 0.5) {
            r = 2 * val;
            g = 1.0;
        } else {
            r = 1.0;
            g = 1.0 - 2 * (val - 0.5);
        }
    }
}

#endif

