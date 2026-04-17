// Strong-type units header based on the mp-units library.
//
// Provides Length and Angle type aliases and unit symbols (cm, m, deg)
// using the mp-units library as required by the assignment.
//
// Usage:  5.0 * cm, 1.0 * m, 90.0 * deg
// Extract: length.numerical_value_in(cm), angle.numerical_value_in(deg)
#pragma once

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/si/unit_symbols.h>
#include <cmath>

namespace units {

// Bring mp-units symbols into the units namespace for convenient use.
using namespace mp_units::si::unit_symbols;   // m, cm, km, ...
using mp_units::non_si::unit_symbols::deg;    // deg
using mp_units::one;                          // dimensionless unit

// Type aliases matching the assignment's surface API.
using Length = decltype(1.0 * cm);
using Angle  = decltype(1.0 * deg);

// Angle normalization helper (mp-units doesn't provide this).
inline Angle normalized(Angle a) {
    double d = std::fmod(a.numerical_value_in(deg), 360.0);
    if (d < 0.0) d += 360.0;
    return d * deg;
}

// Convert angle to radians as a raw double (for std::sin/cos calls).
inline double to_rad(Angle a) {
    return a.numerical_value_in(deg) * 3.14159265358979323846 / 180.0;
}

} // namespace units
