// Lightweight strong-type units header.
//
// The assignment recommends mp-units, but to keep this submission free of
// external dependencies (and compilable with a plain g++ invocation) we
// provide our own minimal Length / Angle types that mimic the same surface
// API: literals such as `5 * cm`, `90 * deg`, `1 * m` produce strong types
// instead of bare doubles.
//
// All Length values are stored internally in centimeters, and all Angle
// values in degrees, matching the on-disk file format described in the
// project README.
#pragma once

#include <cmath>
#include <compare>

namespace units {

class Length {
    double cm_;
public:
    constexpr Length() : cm_(0.0) {}
    constexpr explicit Length(double v_cm) : cm_(v_cm) {}

    constexpr double in_cm() const { return cm_; }
    constexpr double in_m()  const { return cm_ / 100.0; }

    constexpr Length operator+(Length o) const { return Length(cm_ + o.cm_); }
    constexpr Length operator-(Length o) const { return Length(cm_ - o.cm_); }
    constexpr Length operator-() const          { return Length(-cm_); }
    constexpr Length& operator+=(Length o) { cm_ += o.cm_; return *this; }
    constexpr Length& operator-=(Length o) { cm_ -= o.cm_; return *this; }

    constexpr Length operator*(double s) const { return Length(cm_ * s); }
    constexpr Length operator/(double s) const { return Length(cm_ / s); }
    constexpr double operator/(Length o) const { return cm_ / o.cm_; }

    constexpr bool operator==(Length o) const { return cm_ == o.cm_; }
    constexpr auto operator<=>(Length o) const { return cm_ <=> o.cm_; }
};

class Angle {
    double deg_;
public:
    constexpr Angle() : deg_(0.0) {}
    constexpr explicit Angle(double v_deg) : deg_(v_deg) {}

    constexpr double in_deg() const { return deg_; }
    double in_rad() const { return deg_ * 3.14159265358979323846 / 180.0; }

    constexpr Angle operator+(Angle o) const { return Angle(deg_ + o.deg_); }
    constexpr Angle operator-(Angle o) const { return Angle(deg_ - o.deg_); }
    constexpr Angle operator-() const         { return Angle(-deg_); }
    constexpr Angle& operator+=(Angle o) { deg_ += o.deg_; return *this; }
    constexpr Angle& operator-=(Angle o) { deg_ -= o.deg_; return *this; }

    constexpr Angle operator*(double s) const { return Angle(deg_ * s); }
    constexpr Angle operator/(double s) const { return Angle(deg_ / s); }

    constexpr bool operator==(Angle o) const { return deg_ == o.deg_; }
    constexpr auto operator<=>(Angle o) const { return deg_ <=> o.deg_; }

    Angle normalized() const {
        double d = std::fmod(deg_, 360.0);
        if (d < 0.0) d += 360.0;
        return Angle(d);
    }
};

// Unit constants used to construct quantities like `5 * cm`, `90 * deg`.
inline constexpr Length cm{1.0};
inline constexpr Length m {100.0};
inline constexpr Angle  deg{1.0};

constexpr Length operator*(double s, Length u) { return Length(s * u.in_cm()); }
constexpr Length operator*(int s,    Length u) { return Length(static_cast<double>(s) * u.in_cm()); }
constexpr Angle  operator*(double s, Angle u)  { return Angle(s * u.in_deg()); }
constexpr Angle  operator*(int s,    Angle u)  { return Angle(static_cast<double>(s) * u.in_deg()); }

} // namespace units
