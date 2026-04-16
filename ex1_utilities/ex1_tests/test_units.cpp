// Included by test_main.cpp
#include "units/Units.h"

using namespace units;

TEST(units_length_construction) {
    Length a = 5 * cm;
    CHECK_NEAR(a.in_cm(), 5.0, 1e-9);
    Length b = 1 * m;
    CHECK_NEAR(b.in_cm(), 100.0, 1e-9);
    CHECK_NEAR(b.in_m(), 1.0, 1e-9);
}

TEST(units_length_arithmetic) {
    Length a = 100 * cm;
    Length b = 50 * cm;
    CHECK_NEAR((a + b).in_cm(), 150.0, 1e-9);
    CHECK_NEAR((a - b).in_cm(), 50.0, 1e-9);
    CHECK_NEAR((a * 2.0).in_cm(), 200.0, 1e-9);
    CHECK_NEAR((a / 2.0).in_cm(), 50.0, 1e-9);
    CHECK(a > b);
    CHECK(b < a);
    CHECK(a == 100 * cm);
}

TEST(units_angle_normalization) {
    Angle a = 450 * deg;
    CHECK_NEAR(a.normalized().in_deg(), 90.0, 1e-9);
    Angle b = -45 * deg;
    CHECK_NEAR(b.normalized().in_deg(), 315.0, 1e-9);
}

TEST(units_angle_radians) {
    Angle a = 180 * deg;
    CHECK_NEAR(a.in_rad(), 3.14159265, 1e-6);
}
