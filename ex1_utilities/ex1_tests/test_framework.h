#pragma once

// Tiny dependency-free test framework. We avoid GTest because the
// assignment forbids external libraries by default. The framework
// supports CHECK / CHECK_EQ / CHECK_NEAR / CHECK_THROWS macros and a
// TEST(name) macro that registers a test in a global vector.

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace tf {

struct Test {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<Test>& registry() {
    static std::vector<Test> r;
    return r;
}

struct AutoRegister {
    AutoRegister(const char* name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

struct CheckFailure : public std::exception {
    std::string message;
    explicit CheckFailure(std::string msg) : message(std::move(msg)) {}
    const char* what() const noexcept override { return message.c_str(); }
};

inline int run_all() {
    int failed = 0;
    int total  = 0;
    for (const auto& t : registry()) {
        ++total;
        try {
            t.fn();
            std::cout << "[ OK ] " << t.name << "\n";
        } catch (const CheckFailure& e) {
            ++failed;
            std::cout << "[FAIL] " << t.name << "\n  " << e.what() << "\n";
        } catch (const std::exception& e) {
            ++failed;
            std::cout << "[FAIL] " << t.name << "\n  unexpected exception: "
                      << e.what() << "\n";
        } catch (...) {
            ++failed;
            std::cout << "[FAIL] " << t.name << "\n  unknown exception\n";
        }
    }
    std::cout << "----\n" << (total - failed) << "/" << total
              << " tests passed\n";
    return failed == 0 ? 0 : 1;
}

} // namespace tf

#define TF_CONCAT_INNER(a, b) a##b
#define TF_CONCAT(a, b) TF_CONCAT_INNER(a, b)

#define TEST_IMPL(name, id)                                                \
    static void TF_CONCAT(tf_fn_, id)();                                   \
    static ::tf::AutoRegister TF_CONCAT(tf_reg_, id)(                      \
        #name, &TF_CONCAT(tf_fn_, id));                                    \
    static void TF_CONCAT(tf_fn_, id)()

#define TEST(name) TEST_IMPL(name, __COUNTER__)

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::ostringstream _oss;                                       \
            _oss << __FILE__ << ":" << __COUNTER__                            \
                 << ": CHECK(" #cond ") failed";                           \
            throw ::tf::CheckFailure(_oss.str());                          \
        }                                                                  \
    } while (0)

#define CHECK_EQ(a, b)                                                     \
    do {                                                                   \
        const auto _va = (a);                                              \
        const auto _vb = (b);                                              \
        if (!(_va == _vb)) {                                               \
            std::ostringstream _oss;                                       \
            _oss << __FILE__ << ":" << __COUNTER__                            \
                 << ": CHECK_EQ(" #a ", " #b ") failed: "                  \
                 << _va << " != " << _vb;                                  \
            throw ::tf::CheckFailure(_oss.str());                          \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                              \
    do {                                                                   \
        const double _va = static_cast<double>(a);                         \
        const double _vb = static_cast<double>(b);                         \
        if (std::fabs(_va - _vb) > (eps)) {                                \
            std::ostringstream _oss;                                       \
            _oss << __FILE__ << ":" << __COUNTER__                            \
                 << ": CHECK_NEAR(" #a ", " #b ", " #eps ") failed: "      \
                 << _va << " vs " << _vb;                                  \
            throw ::tf::CheckFailure(_oss.str());                          \
        }                                                                  \
    } while (0)
