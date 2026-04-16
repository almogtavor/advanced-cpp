#pragma once  // Prevent repeated inclusion of the mini test framework.

#include <functional>  // std::function stores each test body.
#include <stdexcept>   // std::runtime_error is thrown for assertion failures.
#include <string>      // Test names are stored as strings.
#include <vector>      // Tests are kept in a vector registry.

struct TestCase {                      // One registered test case.
  std::string name;                    // Human-readable test name.
  std::function<void()> fn;            // Callable that executes the test body.
};

std::vector<TestCase>& registry();     // Global registry accessor.

struct RegisterTest {                  // Helper that auto-registers a test during static initialization.
  RegisterTest(std::string name, std::function<void()> fn);  // Constructor pushes the test into the registry.
};

// Macro that declares, registers, and defines a test function.
#define DM_TEST(name) \
  void name(); \
  static RegisterTest name##_register{#name, name}; \
  void name()

// Assert that an expression is true.
#define DM_ASSERT_TRUE(expr) \
  do { \
    if (!(expr)) throw std::runtime_error("Assertion failed: " #expr); \
  } while (false)

// Assert that two expressions compare equal.
#define DM_ASSERT_EQ(a, b) \
  do { \
    if (!((a) == (b))) throw std::runtime_error("Assertion failed: " #a " == " #b); \
  } while (false)
