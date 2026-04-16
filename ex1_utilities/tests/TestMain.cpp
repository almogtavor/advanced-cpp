#include "TestFramework.hpp"  // Minimal custom test framework declarations.

#include <exception>  // std::exception base class for caught failures.
#include <iostream>   // Used to print PASS/FAIL output.
#include <utility>    // std::move is used when registering tests.

std::vector<TestCase>& registry() {  // Return the singleton test registry.
  static std::vector<TestCase> tests;  // Static local vector created once on first use.
  return tests;                        // Return the shared registry.
}

RegisterTest::RegisterTest(std::string name, std::function<void()> fn) {  // Auto-register one test case.
  registry().push_back(TestCase{std::move(name), std::move(fn)});          // Move the test metadata into the registry.
}

int main() {                                                                // Entry point for the test executable.
  int failed = 0;                                                           // Count failed tests.
  for (const auto& test : registry()) {                                     // Run tests in registration order.
    try {                                                                   // Catch assertion failures per test so remaining tests still run.
      test.fn();                                                            // Execute the test body.
      std::cout << "[PASS] " << test.name << '\n';                         // Print a PASS line if no exception was thrown.
    } catch (const std::exception& ex) {                                    // Catch standard assertion/runtime failures.
      ++failed;                                                             // Increment the failure counter.
      std::cout << "[FAIL] " << test.name << ": " << ex.what() << '\n';    // Print a FAIL line with the reason.
    }
  }
  return failed == 0 ? 0 : 1;                                               // Return success only if every test passed.
}
