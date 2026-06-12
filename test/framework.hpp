// A small, dependency-free test framework. Self-registering test cases and a
// handful of assertion macros, so the suite builds with nothing but a C++20
// compiler (no network fetch, no submodule). run_all() returns a process exit
// code: 0 if every case passed, 1 otherwise.
#ifndef SLEW_TEST_FRAMEWORK_HPP
#define SLEW_TEST_FRAMEWORK_HPP

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

namespace slew::test {

struct Failure : std::exception {
  std::string message;
  explicit Failure(std::string m) : message(std::move(m)) {}
  const char* what() const noexcept override { return message.c_str(); }
};

struct Case {
  const char* name;
  void (*fn)();
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

struct Registrar {
  Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline std::string at(const char* file, int line) {
  return std::string(file) + ":" + std::to_string(line);
}

inline int run_all() {
  int passed = 0;
  int failed = 0;
  for (const auto& c : registry()) {
    try {
      c.fn();
      std::printf("[ ok ] %s\n", c.name);
      ++passed;
    } catch (const std::exception& e) {
      std::printf("[FAIL] %s\n       %s\n", c.name, e.what());
      ++failed;
    }
  }
  std::printf("----\n%d passed, %d failed, %d total\n", passed, failed,
              passed + failed);
  return failed == 0 ? 0 : 1;
}

}  // namespace slew::test

#define SLEW_TEST(test_name)                                             \
  static void test_name();                                              \
  static const ::slew::test::Registrar registrar_##test_name(#test_name, \
                                                             test_name);  \
  static void test_name()

#define CHECK(cond)                                                       \
  do {                                                                    \
    if (!(cond)) {                                                        \
      throw ::slew::test::Failure(::slew::test::at(__FILE__, __LINE__) +  \
                                  ": CHECK failed: " #cond);              \
    }                                                                     \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
  do {                                                                         \
    const double va = (a);                                                     \
    const double vb = (b);                                                     \
    if (std::fabs(va - vb) > (tol)) {                                          \
      throw ::slew::test::Failure(                                            \
          ::slew::test::at(__FILE__, __LINE__) + ": CHECK_NEAR failed: " #a    \
          "=" + std::to_string(va) + " vs " #b "=" + std::to_string(vb) +      \
          " (tol " + std::to_string(static_cast<double>(tol)) + ")");          \
    }                                                                          \
  } while (0)

#endif  // SLEW_TEST_FRAMEWORK_HPP
