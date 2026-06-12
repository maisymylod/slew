# Zero-dependency build: needs only a C++20 compiler and make. CMake is the
# canonical build (see CMakeLists.txt and CI); this Makefile exists so the
# project builds and tests on a bare machine with nothing fetched.
#
#   make            build the slew CLI
#   make test       build and run the test suite
#   make demo       build and run a 30-degree slew
#   make asan       build and run the tests under ASan/UBSan
#   make clean

CXX      ?= clang++
CXXFLAGS ?= -std=c++20 -O2 -Wall -Wextra -Wpedantic -Iinclude
SANFLAGS  = -std=c++20 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Iinclude

CORE_SRC  = src/simulation.cpp
CLI_SRC   = src/main.cpp $(CORE_SRC)
TEST_SRC  = $(wildcard test/test_*.cpp) $(CORE_SRC)
HEADERS   = $(wildcard include/slew/*.hpp) test/framework.hpp

BIN = bin

.PHONY: all test demo asan clean
all: $(BIN)/slew

$(BIN)/slew: $(CLI_SRC) $(HEADERS) | $(BIN)
	$(CXX) $(CXXFLAGS) $(CLI_SRC) -o $@

$(BIN)/slew_tests: $(TEST_SRC) $(HEADERS) | $(BIN)
	$(CXX) $(CXXFLAGS) -Itest $(TEST_SRC) -o $@

test: $(BIN)/slew_tests
	./$(BIN)/slew_tests

demo: $(BIN)/slew
	./$(BIN)/slew --slew-deg 30

asan: | $(BIN)
	$(CXX) $(SANFLAGS) -Itest $(TEST_SRC) -o $(BIN)/slew_tests_asan
	./$(BIN)/slew_tests_asan

$(BIN):
	mkdir -p $(BIN)

clean:
	rm -rf $(BIN)
