#
# Build variables
#

# general c++ variables
CXX			:=g++
CFLAGS	:=-Wall -Wextra -pedantic -std=c++11

# general project variables
PROJECT_ROOT	:=$(UNKEI_PROJECT_ROOT)
BUILD_ROOT		:=$(UNKEI_BUILD_ROOT)
SOURCE_ROOT		:=$(PROJECT_ROOT)/src
EXTERNAL_ROOT	:=$(UNKEI_EXTERNAL_ROOT)
TEST_ROOT			:=$(PROJECT_ROOT)/test

# server build variables
SERVER_ROOT		:=$(SOURCE_ROOT)/server
CORE_ROOT			:=$(SERVER_ROOT)/core
CORE_TEST_DIR	:=$(CORE_ROOT)/tests
CORE_TEST_SRC	:=$(wildcard (CORE_ROOT)/*_test.cc)
SERVER_BUILD	:=$(BUILD_ROOT)/server
SERVER_TEST		:=$(TEST_ROOT)/server
CORE_SRCS			:=$(wildcard $(CORE_ROOT)/*.cc)
CORE_HDRS			:=$(wildcard $(CORE_ROOT)/*.h)

# external libraries
GOOGLE_TEST_A	:=$(EXTERNAL_ROOT)/googletest/googletest/make/gtest_main.a
GOOGLE_TEST_I := -isystem $(EXTERNAL_ROOT)/googletest/googletest/include
LIBUV_A				:=$(EXTERNAL_ROOT)/libuv/.libs/libuv.a
LIBUV_I				:=$(EXTERNAL_ROOT)/libuv/include

.PHONY:	clean

all: server-core

#
# Server build rules.
#
$(CORE_SRCS):$(CORE_HDRS)

server-core: $(CORE_SRCS) $(LIBUV_A) | $(SERVER_BUILD)
	$(CXX) $(CFLAGS) -I $(LIBUV_I) $^ -o $(SERVER_BUILD)/$@ \
		-pthread -g -rdynamic

# Server build rules: tests.
server-core-tests: $(SERVER_TEST)/BufferedReader_test

$(SERVER_TEST)/BufferedReader_test: $(CORE_TEST_DIR)/BufferedReader_test.cc \
		$(CORE_ROOT)/BufferedReader.cc $(CORE_ROOT)/BufferedReader.h \
		$(CORE_ROOT)/ccuv.h $(CORE_ROOT)/ccuv.cc | $(SERVER_TEST)
	$(CXX) $(CFLAGS) $(GOOGLE_TEST_I) \
	$(CORE_TEST_DIR)/BufferedReader_test.cc $(CORE_ROOT)/BufferedReader.cc \
	$(CORE_ROOT)/ccuv.h $(GOOGLE_TEST_A) \
	-o $(SERVER_TEST)/BufferedReader_test -pthread

server-core-run-tests: server-core-run-buffered-reader-test

server-core-run-buffered-reader-test: $(SERVER_TEST)/BufferedReader_test
	$(SERVER_TEST)/BufferedReader_test

#
# Setup build directory tree.
#
$(TEST_ROOT):
	mkdir $(TEST_ROOT)

$(BUILD_ROOT):
	mkdir $(BUILD_ROOT)

$(SERVER_TEST): $(TEST_ROOT)
	mkdir $(SERVER_TEST)

$(SERVER_BUILD): $(BUILD_ROOT)
	mkdir $(SERVER_BUILD)

#
# Cleanup.
#
clean:
	rm -rf $(BUILD_ROOT)
