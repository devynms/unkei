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
SERVER_BUILD	:=$(BUILD_ROOT)/server
SERVER_TEST		:=$(TEST_ROOT)/server
CORE_SRCS			:=$(wildcard $(CORE_ROOT)/*.cc)
CORE_HDRS			:=$(wildcard $(CORE_ROOT)/*.h)

.PHONY:	clean

all: server-core

#
# Server build rules.
#
$(CORE_SRCS):$(CORE_HDRS)

server-core: $(CORE_SRCS) | $(SERVER_BUILD)
	$(CXX) $(CFLAGS) -I $(EXTERNAL_ROOT)/libuv/include $^ \
		$(EXTERNAL_ROOT)/libuv/.libs/libuv.a \
		-o $(BUILD_ROOT)/server/$@ -pthread -g -rdynamic

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
