all: server.cc
	g++ -I $$UNKEI_EXTERNAL_ROOT/libuv/include \
			-g -rdynamic \
			-Wall -Wextra -pedantic \
			-std=c++11 \
			server.cc \
			ServerUv.cc \
			$$UNKEI_EXTERNAL_ROOT/libuv/.libs/libuv.a \
			-pthread \
			-o $$UNKEI_BUILD_ROOT/server
