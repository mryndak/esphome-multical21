CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror
ROOT := ..

test: test_protocol
	./test_protocol

test_protocol: test_protocol.cpp test_vectors.h $(ROOT)/components/multical21/protocol.cpp $(ROOT)/components/multical21/protocol.h
	$(CXX) $(CXXFLAGS) -I$(ROOT)/components/multical21 test_protocol.cpp $(ROOT)/components/multical21/protocol.cpp -o $@

clean:
	rm -f test_protocol

.PHONY: test clean
