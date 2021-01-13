PKG = BalanceCar
EXE = balance-car
CPPFLAGS = -O3
include lean.mk

CPP_SRCS = Serial.cpp
CPP_OBJS = $(addprefix $(OUT)/cpp/,$(CPP_SRCS:.cpp=.o))

all: $(BIN_OUT)/$(EXE)

$(OUT)/cpp/%.o: %.cpp
	@mkdir -p "$(@D)"
	c++ -std=c++14 -c -o $@ $< $(CPPFLAGS) `leanc -print-cflags`

$(BIN_OUT)/$(EXE): $(LIB_OUT)/lib${PKG}.a $(CPP_OBJS) | $(BIN_OUT)
	c++ -o $@ $^ `leanc -print-ldflags`
