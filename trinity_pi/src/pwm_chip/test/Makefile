CXX      = g++
LDFLAGS  =
CXXFLAGS = -std=c++14 -g -Wall -Wextra -Wfatal-errors -Wno-narrowing
TARGET   = test
SRCFILES = main.cpp ../pca9685.cpp
OBJECTS  = $(patsubst %.cpp, %.o, $(SRCFILES))

all: $(TARGET)

.c.o:
		$(CXX) $(CXXFLAGS) -c -o $@ $<

$(TARGET): $(OBJECTS)
		$(CXX) $(CXXFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

clean:
		rm -f *.o $(TARGET)
