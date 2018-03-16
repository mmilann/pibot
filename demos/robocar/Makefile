SRC=robocar.cpp ../../driver_cpp/pibot.cpp
HEADERS=../../driver_cpp/pibot.h
INC_DIR = ../../driver_cpp
LDFLAGS= -lwiringPi
CXXFLAGS= -std=c++11 -I$(INC_DIR)

robocar: ${SRC} ${HEADERS}
	${CXX} -g -o $@ ${SRC} ${LDFLAGS} ${CXXFLAGS}

.phony:
clean:
	rm robocar
