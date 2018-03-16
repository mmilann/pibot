SRC=pibot_demo.cpp ../driver_cpp/pibot.cpp
HEADERS=../driver_cpp/pibot.h
LDFLAGS= -lwiringPi
CXXFLAGS= -std=c++11

pibot_demo: ${SRC} ${HEADERS}
	${CXX} -g -o $@ ${SRC} ${LDFLAGS} ${CXXFLAGS}

.phony:
clean:
	rm bottest
