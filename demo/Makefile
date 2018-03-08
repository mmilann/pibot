SRC=pibot_demo.cpp pibot.cpp
HEADERS=pibot.h
LDFLAGS= -lwiringPi
CXXFLAGS= -std=c++11

pibot_demo: ${SRC} ${HEADERS}
	${CXX} -g -o $@ ${SRC} ${LDFLAGS} ${CXXFLAGS}

.phony:
clean:
	rm bottest
