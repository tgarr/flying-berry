
CC=g++
INCLUDEPATH=include
CPPFLAGS=-I$(INCLUDEPATH) -Wall -O2
LINKFLAGS=-liniparser -lpigpio#-lpthread
LIBPATH=fblib

BUILDROOT=buildfs
BUILDPATH=/home/pi/flyingberry-build
BUILDUSER=pi

RPIADDR=192.168.100.6
RPIUSER=pi
RPIPATH=/home/pi/flyingberry

all: syncfiles cross-compile
	
deploy: all # deploy to raspberry using scp
	scp -r flyingberry.cfg $(BUILDROOT)/$(BUILDPATH)/fbrun $(RPIUSER)@$(RPIADDR):$(RPIPATH)

syncfiles:
	rsync -tr --delete Makefile flyingberry.cfg *.cpp include fblib $(BUILDROOT)/$(BUILDPATH)

cross-compile:
	sudo chroot --userspec=$(BUILDUSER) $(BUILDROOT) make -C $(BUILDPATH) compile

compile: fbrun

fbrun: fbrun.cpp drone.o config.o control.o imu.o motor.o pid.o
	$(CC) $(CPPFLAGS) $(LINKFLAGS) -o fbrun fbrun.cpp drone.o config.o control.o imu.o motor.o pid.o

drone.o: $(LIBPATH)/drone.hpp $(LIBPATH)/drone.cpp $(LIBPATH)/config.hpp $(LIBPATH)/imu.hpp $(LIBPATH)/common.hpp $(LIBPATH)/control.hpp $(LIBPATH)/motor.hpp $(LIBPATH)/pid.hpp
	$(CC) $(CPPFLAGS) -c -o drone.o $(LIBPATH)/drone.cpp

config.o: $(LIBPATH)/config.hpp $(LIBPATH)/config.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o config.o $(LIBPATH)/config.cpp

control.o: $(LIBPATH)/control.hpp $(LIBPATH)/control.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o control.o $(LIBPATH)/control.cpp

imu.o: $(LIBPATH)/imu.hpp $(LIBPATH)/imu.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o imu.o $(LIBPATH)/imu.cpp

motor.o: $(LIBPATH)/motor.hpp $(LIBPATH)/motor.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o motor.o $(LIBPATH)/motor.cpp

pid.o: $(LIBPATH)/pid.hpp $(LIBPATH)/pid.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o pid.o $(LIBPATH)/pid.cpp

purge:
	rm -rf $(BUILDROOT)/$(BUILDPATH)/*

clean:
	rm -f $(BUILDROOT)/$(BUILDPATH)/*.o

