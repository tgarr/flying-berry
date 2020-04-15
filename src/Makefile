
CC=g++
INCLUDEPATH=include
CPPFLAGS=-I$(INCLUDEPATH) -Wall -O2
LINKFLAGS=-liniparser -lpigpio#-lpthread
LIBPATH=fblib

BUILDROOT=buildfs
BUILDPATH=/home/pi/flyingberry-build
BUILDUSER=pi

RPIADDR=192.168.100.5
RPIUSER=pi
RPIPATH=/home/pi/flyingberry

all: syncfiles cross-compile
	
deploy: all # deploy to raspberry using scp
	scp -r flyingberry.cfg $(BUILDROOT)/$(BUILDPATH)/flyingberry $(BUILDROOT)/$(BUILDPATH)/calibrate_esc $(BUILDROOT)/$(BUILDPATH)/test_motor $(RPIUSER)@$(RPIADDR):$(RPIPATH)

syncfiles:
	rsync -tr --delete Makefile flyingberry.cfg *.cpp include fblib $(BUILDROOT)/$(BUILDPATH)

cross-compile:
	sudo chroot --userspec=$(BUILDUSER) $(BUILDROOT) make -C $(BUILDPATH) compile

compile: flyingberry calibrate_esc test_motor

flyingberry: flyingberry.cpp drone.o config.o steam.o imu.o motor.o pid.o steamcontroller.a
	$(CC) $(CPPFLAGS) $(LINKFLAGS) -o flyingberry flyingberry.cpp drone.o config.o steam.o imu.o motor.o pid.o steamcontroller.a

calibrate_esc: calibrate_esc.cpp motor.o config.o
	$(CC) $(CPPFLAGS) $(LINKFLAGS) -o calibrate_esc calibrate_esc.cpp config.o motor.o

test_motor: test_motor.cpp motor.o config.o
	$(CC) $(CPPFLAGS) $(LINKFLAGS) -o test_motor test_motor.cpp config.o motor.o

drone.o: $(LIBPATH)/drone.hpp $(LIBPATH)/drone.cpp $(LIBPATH)/config.hpp $(LIBPATH)/imu.hpp $(LIBPATH)/common.hpp $(LIBPATH)/controller.hpp $(LIBPATH)/motor.hpp $(LIBPATH)/pid.hpp
	$(CC) $(CPPFLAGS) -c -o drone.o $(LIBPATH)/drone.cpp

config.o: $(LIBPATH)/config.hpp $(LIBPATH)/config.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o config.o $(LIBPATH)/config.cpp

steam.o: $(LIBPATH)/steam.hpp $(LIBPATH)/steam.cpp $(LIBPATH)/common.hpp $(LIBPATH)/controller.hpp
	$(CC) $(CPPFLAGS) -c -o steam.o $(LIBPATH)/steam.cpp

imu.o: $(LIBPATH)/imu.hpp $(LIBPATH)/imu.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o imu.o $(LIBPATH)/imu.cpp

motor.o: $(LIBPATH)/motor.hpp $(LIBPATH)/motor.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o motor.o $(LIBPATH)/motor.cpp

pid.o: $(LIBPATH)/pid.hpp $(LIBPATH)/pid.cpp $(LIBPATH)/common.hpp
	$(CC) $(CPPFLAGS) -c -o pid.o $(LIBPATH)/pid.cpp

steamcontroller.a: $(INCLUDEPATH)/steamcontroller/steamcontroller.h $(INCLUDEPATH)/steamcontroller/steamcontroller*.c
	$(CC) $(CPPFLAGS) -c -fpermissive $(INCLUDEPATH)/steamcontroller/steamcontroller*.c
	ar rcs steamcontroller.a steamcontroller*.o

purge:
	rm -rf $(BUILDROOT)/$(BUILDPATH)/*

clean:
	rm -f $(BUILDROOT)/$(BUILDPATH)/*.o

