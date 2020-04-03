
CC=g++
INCLUDEPATH=include
CPPFLAGS=-I$(INCLUDEPATH) -Wall -std=c++17 -O2
LINKFLAGS=#-lpthread
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

fbrun: fbrun.cpp mpu6050.o i2cdev.o
	$(CC) $(CPPFLAGS) -o fbrun fbrun.cpp mpu6050.o i2cdev.o

mpu6050.o: $(INCLUDEPATH)/mpu6050/helper_3dmath.h $(INCLUDEPATH)/mpu6050/MPU6050_6Axis_MotionApps20.h $(INCLUDEPATH)/mpu6050/MPU6050.h $(INCLUDEPATH)/mpu6050/MPU6050.cpp
	$(CC) $(CPPFLAGS) -c -o mpu6050.o $(INCLUDEPATH)/mpu6050/MPU6050.cpp

i2cdev.o: $(INCLUDEPATH)/mpu6050/I2Cdev.h $(INCLUDEPATH)/mpu6050/I2Cdev.cpp
	$(CC) $(CPPFLAGS) -c -o i2cdev.o $(INCLUDEPATH)/mpu6050/I2Cdev.cpp

purge:
	rm -rf $(BUILDROOT)/$(BUILDPATH)/*

clean:
	rm -f $(BUILDROOT)/$(BUILDPATH)/*.o

