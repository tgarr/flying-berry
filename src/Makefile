
CC=g++
CPPFLAGS=-Iinclude -Wall -std=c++17 -O2
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

fbrun: fbrun.cpp
	$(CC) $(CPPFLAGS) -o fbrun fbrun.cpp

purge:
	rm -rf $(BUILDROOT)/$(BUILDPATH)/*

clean:
	rm -f $(BUILDROOT)/$(BUILDPATH)/*.o

