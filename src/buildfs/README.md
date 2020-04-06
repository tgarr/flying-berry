# Build file system

Put here the whole Raspbian file system for cross-compiling using qemu-user-static. The following packets should be installed in this copy of Raspbian:

```sudo chroot buildfs apt-get install libpigpio-dev libpigpiod-if-dev libiniparser-dev```

The Makefile will by default copy the files (rsync) to buildfs/home/pi/flyingberry-build and cross-compile them for arm. There is also a target for compiling and deploying it to a raspberry pi:

```make deploy```

This will compile the code and transfer it using scp to a running raspberry pi. IP address and path should be set in the Makefile.

