# battsim

This project is used to convert modbus messages to http. The project has dependencies on libmodbus, libcurl
libmicrohttpd and libjson.

Installing dependencies

cd workspace
# remove any previous installation on raspberry pi
sudo apt-get remove libmicrohttpd-dev 

#clone from git
git clone https://gnunet.org/git/libmicrohttpd.git

# installing microhttpd
sudo apt-get install texi2html texinfo autopoint

# run autoreconf to create "configure"
$ autoreconf -fi
$ ./configure
$ make
$ sudo make install

#install libcurl
sudo apt-get install libcurl4-openssl-dev

#include libjson
sudo apt-get install libjson0 libjson0-dev

The project supports command line argument for the destination IP address and port number.

In order to see list of argument supported see the help by issuing the following command

Running the executable

$ ./battsim -h

To build simply clone and build using the command below 
$ make 

To clean the project issue the following command 
$ make clean

To enable watchdog support on the executable run the following command to a cron timer job
$ make cronjobstart

To disable watchdog support
$ make cronjobstop

