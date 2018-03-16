# mb2http 

This project is used to convert modbus messages to http. The project has dependencies on libmodbus and libcurl 
The project supports command line argument for the destination IP address and port number.

In order to see list of argument supported see the help by issuing the following command

$ ./mb2http -h

To build simply clone and build using the command below 
$ make 

To clean the project issue the following command 
$ make clean

To enable watchdog support on the executable run the following command to a cron timer job
$ make cronjobstart

To disable watchdog support
$ make cronjobstop

