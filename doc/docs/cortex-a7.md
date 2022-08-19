# Cortex A7 Application

## Main entry point

The main entry point of the Cortex A7 application is a simple C program which starts 
one thread for each function required by the cortex A7 application.
These threads run in parallel, controlled by the linux scheduler. <br>
The main entry point will then monitor the threads and restart them in case of unexpected interruption. 

## Object dictionnary logger

The object dictionnary logger will connect to the `/dev/ttyRPMSG1` inter processor comunication device and
store into memory (linux files) the data received. This data will come mainly from the cortex
M4 application.

## Feedback logger

## Kalman filter