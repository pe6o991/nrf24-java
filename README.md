# nrf24-java
1.Written specifically for raspberry pi\
2.Requires pi4j\
\
\
This is a library for the nrf24l01 wireless module written in java.It tries to use as much of the syntax from the Arduino nrf library but due to the differences 
between java and c++ functions may require additional parameters or return different types.It uses pi4j library to access the SPI bus on the raspberry.There is a
build script provided by the pi4j library to build the class file.Currently it uses SPI0 on the raspberry pi(thus CS0 pin) and it uses gpio pin 6(might be different pin on the 40 pin header) for the EN pin on the nrf24l01 module.Also this library does not support interrupts comming from the nrf module.
