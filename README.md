Cinder-Wax3Accelerometer
========================

The [WAX3](http://axivity.com/v2/index.php?page=product.php&product=wax3) is a tiny, wireless 3-axis accelerometer built by [Axivity](http://axivity.com) that combines a MEMS inertial sensor with a low power radio, very useful for collecting real-time movement data.

This Block provides an interface to obtain data from the sensor from within a Cinder app, without the need to run any external software. It also provides a set of commonly needed functions to extract data from an accelerometer, such as history, general acceleration, pitch and roll, and can be easily extended for any other kind of accelerometer device.

It's tested only on Mac OS X and was built by [Adrià Navarro](http://adrianavarro.net) at [Red Paper Heart](http://redpaperheart.com).

Setting up the hardware
------------------------
0. Basic resources:
    - [Developer Guide](https://openmovement.googlecode.com/svn/trunk/Software/WAX3/WAX%20Developer%20Guide.pdf)
    - [Video Tutorial](http://www.youtube.com/watch?v=5wsR7MQ_Z48&list=PLC1bL6IftT9n9bDxPWErXxg7s7rr__6V7&index=2)
    - [Downloads](http://axivity.com/v2/index.php?page=product.php&product=wax3)

1. Setup at least one device as transmitter and one as receiver. This is done by updating the firmware on a real Windows machine. VMWare is not supported. See the developer guide or video for instructions.

2. Specify the settings for every device with [CoolTerm](http://freeware.the-meiers.org/) (mac) or another serial communication app.
    - Using CoolTerm, select the usbmodem Port in "Options" > "Serial Port Options"
    - Click "Connect"
    - For the receiver type the commands:
         - device=N (’N’ must be a unique integer)
         - channel=N (default is probably ok)
    - For the transmitter type the commands:
         - device=N (different than transmitter)
         - target=N (this should be the device # of the receiver you're targeting)
         - channel=N (same as in transmitter)
    - Disconnect 

Using the block
----------------

1. Clone the block into your blocks folder
2. Plug the usb receiver and check the port it's connected.
3. Create a new project with Tinderbox, or drag the src/ and include/ subfolders to your project.
4. Instantiate a Wax3Receiver for every receiver you have (usually one) and call setup with the port address.
5. Instantiate an Accelerometer for every transmitter with different ID that you have, and call setup passing the ID of the device, and the receiver as its data source.
6. On every frame, call update() in all your accelerometers and process their new data with getNumNewReadings(). See example for details.

If you have raw acceleration data coming from another device, you can extend the AccelDataSource class and use it as datasource for the same Accelerometer class.

Advanced
--------
This block is based on the [Waxrec command line app](https://code.google.com/p/openmovement/source/browse/trunk/Software/WAX3/waxrec/waxrec.c) written in C by Axivity. Waxrec provides a lot more functionality, such as logging, UDP input, OSC output, etc, that hasn't been ported to the block. While this covers most of the general cases needed in a realtime Cinder application, for some situations you might find the need to use waxrec instead.

**TO DO**: The original waxrec code runs well on Win and OS X, but for the sake of simplicity only the mac stuff has been ported. Ideally it would be good to make it multiplatform by replacing the low level C API calls with libraries built in Cinder like Serial. 

License
-------
Copyright (c) 2012, Red Paper Heart, All rights reserved. To contact Red Paper Heart, email hello@redpaperheart.com or tweet @redpaperhearts

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.