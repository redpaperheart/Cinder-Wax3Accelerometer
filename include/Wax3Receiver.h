/* 
 Wax3Receiver
 Class to obtain data from the Wax3 device via serial port.
 
 Code based on the waxrec application:
 https://code.google.com/p/openmovement/source/browse/trunk/Software/WAX3/waxrec

 For more information read developer guide:
 http://openmovement.googlecode.com/svn/trunk/Software/WAX3/WAX%20Developer%20Guide.pdf
 */

/*
 Created by Adrià Navarro at Red Paper Heart
 
 Copyright (c) 2012, Red Paper Heart
 All rights reserved.
 
 This code is designed for use with the Cinder C++ library, http://libcinder.org
 
 To contact Red Paper Heart, email hello@redpaperheart.com or tweet @redpaperhearts
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:
 
 * Redistributions of source code must retain the above copyright notice, this list of conditions and
 the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */
 
#pragma once

// Cinder stuff
#include "cinder/app/App.h"
#include "cinder/Thread.h"
#include "cinder/ConcurrentCircularBuffer.h"

// Sockets
#include <termios.h>
#include <fcntl.h>
#include <sys/timeb.h>

// USB IDs
#define DEFAULT_VID 0x04D8           /* USB Vendor ID  */
#define DEFAULT_PID 0x000A           /* USB Product ID */

// Data Source
#include "AccelDataSource.h"
#define BUFFER_SIZE 0xffff

// Wax Structures
#define MAX_SAMPLES 32

typedef struct
{
    unsigned long long timestamp;
    unsigned short sampleIndex;
    short x, y, z;
} WaxSample;

typedef struct
{
    unsigned long long timestamp;
    unsigned short deviceId;
    unsigned short sequenceId;
    unsigned char sampleCount;
    WaxSample samples[MAX_SAMPLES];
} WaxPacket;



using namespace std;
using namespace ci;

class Wax3Receiver : public AccelDataSource {
public:
    Wax3Receiver() {};
    ~Wax3Receiver();
    
    // Start and stop
    int setup(string portName);
    int stop();
    void setDebug(bool b) { bDebug = b; }
    
    // AccelDataSource protocol
    bool    hasNewReadings(ushort id);
    Vec3f   getNextReading(ushort id);
    

private:
    // start functions
    int         openport(const char* infile, char writeable);

    // input thread parsing
    int         readPackets();
    size_t      slipread(int fd, void *inBuffer, size_t len);
    size_t      lineread(int fd, void *inBuffer, size_t len);
    WaxPacket*  parseWaxPacket(const void *inputBuffer, size_t len, unsigned long long now);
    
    // utils
    void        printWax(WaxPacket *waxPacket, int timeformat);
    const char* timestamp(unsigned long long ticks);
    unsigned long long TicksNow(void);

    // members
    bool        bDebug;
    int         mFd;
    const char* mInfile;
    string      mPortName;
    shared_ptr<thread> mThread;
    
    map<ushort, ConcurrentCircularBuffer<WaxSample>* > mBuffers;


};

