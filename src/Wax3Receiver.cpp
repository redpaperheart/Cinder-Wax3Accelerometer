/*
 Created by Adrià Navarro at Red Paper Heart
 
 Copyright (c) 2015, Red Paper Heart
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

#include "Wax3Receiver.h"
#include "cinder/Serial.h"

/* -------------------------------------------------------------------------------------------------- */
#pragma mark AccelDataSource protocol
/* -------------------------------------------------------------------------------------------------- */

bool Wax3Receiver::hasNewReadings(ushort id)
{
    return mBuffers.count(id) == 1 && mBuffers.at(id)->isNotEmpty();
}

Vec3f Wax3Receiver::getNextReading(ushort id)
{
    WaxSample sample;
    mBuffers.at(id)->popBack(&sample);
    return Vec3f(sample.x/256.0f, sample.y/256.0f, sample.z/256.0f);
}


/* -------------------------------------------------------------------------------------------------- */
#pragma mark constructors and setup
/* -------------------------------------------------------------------------------------------------- */

Wax3Receiver::Wax3Receiver()
{
    bConnected = false;
    bCloseThread = false;
    bDebug = false;
}

Wax3Receiver::Wax3Receiver(std::string portName) : Wax3Receiver()
{
    setup(portName);
}

Wax3Receiver::~Wax3Receiver()
{
    closeThread();
}

/* Close input and thread */
bool Wax3Receiver::stop()
{
    closeThread();
    bConnected = false;
    return true;
}

bool Wax3Receiver::closeThread()
{
    bCloseThread = true;
    if(mThread && mThread->joinable())
    {
        mThread->join();
        return true;
    }
    return false;
}

bool Wax3Receiver::setup(string portName)
{
    bConnected = false;
    
    for( auto device : Serial::getDevices()) app::console() << device.getName() << ", " << device.getPath() << std::endl;
    try {
        Serial::Device device = Serial::findDeviceByNameContains(portName);
        mSerial = Serial(device, 115200);
        app::console() << "Receiver sucessfully connected to " << portName << std::endl;
    }
    catch(SerialExc e) {
        app::console() << "Receiver unable to connect to " << portName << ": " << e.what() << std::endl;
        bConnected = false;
        return false;
    }
    
    /* Start Input thread */
    closeThread();
    mThread = shared_ptr<thread>( new thread( bind( &Wax3Receiver::readPackets, this ) ) );
    bConnected = true;
    return true;
}

/* -------------------------------------------------------------------------------------------------- */
#pragma mark input thread
/* -------------------------------------------------------------------------------------------------- */

int Wax3Receiver::readPackets()
{
    ci::ThreadSetup threadSetup;
    bCloseThread = false;
    
    static char buffer[BUFFER_SIZE];
    
    /* Read packets */
    
    while (!bCloseThread)
    {
        if(mSerial.getNumBytesAvailable() > 0)
        {
            /* Read data */
            size_t bytesRead = lineread(buffer, BUFFER_SIZE);
            
            if (bytesRead == (size_t) - 1)
            {
                bytesRead = slipread(buffer, BUFFER_SIZE);
            }
            if (bytesRead == 0) { break; }
            
            
            // Get time now
            unsigned long long now = ticksNow();
            
            // If it appears to be a binary WAX packet...
            if (bytesRead > 1 && buffer[0] == 0x12 && (buffer[1] == 0x78 || buffer[1] == 0x58))
            {
                WaxPacket *waxPacket = parseWaxPacket(buffer, bytesRead, now);
                if (waxPacket != NULL)
                {
                    if(bDebug) printWax(waxPacket, 1);
                    
                    // Save samples in concurrent buffer
                    for (int i = 0; i < waxPacket->sampleCount; i++)
                    {
                        if(mBuffers.count(waxPacket->deviceId) == 0){ // can't we do this automatically? maybe with default allocator?
                            mBuffers[waxPacket->deviceId] = new ConcurrentCircularBuffer<WaxSample>( 50 );
                        }
                        mBuffers.at(waxPacket->deviceId)->tryPushFront(waxPacket->samples[i]);
                    }
                }
            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // need this or cpu consumption goes up to 115%
        }
    }
    return 1;
}

/* -------------------------------------------------------------------------------------------------- */
#pragma mark packet parsing
/* -------------------------------------------------------------------------------------------------- */

#define SLIP_END     0xC0                   /* End of packet indicator */
#define SLIP_ESC     0xDB                   /* Escape character, next character will be a substitution */
#define SLIP_ESC_END 0xDC                   /* Escaped substitution for the END data byte */
#define SLIP_ESC_ESC 0xDD                   /* Escaped substitution for the ESC data byte */

/* Read a line from the device */
size_t Wax3Receiver::lineread(void *inBuffer, size_t len)
{
    unsigned char *p = (unsigned char *)inBuffer;
    unsigned char c;
    size_t bytesRead = 0;
    
    if (inBuffer == NULL) { return 0; }
    *p = '\0';
    while(!bCloseThread)
    {
        c = '\0';
        
        try{
            c = mSerial.readByte();
        }
        catch(...) {
            return bytesRead;
        }
        
        if (c == SLIP_END) { /* A SLIP_END means the reader should switch to slip reading. */
            return (size_t)-1;
        }
        if (c == '\r' || c == '\n')
        {
            if (bytesRead) return bytesRead;
        }
        else
        {
            if (bytesRead < len - 1) {
                p[bytesRead++] = (char)c;
                p[bytesRead] = 0;
            }
        }
    }
    return 0;
}

/* Read a SLIP-encoded packet from the device */
size_t Wax3Receiver::slipread(void *inBuffer, size_t len)
{
    unsigned char *p = (unsigned char *)inBuffer;
    unsigned char c = '\0';
    size_t bytesRead = 0;
    
    if (inBuffer == NULL) return 0;
    
    while(!bCloseThread)
    {
        c = '\0';
        
        try{
            c = mSerial.readByte();
        }
        catch(...) {
            return bytesRead;
        }
        switch (c)
        {
            case SLIP_END:
                if (bytesRead) return bytesRead;
                break;
                
            case SLIP_ESC:
                c = '\0';
                
                try{
                    c = mSerial.readByte();
                }
                catch(...) {
                    return bytesRead;
                }
                
                switch (c){
                    case SLIP_ESC_END:
                        c = SLIP_END;
                        break;
                    case SLIP_ESC_ESC:
                        c = SLIP_ESC;
                        break;
                    default:
                        fprintf(stderr, "<Unexpected escaped value: %02x>", c);
                        break;
                }
                
                /* ... fall through to default case with our replaced character ... */
            default:
                if (bytesRead < len) {
                    p[bytesRead++] = c;
                }
                break;
        }
    }
    return 0;
}

WaxPacket* Wax3Receiver::parseWaxPacket(const void *inputBuffer, size_t len, unsigned long long now)
{
    const unsigned char *buffer = (const unsigned char *)inputBuffer;
    static WaxPacket waxPacket;
    
    if (buffer == NULL || len <= 0) { return 0; }
    
    if (len >= 12 && buffer[0] == 0x12 && buffer[1] == 0x78)
    {
        
        unsigned short deviceId = buffer[2] | ((unsigned short)buffer[3] << 8);
        unsigned char format = buffer[7];
        unsigned short sequenceId = buffer[8] | ((unsigned short)buffer[9] << 8);
        unsigned char outstanding = buffer[10];
        unsigned char sampleCount = buffer[11];
        int bytesPerSample;
        size_t expectedLength;
        
        /* Format: 0xE0 | AccelCurrentRate() */
        /* [1] = Accelerometer data format 0xEA (3-axis, 2-bytes, 100Hz); Top two bits is number of axes (3), next two bits is format [1=unsigned 8-bit,2=signed 16-bit] (2); lowest four bits is the rate code: frequency = 3200 / (1 << (15-(n & 0x0f))) */
        bytesPerSample = 0;
        if (((format >> 4) & 0x03) == 2) { bytesPerSample = 6; }    /* 3*16-bit */
        else if (((format >> 4) & 0x03) == 0) { bytesPerSample = 4; }    /* 3*10-bit + 2 */
        
        expectedLength = 12 + sampleCount * bytesPerSample;
        if (len < expectedLength) { fprintf(stderr, "WARNING: Ignoring truncated- or unknown-format data packet (received %d expected %d).", (int)len, (int)expectedLength); }
        else
        {
            int i;
            if (len > expectedLength) { fprintf(stderr, "WARNING: Data packet was larger than expected, ignoring additional samples"); }
            
            waxPacket.timestamp = now;
            waxPacket.deviceId = deviceId;
            waxPacket.sequenceId = sequenceId;
            waxPacket.sampleCount = sampleCount;
            
            for (i = 0; i < sampleCount; i++)
            {
                int frequency;
                short millisecondsAgo;
                short x = 0, y = 0, z = 0;
                
                if (bytesPerSample == 6)
                {
                    x = (short)((unsigned short)(buffer[12 + i * 6] | (((unsigned short)buffer[13 + i * 6]) << 8)));
                    y = (short)((unsigned short)(buffer[14 + i * 6] | (((unsigned short)buffer[15 + i * 6]) << 8)));
                    z = (short)((unsigned short)(buffer[16 + i * 6] | (((unsigned short)buffer[17 + i * 6]) << 8)));
                }
                else if (bytesPerSample == 4)
                {
                    /* Packed accelerometer value
                     [byte-3] [byte-2] [byte-1] [byte-0]
                     eezzzzzz zzzzyyyy yyyyyyxx xxxxxxxx
                     10987654 32109876 54321098 76543210
                     Must sign-extend 10-bit value, adjust for exponent
                     */
                    unsigned int value = (unsigned int)buffer[12 + i * 4] | ((unsigned int)buffer[13 + i * 4] << 8) | ((unsigned int)buffer[14 + i * 4] << 16) | ((unsigned int)buffer[15 + i * 4] << 24);
                    
                    x = (short)( (short)((unsigned short)0xffc0 & (unsigned short)(value <<  6)) >> (6 - ((unsigned char)(value >> 30))) );
                    y = (short)( (short)((unsigned short)0xffc0 & (unsigned short)(value >>  4)) >> (6 - ((unsigned char)(value >> 30))) );
                    z = (short)( (short)((unsigned short)0xffc0 & (unsigned short)(value >> 14)) >> (6 - ((unsigned char)(value >> 30))) );
                }
                frequency = 3200 / ((unsigned short)1 << (15 - (format & 0x0f)));
                millisecondsAgo = !frequency ? 0 : (short)((sampleCount + outstanding - 1 - i) * 1000L / frequency);
                
                waxPacket.samples[i].timestamp = now - millisecondsAgo;
                waxPacket.samples[i].sampleIndex = sequenceId + i;
                waxPacket.samples[i].x = x;
                waxPacket.samples[i].y = y;
                waxPacket.samples[i].z = z;
            }
            return &waxPacket;
        }
    }
    else if (len >= 12 && buffer[0] == 0x12 && buffer[1] == 0x58)
    {
        fprintf(stderr, "WARNING: Received old WAX packet format -- ignoring.\n");
    }
    else
    {
        fprintf(stderr, "WARNING: Unrecognized packet -- ignoring.\n");
    }
    return NULL;
}


/* -------------------------------------------------------------------------------------------------- */
#pragma mark utils
/* -------------------------------------------------------------------------------------------------- */

void Wax3Receiver::printWax(WaxPacket *waxPacket, int timeformat)
{
    for (int i = 0; i < waxPacket->sampleCount; i++)
    {
        const char *timeString = timestamp(waxPacket->samples[i].timestamp);
        
        printf("ACCEL %s \ndevice:%u \nsample:%u \n%f,%f,%f\n\n", timeString, waxPacket->deviceId, waxPacket->samples[i].sampleIndex, waxPacket->samples[i].x / 256.0f, waxPacket->samples[i].y / 256.0f, waxPacket->samples[i].z / 256.0f);
    }
}


/* Returns the number of milliseconds since the epoch */
unsigned long long Wax3Receiver::ticksNow(void)
{
    struct timeb tp;
    ftime(&tp);
    return (unsigned long long)tp.time * 1000 + tp.millitm;
}

/* Returns a date/time string for the specific number of milliseconds since the epoch */
const char* Wax3Receiver::timestamp(unsigned long long ticks)
{
    static char output[] = "YYYY-MM-DD HH:MM:SS.fff";
    output[0] = '\0';
    
    struct tm *today;
    struct timeb tp = {0};
    tp.time = (time_t)(ticks / 1000);
    tp.millitm = (unsigned short)(ticks % 1000);
    tzset();
    today = localtime(&(tp.time));
    if (strlen(output) != 0) { strcat(output, ","); }
    sprintf(output + strlen(output), "%04d-%02d-%02d %02d:%02d:%02d.%03d", 1900 + today->tm_year, today->tm_mon + 1, today->tm_mday, today->tm_hour, today->tm_min, today->tm_sec, tp.millitm);
    
    return output;
}
