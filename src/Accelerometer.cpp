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

#include "Accelerometer.h"
#include "cinder/CinderMath.h"

Accelerometer::Accelerometer()
{
    mActive = false;
    mSmooth  = false;
    mSmoothFactor = 0.8;
    mNewReadings = 0;

    mMaxAccel = Vec3f::zero();
    mMaxAccelMag = 0;
}

void Accelerometer::setup(unsigned short id, AccelDataSource* dataSource, int historyLength)
{
    mId = id;
    mDataSource = dataSource;
    mHistoryLength = historyLength;
    
    mAccels     = new boost::circular_buffer<Vec3f>(historyLength);
    mAccelMags  = new boost::circular_buffer<float>(historyLength);    
}

void Accelerometer::update()
{
    mNewReadings = 0;

    // Obtain new data from dataSource
    while (mDataSource->hasNewReadings(mId)) {
        mActive = true;
        mNewReadings ++;
        
        // Update accel buffer
        Vec3f newAccel = mDataSource->getNextReading(mId);
        
        if(mSmooth && !mAccels->empty()){
            newAccel = newAccel * mSmoothFactor + (1 - mSmoothFactor) * mAccels->front();
        }
        mAccels->push_front(newAccel);
        mAccelMags->push_front(newAccel.length());  // try lengthSquared?
        
        // Save max and min for every component
        for(int i=0; i<3; i++){
            mMaxAccel[i] = max(mMaxAccel[i], newAccel[i]);
        }
        mMaxAccelMag = max(mMaxAccelMag, mAccelMags->front());
    }
    
    // sometimes after sleeping for a while we get a lot of readings in the same packet
    // we can't let numNewReadings be higher than the size of the circular buffers
    if (mNewReadings > mHistoryLength) mNewReadings = mHistoryLength;
}

float Accelerometer::getPitch()
{
    Vec3f acc = getAccel();
    return (atan2(acc.x, sqrt(acc.y*acc.y + acc.z*acc.z))*180.0)/M_PI;
}

float Accelerometer::getRoll()
{
    Vec3f acc = getAccel();
    return (atan2(-acc.y, acc.z)*180.0)/M_PI;
}

