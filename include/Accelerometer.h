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

#include <boost/circular_buffer.hpp>
#include "cinder/app/App.h"
#include "AccelDataSource.h"

using namespace ci;
using namespace std;

const static int kAccelHistoryLength = 120;

class Accelerometer
{
    
public:
    Accelerometer();
    ~Accelerometer(){};
    
    void        setup(ushort id, AccelDataSource* dataSource, int historyLength = 120);
    void        update();
    void        setSmooth(bool s, float f = 0.2f)   {mSmooth = s; mSmoothFactor = f;}
    
    bool        isActive()                          {return mActive;}
    bool        hasNewReadings()                    {return mNewReadings>0;}
    int         getNumNewReadings()                 {return mNewReadings;}
    ushort      getId()                             {return mId;}

    Vec3f       getAccel()                          {return mAccels->front();}
    Vec3f       getAccel(int i)                     {return mAccels->at(i);}
    Vec3f*      getAccelHistory()                   {return mAccels->linearize();}
    float       getAccelMagnitude()                 {return mAccelMags->front();}
    float       getAccelMagnitude(int i)            {return mAccelMags->at(i);}
    float*      getAccelMagHistory()                {return mAccelMags->linearize();}
    Vec3f       getMaxAccel()                       {return mMaxAccel;}
    float       getMaxAccelMagnitude()              {return mMaxAccelMag;}
    int         getHistoryLength()                  {return mAccels->size();}
    
    float       getPitch(); // rotation in x axis in degrees
    float       getRoll(); // rotation in z axis in degrees

    
protected:
    ushort      mId;
    bool        mActive;
    bool        mSmooth;
    float       mSmoothFactor;
    int         mNewReadings;
    
    boost::circular_buffer<float>* mAccelMags;
    boost::circular_buffer<Vec3f>* mAccels;
    
    Vec3f       mMaxAccel;
    float       mMaxAccelMag;
    
    AccelDataSource* mDataSource;
};