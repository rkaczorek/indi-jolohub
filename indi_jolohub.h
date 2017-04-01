/*******************************************************************************
 Copyright(c) 2016 Radek Kaczorek  <rkaczorek AT gmail DOT com>
 .
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#ifndef ASTROHUB_H
#define ASTROHUB_H

#include <string>
#include <iostream>
#include <stdio.h>

#include <defaultdevice.h>
#include <connectionplugins/connectionserial.h>

class IndiAstrohub : public INDI::DefaultDevice
{
protected:
private:
	int counter;
	ISwitch Power1S[2];
	ISwitchVectorProperty Power1SP;
	ISwitch Power2S[2];
	ISwitchVectorProperty Power2SP;
	ISwitch Power3S[2];
	ISwitchVectorProperty Power3SP;
	ISwitch Power4S[2];
	ISwitchVectorProperty Power4SP;
	ISwitch Focus1MotionS[2];
	ISwitchVectorProperty Focus1MotionSP;
	INumber Focus1StepN[1];
	INumberVectorProperty Focus1StepNP;
	INumber Focus1AbsPosN[1];
	INumberVectorProperty Focus1AbsPosNP;
	ISwitch Focus2MotionS[2];
	ISwitchVectorProperty Focus2MotionSP;
	INumber Focus2StepN[1];
	INumberVectorProperty Focus2StepNP;
	INumber Focus2AbsPosN[1];
	INumberVectorProperty Focus2AbsPosNP;
	ISwitch Focus3MotionS[2];
	ISwitchVectorProperty Focus3MotionSP;
	INumber Focus3SpeedN[1];
	INumberVectorProperty Focus3SpeedNP;
	INumber Focus3StepN[1];
	INumberVectorProperty Focus3StepNP;
	INumber Sensor1N[3];
	INumberVectorProperty Sensor1NP;
	INumber Sensor2N[3];
	INumberVectorProperty Sensor2NP;
	INumber Sensor3N[1];
	INumberVectorProperty Sensor3NP;
	INumber PWM1N[1];
	INumberVectorProperty PWM1NP;
	INumber PWM2N[1];
	INumberVectorProperty PWM2NP;
	INumber PWM3N[1];
	INumberVectorProperty PWM3NP;
	INumber PWM4N[1];
	INumberVectorProperty PWM4NP;
public:
    IndiAstrohub();
	virtual ~IndiAstrohub();

	virtual const char *getDefaultName();

	virtual void TimerHit();
	virtual bool Handshake();
	virtual bool initProperties();
	virtual bool updateProperties();
	virtual void ISGetProperties(const char *dev);
	virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
	virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
	virtual bool ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n);
	virtual bool ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n);
	virtual bool ISSnoopDevice(XMLEle *root);
	virtual bool saveConfigItems(FILE *fp);
	int PortFD=-1;
	Connection::Serial *serialConnection=NULL;
	virtual char* serialCom(const char* input);
};

#endif
