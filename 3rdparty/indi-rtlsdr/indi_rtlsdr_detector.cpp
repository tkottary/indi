/*
    indi_rtlsdr_detector - a software defined radio driver for INDI
    Copyright (C) 2017  Ilia Platone

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "indi_rtlsdr_detector.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <indilogger.h>
#include <libdspau.h>
#include <memory>

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })
#define MAX_TRIES 20
#define MAX_DEVICES 4
#define SUBFRAME_SIZE (16384)
#define MIN_FRAME_SIZE (512)
#define MAX_FRAME_SIZE (SUBFRAME_SIZE * 16)
#define SPECTRUM_SIZE 256

static int iNumofConnectedDetectors;
static RTLSDR *receivers[MAX_DEVICES];

static void cleanup()
{
    for (int i = 0; i < iNumofConnectedDetectors; i++)
    {
        delete receivers[i];
    }
}

static void callback(unsigned char *buf, uint32_t len, void *ctx)
{
    RTLSDR *receiver = (RTLSDR*)ctx;
    receiver->grabData(buf, len);
}

void ISInit()
{
    static bool isInit = false;
    if (!isInit)
    {
        iNumofConnectedDetectors = 0;

        iNumofConnectedDetectors = rtlsdr_get_device_count();
        if (iNumofConnectedDetectors > MAX_DEVICES)
            iNumofConnectedDetectors = MAX_DEVICES;
        if (iNumofConnectedDetectors <= 0)
        {
            //Try sending IDMessage as well?
            IDLog("No RTLSDR receivers detected. Power on?");
            IDMessage(nullptr, "No RTLSDR receivers detected. Power on?");
        }
        else
        {
            for (int i = 0; i < iNumofConnectedDetectors; i++)
            {
                receivers[i] = new RTLSDR(i);
            }
        }

        atexit(cleanup);
        isInit = true;
    }
}

void ISGetProperties(const char *dev)
{
    ISInit();

    if (iNumofConnectedDetectors == 0)
    {
        IDMessage(nullptr, "No RTLSDR receivers detected. Power on?");
        return;
    }

    for (int i = 0; i < iNumofConnectedDetectors; i++)
    {
        RTLSDR *receiver = receivers[i];
        if (dev == nullptr || !strcmp(dev, receiver->getDeviceName()))
        {
            receiver->ISGetProperties(dev);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    ISInit();
    for (int i = 0; i < iNumofConnectedDetectors; i++)
    {
        RTLSDR *receiver = receivers[i];
        if (dev == nullptr || !strcmp(dev, receiver->getDeviceName()))
        {
            receiver->ISNewSwitch(dev, name, states, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < iNumofConnectedDetectors; i++)
    {
        RTLSDR *receiver = receivers[i];
        if (dev == nullptr || !strcmp(dev, receiver->getDeviceName()))
        {
            receiver->ISNewText(dev, name, texts, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    ISInit();
    for (int i = 0; i < iNumofConnectedDetectors; i++)
    {
        RTLSDR *receiver = receivers[i];
        if (dev == nullptr || !strcmp(dev, receiver->getDeviceName()))
        {
            receiver->ISNewNumber(dev, name, values, names, num);
            if (dev != nullptr)
                break;
        }
    }
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle *root)
{
    ISInit();

    for (int i = 0; i < iNumofConnectedDetectors; i++)
    {
        RTLSDR *receiver = receivers[i];
        receiver->ISSnoopDevice(root);
    }
}

RTLSDR::RTLSDR(uint32_t index)
{
	InCapture = false;

    detectorIndex = index;

    char name[MAXINDIDEVICE];
    snprintf(name, MAXINDIDEVICE, "%s %d", getDefaultName(), index);
    setDeviceName(name);
	continuum = (uint8_t*)malloc(sizeof(uint8_t));
    spectrum = (uint8_t *)malloc(sizeof(uint8_t));
}

/**************************************************************************************
** Client is asking us to establish connection to the device
***************************************************************************************/
bool RTLSDR::Connect()
{    	
    int r = rtlsdr_open(&rtl_dev, detectorIndex);
    if (r < 0)
    {
        LOGF_ERROR("Failed to open rtlsdr device index %d.", detectorIndex);
		return false;
	}

    LOG_INFO("RTL-SDR Detector connected successfully!");
	// Let's set a timer that checks teleDetectors status every POLLMS milliseconds.
    // JM 2017-07-31 SetTimer already called in updateProperties(). Just call it once
    //SetTimer(POLLMS);


	return true;
}

/**************************************************************************************
** Client is asking us to terminate connection to the device
***************************************************************************************/
bool RTLSDR::Disconnect()
{
	InCapture = false;
	rtlsdr_close(rtl_dev);
	free(continuum);
	free(spectrum);
	LOG_INFO("RTL-SDR Detector disconnected successfully!");
	return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *RTLSDR::getDefaultName()
{
	return "RTL-SDR Receiver";
}

/**************************************************************************************
** INDI is asking us to init our properties.
***************************************************************************************/
bool RTLSDR::initProperties()
{
	// Must init parent properties first!
	INDI::Detector::initProperties();

	// We set the Detector capabilities
	uint32_t cap = DETECTOR_CAN_ABORT | DETECTOR_HAS_CONTINUUM | DETECTOR_HAS_SPECTRUM;
	SetDetectorCapability(cap);

    PrimaryDetector.setMinMaxStep("DETECTOR_CAPTURE", "DETECTOR_CAPTURE_VALUE", 0.001, 10, 1, false);
	PrimaryDetector.setMinMaxStep("DETECTOR_SETTINGS", "DETECTOR_FREQUENCY", 2.4e+7, 2.0e+9, 1, false);
	PrimaryDetector.setMinMaxStep("DETECTOR_SETTINGS", "DETECTOR_SAMPLERATE", 1, 1, 0, false);
	PrimaryDetector.setMinMaxStep("DETECTOR_SETTINGS", "DETECTOR_BITSPERSAMPLE", 8, 8, 1, false);
    PrimaryDetector.setCaptureExtension("fits");

	// Add Debug, Simulator, and Configuration controls
	addAuxControls();

    setDefaultPollingPeriod(500);

	return true;
}

/********************************************************************************************
** INDI is asking us to update the properties because there is a change in CONNECTION status
** This fucntion is called whenever the device is connected or disconnected.
*********************************************************************************************/
bool RTLSDR::updateProperties()
{
	// Call parent update properties first
	INDI::Detector::updateProperties();

	if (isConnected())
	{
		// Let's get parameters now from Detector
		setupParams();

		// Start the timer
		SetTimer(POLLMS);
	}

	return true;
}

/**************************************************************************************
** Setting up Detector parameters
***************************************************************************************/
void RTLSDR::setupParams()
{
	// Our Detector is an 8 bit Detector, 100MHz frequency 1MHz bandwidth.
    SetDetectorParams(2000000.0, 100000000.0, 8, 10000.0, 100.0);
}

/**************************************************************************************
** Client is asking us to start an exposure
***************************************************************************************/
bool RTLSDR::StartCapture(float duration)
{
	CaptureRequest = duration;

	// Since we have only have one Detector with one chip, we set the exposure duration of the primary Detector
    PrimaryDetector.setCaptureDuration(duration);
    b_read = 0;
    to_read = PrimaryDetector.getSampleRate() * PrimaryDetector.getCaptureDuration();

    PrimaryDetector.setContinuumBufferSize(to_read);
    PrimaryDetector.setSpectrumBufferSize(SPECTRUM_SIZE);

    if(to_read > 0) {
        rtlsdr_reset_buffer(rtl_dev);
        gettimeofday(&CapStart, nullptr);
        InCapture = true;
        rtlsdr_read_async(rtl_dev, callback, this, 1, min(MAX_FRAME_SIZE, to_read));
        return true;
    }

	// We're done
    return false;
}

/**************************************************************************************
** Client is updating capture settings
***************************************************************************************/
bool RTLSDR::CaptureParamsUpdated(float sr, float freq, float bps, float bw, float gain)
{
    INDI_UNUSED(bw);
    INDI_UNUSED(bps);
	int r = 0;

    r |= rtlsdr_set_agc_mode(rtl_dev, 0);
    r |= rtlsdr_set_tuner_gain_mode(rtl_dev, 1);

    r |= rtlsdr_set_tuner_gain(rtl_dev, (int)(gain * 10));
    //r |= rtlsdr_set_tuner_bandwidth(rtl_dev, (uint32_t)bw);
    r |= rtlsdr_set_center_freq(rtl_dev, (uint32_t)freq);
    r |= rtlsdr_set_sample_rate(rtl_dev, (uint32_t)sr);

    if(r != 0) {
		return false;
    }

    return true;
}

/**************************************************************************************
** Client is asking us to abort a capture
***************************************************************************************/
bool RTLSDR::AbortCapture()
{
	InCapture = false;
	return true;
}

/**************************************************************************************
** How much longer until exposure is done?
***************************************************************************************/
float RTLSDR::CalcTimeLeft()
{
	double timesince;
	double timeleft;
	struct timeval now;
	gettimeofday(&now, nullptr);

	timesince = (double)(now.tv_sec * 1000.0 + now.tv_usec / 1000) -
				(double)(CapStart.tv_sec * 1000.0 + CapStart.tv_usec / 1000);
	timesince = timesince / 1000;

	timeleft = CaptureRequest - timesince;
	return timeleft;
}

/**************************************************************************************
** Main device loop. We check for capture progress here
***************************************************************************************/
void RTLSDR::TimerHit()
{
	long timeleft;

	if (isConnected() == false)
		return; //  No need to reset timer if we are not connected anymore

	if (InCapture)
	{
		timeleft = CalcTimeLeft();
		if(timeleft < 0.1)
		{
			/* We're done capturing */
            LOG_INFO("Capture done, expecting data...");
			timeleft = 0.0;
		}

		// This is an over simplified timing method, check DetectorSimulator and rtlsdrDetector for better timing checks
		PrimaryDetector.setCaptureLeft(timeleft);
	}

	SetTimer(POLLMS);
	return;
}

/**************************************************************************************
** Create the spectrum
***************************************************************************************/
void RTLSDR::grabData(unsigned char *buf, int n_read)
{
    if(InCapture) {
        n_read = min(to_read, n_read);
        continuum = PrimaryDetector.getContinuumBuffer();
        spectrum = PrimaryDetector.getSpectrumBuffer();
        if(to_read > 0) {
            memcpy(continuum + b_read, buf, n_read);
            b_read += n_read;
            to_read -= n_read;
        }

        LOG_INFO("Downloading...");

        if(to_read <= 0) {
            InCapture = false;
            rtlsdr_cancel_async(rtl_dev);

            //Create the dspau stream
            dspau_stream_p stream = dspau_stream_new();
            dspau_stream_add_dim(stream, b_read);
            dspau_convert_from(continuum, stream->in, unsigned char, b_read);

            //Create the spectrum
            stream->out = dspau_fft_spectrum(stream, magnitude_root, SPECTRUM_SIZE);
            dspau_convert_to(stream->out, spectrum, unsigned char, SPECTRUM_SIZE);

            //Destroy the dspau stream
            dspau_stream_free(stream);

            LOG_INFO("Download complete.");
            CaptureComplete(&PrimaryDetector);
        }
    }
}
