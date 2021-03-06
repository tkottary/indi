#include "skywatcherazgtimount.h"
#include "indicom.h"
#include "connectionplugins/connectiontcp.h"
#include <libnova/transform.h>
#include <libnova/sidereal_time.h>
#include <libnova/utility.h>
//#include "mach_gettime.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <termios.h>
#include <cmath>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netdb.h>
#include <alignment/DriverCommon.h> // For DBG_ALIGNMENT
#include "alignment/AlignmentSubsystemForDrivers.h"

// libnova specifies round() on old systems and it collides with the new gcc 5.x/6.x headers
#define HAVE_ROUND
/* Preset Slew Speeds */
#define SLEWMODES 9
double SlewSpeeds[SLEWMODES] = { 1.0, 2.0, 8.0, 16.0, 32.0, 200.0, 400.0, 600.0,800.0 };
const int MSG_SIZE = 100;
void ISPoll(void *p);

using namespace INDI::AlignmentSubsystem;

// We declare an auto pointer to Synscan.
static std::unique_ptr<SkywatcherAZGTIMount> skywatcherAZGTIMount(new SkywatcherAZGTIMount());

namespace
{

std::string GetLogTimestamp()
{
    time_t Now = time(nullptr);
    struct tm TimeStruct;
    char Buffer[60];
    std::string FinalStr;

    TimeStruct = *localtime(&Now);
    strftime(Buffer, sizeof(Buffer), "%Y%m%d %H:%M:%S", &TimeStruct);
    FinalStr = Buffer;
    // Add the millisecond part
    std::chrono::system_clock::time_point NowClock = std::chrono::system_clock::now();
    std::chrono::system_clock::duration TimePassed = NowClock.time_since_epoch();

    TimePassed -= std::chrono::duration_cast<std::chrono::seconds>(TimePassed);
    FinalStr += "."+std::to_string(static_cast<unsigned>(TimePassed / std::chrono::milliseconds(1)));
    return FinalStr;
}
}

void ISGetProperties(const char *dev)
{
    skywatcherAZGTIMount->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    skywatcherAZGTIMount->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    skywatcherAZGTIMount->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    skywatcherAZGTIMount->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    skywatcherAZGTIMount->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISSnoopDevice(XMLEle *root)
{
    INDI_UNUSED(root);
}

SkywatcherAZGTIMount::SkywatcherAZGTIMount()
{
    // Set up the logging pointer in SkyWatcherAPI
    pChildTelescope  = this;
    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT |TELESCOPE_HAS_TRACK_MODE | TELESCOPE_CAN_CONTROL_TRACK
                               |TELESCOPE_HAS_TIME |TELESCOPE_HAS_LOCATION|TELESCOPE_HAS_PIER_SIDE,
                           SLEWMODES);
    //currentRA  = ln_get_apparent_sidereal_time(ln_get_julian_from_sys());
    currentRA=0;
    currentDEC = 90;
}

bool SkywatcherAZGTIMount::Abort()
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::Abort");
    LogMessage("MOVE ABORT");
    SlowStop(AXIS1);
    SlowStop(AXIS2);
    TrackState = SCOPE_IDLE;

    if (GuideNSNP.s == IPS_BUSY || GuideWENP.s == IPS_BUSY)
    {
        GuideNSNP.s = GuideWENP.s = IPS_IDLE;
        GuideNSN[0].value = GuideNSN[1].value = 0.0;
        GuideWEN[0].value = GuideWEN[1].value = 0.0;

        IDMessage(getDeviceName(), "Guide aborted.");
        IDSetNumber(&GuideNSNP, nullptr);
        IDSetNumber(&GuideWENP, nullptr);

        return true;
    }

    return true;
}

bool SkywatcherAZGTIMount::Handshake()
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::Handshake");
    return true;
}


bool SkywatcherAZGTIMount::Connect()
{
    LOG_DEBUG("SkywatcherAZGTIMount::Connect");

    addAuxControls();

    if (isConnected())
        return true;

    bool Ret = INDI::Telescope::Connect();

    if (Ret && getActiveConnection()->type() == Connection::Interface::CONNECTION_TCP)
    {


        //Add check if its UDP. Do we need to , since this works only in UDP now ?

        tty_set_skywatcher_udp_format(2);
        PortFD = tcpConnection->getPortFD();
        SetSerialPort(tcpConnection->getPortFD());

       if(!detectScope())
           return false;

        bool initMountFromEQ =  InitMount(RecoverAfterReconnection);
        DEBUGF(DBG_SCOPE, "SkywatcherAZGTIMount initMount - Result: %d", initMountFromEQ);

        if( MountCode>0 && (IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON))
            startTracking();
}
    return Ret;
}


bool SkywatcherAZGTIMount::detectScope(){

    int fd=-1;	/* our socket */
    //int sockbufsize = 2048;

    int status;
    struct addrinfo hints;
    //struct addrinfo *servinfo;  // will point to the results

    timeval tv;
    tv.tv_sec  = 2;
    tv.tv_usec = 0;

    /* fill in the server's address and data */


    memset(&hints, 0, sizeof hints); // make sure the struct is empty
    hints.ai_family = AF_UNSPEC;     // don't care IPv4 or IPv6
    hints.ai_socktype = SOCK_DGRAM; // TCP stream sockets
    hints.ai_flags = AI_PASSIVE;     // fill in my IP for me
    hints.ai_protocol = IPPROTO_UDP;

    //fix this hardcoding.

    if ((status = getaddrinfo(tcpConnection->host(),tcpConnection->portStr(), &hints, &serverInfo)) != 0) {
        fprintf(stderr, "getaddrinfo error: %s\n", gai_strerror(status));
        return false;
        //exit(1);
    }

    fd = socket(serverInfo->ai_family, serverInfo->ai_socktype, serverInfo->ai_protocol);
    setUdpFd(fd);

   int bindValue= bind(fd, serverInfo->ai_addr, serverInfo->ai_addrlen);
//        if (bindValue<0)
//           return false;

    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval))<0){
         DEBUG(DBG_SCOPE, "did not SO_RCVTIMEO ");
    };

    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(struct timeval))<0){
         DEBUG(DBG_SCOPE, "did not SO_SNDTIMEO ");
    };

//    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF,(char *)&sockbufsize,  (int)sizeof(sockbufsize))<0){
//         DEBUG(DBG_SCOPE, "did not SO_RCVBUF ");
//    };
//    if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF,(char *)&sockbufsize,  (int)sizeof(sockbufsize))<0){
//         DEBUG(DBG_SCOPE, "did not SO_SNDBUF ");
//    };
        return true;
}
const char *SkywatcherAZGTIMount::getDefaultName()
{
    //DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::getDefaultName\n");
    return "Skywatcher AZ-GTI";
}
bool SkywatcherAZGTIMount::EQGoto(double ra,double dec){

    double juliandate=0;
    double lst=0;

    unsigned long TargetRaStep, TargetDeStep, MaxStep;

    //double datevalues[2];
    char hrlst[12];

    unsigned long zeroRAEncoder=ZeroPositionEncoders[AXIS1];//8388608
    unsigned long totalRAEncoder=MicrostepsPerRevolution[AXIS1];
    unsigned long eastLimit =7941273;//7902228
    unsigned long westLimit=8890259;//8922056

    juliandate = getJulianDate();
    lst        = getLst(juliandate, getLongitude());
    double HA   = rangeHA(ra - lst);
    fs_sexa(hrlst, lst, 2, 360000);
    hrlst[11] = '\0';
    DEBUGF(DBG_SCOPE, "Compute local time: lst=%2.8f (%s) - julian date=%8.8f", lst, hrlst, juliandate);


      if ((TrackState == SCOPE_SLEWING) || (TrackState == SCOPE_PARKING) || (TrackState == SCOPE_PARKED)){
            LOG_WARN("Can not perform goto while goto/park in progress, or scope parked.");
            EqNP.s = IPS_IDLE;
            IDSetNumber(&EqNP, nullptr);
            return true;
        }



        //LOGF_INFO("Starting Goto RA=%g DE=%g (current RA=%g DE=%g)", ra, dec, currentRA, currentDEC);
        targetRA  = ra;
        targetDEC = dec;

        GetEncoder(AXIS1);
        GetEncoder(AXIS2);
        EncoderValuesfromHAanDEC(HA, dec, TargetRaStep, TargetDeStep);

//        // Detemine max step in either RA or DEC - used to calculate time of slew to get more accurate RA position
            if (abs(TargetRaStep - CurrentEncoders[AXIS1]) > abs(TargetDeStep - CurrentEncoders[AXIS2])) {
                MaxStep = abs(TargetRaStep - CurrentEncoders[AXIS1]);
            }
            else {
                MaxStep = abs(TargetDeStep - CurrentEncoders[AXIS2]);
            }
            StartTargetSlew(AXIS1, CurrentEncoders[AXIS1], TargetRaStep, MicrostepsPerRevolution[AXIS1], MaxStep);
            StartTargetSlew(AXIS2, CurrentEncoders[AXIS2], TargetDeStep, MicrostepsPerRevolution[AXIS2], 0);


                long AltitudeOffsetMicrosteps=  TargetDeStep-CurrentEncoders[AXIS2];
                long AzimuthOffsetMicrosteps= TargetRaStep-CurrentEncoders[AXIS1];

                // Do I need to take out any complete revolutions before I do this test?
            if (AltitudeOffsetMicrosteps > MicrostepsPerRevolution[AXIS2] / 2)
            {
                // Going the long way round - send it the other way
                AltitudeOffsetMicrosteps -= MicrostepsPerRevolution[AXIS2];
            }

            if (AzimuthOffsetMicrosteps > MicrostepsPerRevolution[AXIS1] / 2)
            {
                // Going the long way round - send it the other way
                AzimuthOffsetMicrosteps -= MicrostepsPerRevolution[AXIS1];
            }

            SlewTo(AXIS1, AzimuthOffsetMicrosteps);
            SlewTo(AXIS2, AltitudeOffsetMicrosteps);

            LOGF_INFO("AzimuthOffsetMicrosteps AltitudeOffsetMicrosteps   : DE increment = %ld, RA increment = %ld",AltitudeOffsetMicrosteps,AzimuthOffsetMicrosteps);
            TrackState         = SCOPE_SLEWING;
            EqNP.s = IPS_BUSY;
}
long SkywatcherAZGTIMount::abs(long value)
{
    if (value > 0.0) {
        return value;
    } else {
        return -value;
    }
}
void SkywatcherAZGTIMount::EncoderValuesfromHAanDEC(double dHa, double dDec, unsigned long &RAEncoder, unsigned long &DEEncoder)
{

    long RAStepInit= ZeroPositionEncoders[AXIS1];
    long DEStepInit =ZeroPositionEncoders[AXIS2];
    long RASteps360= MicrostepsPerRevolution[AXIS1];
    long DESteps360= MicrostepsPerRevolution[AXIS1];

    if (Hemisphere ==NORTH) {
        if (dHa < 0) {	 //	 Pre-Meridian
            DEEncoder = (int)(DEStepInit + (dDec - 90.0)*DESteps360 / 360.0);
            RAEncoder = (int)(RAStepInit + (dHa + 6.0)*RASteps360 / 24.0);
        }
        else {	// Post-Meridian
            DEEncoder = (int)(DEStepInit - (dDec - 90.0)*DESteps360 / 360.0);
            RAEncoder = (int)(RAStepInit + (dHa - 6.0)*RASteps360 / 24.0);
        }
    }
    else {
        if (dHa < 0) {	 //	 Pre-Meridian
            DEEncoder = (int)(DEStepInit + (dDec + 90.0)*DESteps360 / 360.0);
            RAEncoder = (int)(RAStepInit - (dHa + 6.0)*RASteps360 / 24.0);
        }
        else {	// Post-Meridian
            DEEncoder = (int)(DEStepInit - (dDec + 90.0)*DESteps360 / 360.0);
            RAEncoder = (int)(RAStepInit - (dHa - 6.0)*RASteps360 / 24.0);
        }
    }
}

void SkywatcherAZGTIMount::HAandDECfromEncoderValues(unsigned long RAEncoder, unsigned long DEEncoder, double &dHa, double &dDec)
{
    long RAStepInit= ZeroPositionEncoders[AXIS1];
    long DEStepInit =ZeroPositionEncoders[AXIS2];
    long RASteps360= MicrostepsPerRevolution[AXIS1];
    long DESteps360= MicrostepsPerRevolution[AXIS1];

    // Convert from encoder values
    if (Hemisphere ==NORTH) {
        if (DEEncoder < DEStepInit) {	  //  Pre-meridian
            dDec = 90.0 - (DEStepInit*1.0 - DEEncoder*1.0) / (DESteps360*1.0)*360.0;
            dHa = -6.0 + (RAEncoder*1.0 - RAStepInit*1.0) / (RASteps360*1.0)*24.0;
        }
        else {
            dDec = 90.0 + (DEStepInit*1.0 - DEEncoder*1.0) / (DESteps360*1.0)*360.0;
            dHa = 6.0 + (RAEncoder*1.0 - RAStepInit*1.0) / (RASteps360*1.0)*24.0;
        }
    }
    else {
        if (DEEncoder > DEStepInit) {	  //  Pre-meridian
            dDec = -90.0 - (DEStepInit*1.0 - DEEncoder*1.0) / (DESteps360*1.0)*360.0;
            dHa = -6.0 - (RAEncoder*1.0 - RAStepInit*1.0) / (RASteps360*1.0)*24.0;
        }
        else {
            dDec = -90.0 + (DEStepInit*1.0 - DEEncoder*1.0) / (DESteps360*1.0)*360.0;
            dHa = 6.0 - (RAEncoder*1.0 - RAStepInit*1.0) / (RASteps360*1.0)*24.0;
        }

    }
}


void SkywatcherAZGTIMount::SetSouthernHemisphere(bool southern)
{

    LOGF_DEBUG("Set southern %s", southern);
    if (southern)
        Hemisphere = SOUTH;
    else
        Hemisphere = NORTH;
    RAInverted = (Hemisphere == SOUTH);
    //UpdateDEInverted();
    if (Hemisphere == NORTH)
    {
        HemisphereS[NORTH].s=ISS_ON;
        HemisphereS[SOUTH].s=ISS_OFF;
    }
    else
    {
        HemisphereS[NORTH].s=ISS_OFF;
        HemisphereS[SOUTH].s=ISS_ON;
    }

}



double SkywatcherAZGTIMount::getJulianDate()
{
//    struct timespec currentclock, diffclock;
//    double nsecs;
//    get_utc_time(&currentclock);
//    diffclock.tv_sec  = currentclock.tv_sec - lastclockupdate.tv_sec;
//    diffclock.tv_nsec = currentclock.tv_nsec - lastclockupdate.tv_nsec;
//    while (diffclock.tv_nsec > 1000000000)
//    {
//        diffclock.tv_sec++;
//        diffclock.tv_nsec -= 1000000000;
//    }
//    while (diffclock.tv_nsec < 0)
//    {
//        diffclock.tv_sec--;
//        diffclock.tv_nsec += 1000000000;
//    }

//    lndate.seconds += (diffclock.tv_sec + ((double)diffclock.tv_nsec / 1000000000.0));
//    nsecs        = lndate.seconds - floor(lndate.seconds);
//    utc.tm_sec   = lndate.seconds;
//    utc.tm_isdst = -1; // let mktime find if DST already in effect in utc
//    mktime(&utc); // normalize time
//    ln_get_date_from_tm(&utc, &lndate);
//    lndate.seconds += nsecs;
//    lastclockupdate = currentclock;
//    juliandate      = ln_get_julian_day(&lndate);
    juliandate=ln_get_julian_from_sys();
    return juliandate;
}

double SkywatcherAZGTIMount::getLst(double jd, double lng)
{
    double lst;
    lst = ln_get_apparent_sidereal_time(jd);
    lst += (lng / 15.0);
    lst = range24(lst);
    return lst;
}
double SkywatcherAZGTIMount::getLongitude(){
    double longitude=0;
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude);
    return longitude;
}
double SkywatcherAZGTIMount::getLatitude(){
    return (IUFindNumber(&LocationNP, "LAT")->value);
}
bool SkywatcherAZGTIMount::Goto(double ra, double dec)
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::Goto");


    if((IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON)){
        CurrentTrackingTarget.ra  = ra;
        CurrentTrackingTarget.dec = dec;
        EQGoto(ra,dec);
    }else{


    if (TrackState != SCOPE_IDLE)
        Abort();

    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "RA %lf DEC %lf", ra, dec);

    if (IUFindSwitch(&CoordSP, "TRACK")->s == ISS_ON || IUFindSwitch(&CoordSP, "SLEW")->s == ISS_ON)
    {
        char RAStr[32], DecStr[32];
        fs_sexa(RAStr, ra, 2, 3600);
        fs_sexa(DecStr, dec, 2, 3600);
        CurrentTrackingTarget.ra  = ra;
        CurrentTrackingTarget.dec = dec;
        LOGF_INFO("New Tracking target RA %s DEC %s", RAStr, DecStr);
    }

    ln_hrz_posn AltAz { 0, 0 };
    TelescopeDirectionVector TDV;

    if (TransformCelestialToTelescope(ra, dec, 0.0, TDV))
    {
        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "TDV x %lf y %lf z %lf", TDV.x, TDV.y, TDV.z);
        AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
        DEBUG(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Conversion OK");
    }
    else
    {
        // Try a conversion with the stored observatory position if any
        bool HavePosition = false;
        ln_lnlat_posn Position { 0, 0 };

        if ((nullptr != IUFindNumber(&LocationNP, "LAT")) && (0 != IUFindNumber(&LocationNP, "LAT")->value) &&
            (nullptr != IUFindNumber(&LocationNP, "LONG")) && (0 != IUFindNumber(&LocationNP, "LONG")->value))
        {
            // I assume that being on the equator and exactly on the prime meridian is unlikely
            Position.lat = IUFindNumber(&LocationNP, "LAT")->value;
            Position.lng = IUFindNumber(&LocationNP, "LONG")->value;
            HavePosition = true;
        }
        ln_equ_posn EquatorialCoordinates { 0, 0 };

        // libnova works in decimal degrees
        EquatorialCoordinates.ra  = ra * 360.0 / 24.0;
        EquatorialCoordinates.dec = dec;
        if (HavePosition)
        {
#ifdef USE_INITIAL_JULIAN_DATE
            ln_get_hrz_from_equ(&EquatorialCoordinates, &Position, InitialJulianDate, &AltAz);
#else
            ln_get_hrz_from_equ(&EquatorialCoordinates, &Position, ln_get_julian_from_sys(), &AltAz);
#endif
            TDV = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
            switch (GetApproximateMountAlignment())
            {
                case ZENITH:
                    break;

                case NORTH_CELESTIAL_POLE:
                    // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 minus
                    // the (positive)observatory latitude. The vector itself is rotated anticlockwise
                    TDV.RotateAroundY(Position.lat - 90.0);
                    break;

                case SOUTH_CELESTIAL_POLE:
                    // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 plus
                    // the (negative)observatory latitude. The vector itself is rotated clockwise
                    TDV.RotateAroundY(Position.lat + 90.0);
                    break;
            }
            AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
        }
        else
        {
            // The best I can do is just do a direct conversion to Alt/Az
            TDV = TelescopeDirectionVectorFromEquatorialCoordinates(EquatorialCoordinates);
            AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
        }
        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Conversion Failed - HavePosition %d", HavePosition);
    }
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT,
           "New Altitude %lf degrees %ld microsteps Azimuth %lf degrees %ld microsteps", AltAz.alt,
           DegreesToMicrosteps(AXIS2, AltAz.alt), AltAz.az, DegreesToMicrosteps(AXIS1, AltAz.az));

    // Update the current encoder positions
    GetEncoder(AXIS1);
    GetEncoder(AXIS2);

    long AltitudeOffsetMicrosteps =
        DegreesToMicrosteps(AXIS2, AltAz.alt) + ZeroPositionEncoders[AXIS2] - CurrentEncoders[AXIS2];
    long AzimuthOffsetMicrosteps =
        DegreesToMicrosteps(AXIS1, AltAz.az) + ZeroPositionEncoders[AXIS1] - CurrentEncoders[AXIS1];

    // Do I need to take out any complete revolutions before I do this test?
    if (AltitudeOffsetMicrosteps > MicrostepsPerRevolution[AXIS2] / 2)
    {
        // Going the long way round - send it the other way
        AltitudeOffsetMicrosteps -= MicrostepsPerRevolution[AXIS2];
    }

    if (AzimuthOffsetMicrosteps > MicrostepsPerRevolution[AXIS1] / 2)
    {
        // Going the long way round - send it the other way
        AzimuthOffsetMicrosteps -= MicrostepsPerRevolution[AXIS1];
    }
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Initial Axis2 %ld microsteps Axis1 %ld microsteps",ZeroPositionEncoders[AXIS2], ZeroPositionEncoders[AXIS1]);
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Current Axis2 %ld microsteps Axis1 %ld microsteps",CurrentEncoders[AXIS2], CurrentEncoders[AXIS1]);
    DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, "Altitude offset %ld microsteps Azimuth offset %ld microsteps",AltitudeOffsetMicrosteps, AzimuthOffsetMicrosteps);


    SlewTo(AXIS1, AzimuthOffsetMicrosteps);
    SlewTo(AXIS2, AltitudeOffsetMicrosteps);

    TrackState = SCOPE_SLEWING;

    EqNP.s = IPS_BUSY;

    return true;
     }
}

bool SkywatcherAZGTIMount::initProperties()
{
    IDLog("SkywatcherAZGTIMount::initProperties\n");

    // Allow the base class to initialise its visible before connection properties
    INDI::Telescope::initProperties();

    for (int i = 0; i < SlewRateSP.nsp; ++i)
    {
        sprintf(SlewRateSP.sp[i].label, "%.fx", SlewSpeeds[i]);
        SlewRateSP.sp[i].aux = (void *)&SlewSpeeds[i];
    }
    strncpy(SlewRateSP.sp[SlewRateSP.nsp - 1].name, "SLEW_MAX", MAXINDINAME);

    // Add default properties
    addDebugControl();
    addConfigurationControl();

    // Add alignment properties

    InitAlignmentProperties(this);

    // Set up property variables
    IUFillText(&BasicMountInfo[MOTOR_CONTROL_FIRMWARE_VERSION], "MOTOR_CONTROL_FIRMWARE_VERSION",
               "Motor control firmware version", "-");
    IUFillText(&BasicMountInfo[MOUNT_CODE], "MOUNT_CODE", "Mount code", "-");
    IUFillText(&BasicMountInfo[MOUNT_NAME], "MOUNT_NAME", "Mount name", "-");
    IUFillText(&BasicMountInfo[IS_DC_MOTOR], "IS_DC_MOTOR", "Is DC motor", "-");
    IUFillTextVector(&BasicMountInfoV, BasicMountInfo, 4, getDeviceName(), "BASIC_MOUNT_INFO",
                     "Basic mount information", DetailedMountInfoPage, IP_RO, 60, IPS_IDLE);

    IUFillNumber(&AxisOneInfo[MICROSTEPS_PER_REVOLUTION], "MICROSTEPS_PER_REVOLUTION", "Microsteps per revolution",
                 "%.0f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisOneInfo[STEPPER_CLOCK_FREQUENCY], "STEPPER_CLOCK_FREQUENCY", "Stepper clock frequency", "%.0f", 0,
                 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisOneInfo[HIGH_SPEED_RATIO], "HIGH_SPEED_RATIO", "High speed ratio", "%.0f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisOneInfo[MICROSTEPS_PER_WORM_REVOLUTION], "MICROSTEPS_PER_WORM_REVOLUTION",
                 "Microsteps per worm revolution", "%.0f", 0, 0xFFFFFF, 1, 0);

    IUFillNumberVector(&AxisOneInfoV, AxisOneInfo, 4, getDeviceName(), "AXIS_ONE_INFO", "Axis one information",
                       DetailedMountInfoPage, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&AxisOneState[FULL_STOP], "FULL_STOP", "FULL_STOP", ISS_OFF);
    IUFillSwitch(&AxisOneState[SLEWING], "SLEWING", "SLEWING", ISS_OFF);
    IUFillSwitch(&AxisOneState[SLEWING_TO], "SLEWING_TO", "SLEWING_TO", ISS_OFF);
    IUFillSwitch(&AxisOneState[SLEWING_FORWARD], "SLEWING_FORWARD", "SLEWING_FORWARD", ISS_OFF);
    IUFillSwitch(&AxisOneState[HIGH_SPEED], "HIGH_SPEED", "HIGH_SPEED", ISS_OFF);
    IUFillSwitch(&AxisOneState[NOT_INITIALISED], "NOT_INITIALISED", "NOT_INITIALISED", ISS_ON);
    IUFillSwitchVector(&AxisOneStateV, AxisOneState, 6, getDeviceName(), "AXIS_ONE_STATE", "Axis one state",
                       DetailedMountInfoPage, IP_RO, ISR_NOFMANY, 60, IPS_IDLE);

    IUFillNumber(&AxisTwoInfo[MICROSTEPS_PER_REVOLUTION], "MICROSTEPS_PER_REVOLUTION", "Microsteps per revolution",
                 "%.0f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisTwoInfo[STEPPER_CLOCK_FREQUENCY], "STEPPER_CLOCK_FREQUENCY", "Step timer frequency", "%.0f", 0,
                 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisTwoInfo[HIGH_SPEED_RATIO], "HIGH_SPEED_RATIO", "High speed ratio", "%.0f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisTwoInfo[MICROSTEPS_PER_WORM_REVOLUTION], "MICROSTEPS_PER_WORM_REVOLUTION",
                 "Mictosteps per worm revolution", "%.0f", 0, 0xFFFFFF, 1, 0);

    IUFillNumberVector(&AxisTwoInfoV, AxisTwoInfo, 4, getDeviceName(), "AXIS_TWO_INFO", "Axis two information",
                       DetailedMountInfoPage, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&AxisTwoState[FULL_STOP], "FULL_STOP", "FULL_STOP", ISS_OFF);
    IUFillSwitch(&AxisTwoState[SLEWING], "SLEWING", "SLEWING", ISS_OFF);
    IUFillSwitch(&AxisTwoState[SLEWING_TO], "SLEWING_TO", "SLEWING_TO", ISS_OFF);
    IUFillSwitch(&AxisTwoState[SLEWING_FORWARD], "SLEWING_FORWARD", "SLEWING_FORWARD", ISS_OFF);
    IUFillSwitch(&AxisTwoState[HIGH_SPEED], "HIGH_SPEED", "HIGH_SPEED", ISS_OFF);
    IUFillSwitch(&AxisTwoState[NOT_INITIALISED], "NOT_INITIALISED", "NOT_INITIALISED", ISS_ON);
    IUFillSwitchVector(&AxisTwoStateV, AxisTwoState, 6, getDeviceName(), "AXIS_TWO_STATE", "Axis two state",
                       DetailedMountInfoPage, IP_RO, ISR_NOFMANY, 60, IPS_IDLE);

    IUFillNumber(&AxisOneEncoderValues[RAW_MICROSTEPS], "RAW_MICROSTEPS", "Raw Microsteps", "%.0f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisOneEncoderValues[MICROSTEPS_PER_ARCSEC], "MICROSTEPS_PER_ARCSEC", "Microsteps/arcsecond",
                 "%.4f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisOneEncoderValues[OFFSET_FROM_INITIAL], "OFFSET_FROM_INITIAL", "Offset from initial", "%.0f", 0,
                 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisOneEncoderValues[DEGREES_FROM_INITIAL], "DEGREES_FROM_INITIAL", "Degrees from initial", "%.2f",
                 -1000.0, 1000.0, 1, 0);

    IUFillNumberVector(&AxisOneEncoderValuesV, AxisOneEncoderValues, 4, getDeviceName(), "AXIS1_ENCODER_VALUES",
                       "Axis 1 Encoder values", DetailedMountInfoPage, IP_RO, 60, IPS_IDLE);

    IUFillNumber(&AxisTwoEncoderValues[RAW_MICROSTEPS], "RAW_MICROSTEPS", "Raw Microsteps", "%.0f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisTwoEncoderValues[MICROSTEPS_PER_ARCSEC], "MICROSTEPS_PER_ARCSEC", "Microsteps/arcsecond",
                 "%.4f", 0, 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisTwoEncoderValues[OFFSET_FROM_INITIAL], "OFFSET_FROM_INITIAL", "Offset from initial", "%.0f", 0,
                 0xFFFFFF, 1, 0);
    IUFillNumber(&AxisTwoEncoderValues[DEGREES_FROM_INITIAL], "DEGREES_FROM_INITIAL", "Degrees from initial", "%.2f",
                 -1000.0, 1000.0, 1, 0);

    IUFillNumberVector(&AxisTwoEncoderValuesV, AxisTwoEncoderValues, 4, getDeviceName(), "AXIS2_ENCODER_VALUES",
                       "Axis 2 Encoder values", DetailedMountInfoPage, IP_RO, 60, IPS_IDLE);
    // Register any visible before connection properties

    // Slew modes
    IUFillSwitch(&SlewModes[SLEW_SILENT], "SLEW_SILENT", "Silent", ISS_OFF);
    IUFillSwitch(&SlewModes[SLEW_NORMAL], "SLEW_NORMAL", "Normal", ISS_ON);
    IUFillSwitchVector(&SlewModesSP, SlewModes, 2, getDeviceName(), "TELESCOPE_MOTION_SLEWMODE", "Slew Mode",
                       MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Wedge mode
    IUFillSwitch(&WedgeMode[WEDGE_SIMPLE], "WEDGE_SIMPLE", "Simple wedge", ISS_OFF);
    IUFillSwitch(&WedgeMode[WEDGE_EQ], "WEDGE_EQ", "EQ wedge", ISS_ON);
    IUFillSwitch(&WedgeMode[WEDGE_DISABLED], "WEDGE_DISABLED", "Disabled", ISS_OFF);
    IUFillSwitchVector(&WedgeModeSP, WedgeMode, 3, getDeviceName(), "TELESCOPE_MOTION_WEDGEMODE",
                       "Wedge Mode", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Track logging mode
    IUFillSwitch(&TrackLogMode[TRACKLOG_ENABLED], "TRACKLOG_ENABLED", "Enable logging", ISS_OFF);
    IUFillSwitch(&TrackLogMode[TRACKLOG_DISABLED], "TRACKLOG_DISABLED", "Disabled", ISS_ON);
    IUFillSwitchVector(&TrackLogModeSP, TrackLogMode, 2, getDeviceName(), "TELESCOPE_MOTION_TRACKLOGMODE",
                       "Track Logging Mode", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Guiding rates for RA/DEC axes
    IUFillNumber(&GuidingRatesN[0], "GUIDERA_RATE", "microsteps/seconds (RA)", "%1.3f", 0.00001, 100000.0, 0.00001, 0.5);
    IUFillNumber(&GuidingRatesN[1], "GUIDEDEC_RATE", "microsteps/seconds (Dec)", "%1.3f", 0.00001, 100000.0, 0.00001, 0.5);
    IUFillNumberVector(&GuidingRatesNP, GuidingRatesN, 2, getDeviceName(), "GUIDE_RATES", "Guide Rates", MOTION_TAB,
                       IP_RW, 60, IPS_IDLE);

    // Tracking rate
    // For Skywatcher Virtuoso:
    // Alt rate: 0.72, Az rate: 0.72, timeout: 1000 msec
    // For Skywatcher Merlin:
    // Alt rate: 0.64, Az rate: 0.64, timeout: 1000 msec
    IUFillNumber(&TrackingValuesN[0], "TRACKING_RATE_ALT", "rate (Alt)", "%1.3f", 0.001, 10.0, 0.000001, 1.00);
    IUFillNumber(&TrackingValuesN[1], "TRACKING_RATE_AZ", "rate (Az)", "%1.3f", 0.001, 10.0, 0.000001, 1.00);
    IUFillNumber(&TrackingValuesN[2], "TRACKING_TIMEOUT", "msec (period)", "%1.3f", 0.001, 10000.0, 0.000001, 1000.0);
    IUFillNumberVector(&TrackingValuesNP, TrackingValuesN, 3, getDeviceName(), "TRACKING_VALUES", "Tracking Values", MOTION_TAB,
                       IP_RW, 60, IPS_IDLE);

    // Park movement directions
    IUFillSwitch(&ParkMovementDirection[PARK_COUNTERCLOCKWISE], "PMD_COUNTERCLOCKWISE", "Counterclockwise", ISS_ON);
    IUFillSwitch(&ParkMovementDirection[PARK_CLOCKWISE], "PMD_CLOCKWISE", "Clockwise", ISS_OFF);
    IUFillSwitchVector(&ParkMovementDirectionSP, ParkMovementDirection, 2, getDeviceName(), "PARK_DIRECTION",
                       "Park Direction", MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Park positions
    IUFillSwitch(&ParkPosition[PARK_NORTH], "PARK_NORTH", "North", ISS_ON);
    IUFillSwitch(&ParkPosition[PARK_EAST], "PARK_EAST", "East", ISS_OFF);
    IUFillSwitch(&ParkPosition[PARK_SOUTH], "PARK_SOUTH", "South", ISS_OFF);
    IUFillSwitch(&ParkPosition[PARK_WEST], "PARK_WEST", "West", ISS_OFF);
    IUFillSwitchVector(&ParkPositionSP, ParkPosition, 4, getDeviceName(), "PARK_POSITION", "Park Position", MOTION_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // Unpark positions
    IUFillSwitch(&UnparkPosition[PARK_NORTH], "UNPARK_NORTH", "North", ISS_OFF);
    IUFillSwitch(&UnparkPosition[PARK_EAST], "UNPARK_EAST", "East", ISS_OFF);
    IUFillSwitch(&UnparkPosition[PARK_SOUTH], "UNPARK_SOUTH", "South", ISS_OFF);
    IUFillSwitch(&UnparkPosition[PARK_WEST], "UNPARK_WEST", "West", ISS_OFF);
    IUFillSwitchVector(&UnparkPositionSP, UnparkPosition, 4, getDeviceName(), "UNPARK_POSITION", "Unpark Position",
                       MOTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    /* Hemisphere */
    IUFillSwitch(&HemisphereS[NORTH], "North", "", ISS_ON);
    IUFillSwitch(&HemisphereS[SOUTH], "South", "", ISS_OFF);
    IUFillSwitchVector(&HemisphereSP, HemisphereS, 2, getDeviceName(), "HEMISPHERE", "Hemisphere", MOTION_TAB, IP_RO,
                       ISR_1OFMANY, 0, IPS_IDLE);

    // Guiding support
    initGuiderProperties(getDeviceName(), GUIDE_TAB);
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);

    //Track MODE
    IUFillSwitch(&TrackModeS[0], "SIDEREAL", "Sidereal", ISS_ON);
    IUFillSwitch(&TrackModeS[1], "LUNAR", "Lunar", ISS_OFF);
    IUFillSwitch(&TrackModeS[2], "SOLAR", "Solar", ISS_OFF);
    IUFillSwitchVector(&TrackModeSP, TrackModeS, NARRAY(TrackModeS), getDeviceName(), "TRACK_MODE", "Track Mode", MOTION_TAB,
                       IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    //TurnRAEncoder(false);
    //TurnDEEncoder(false);
    SetParkDataType(PARK_RA_DEC);
    double longitude=0, latitude=90;
    // Get value from config file if it exists.
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude);
    currentRA  = get_local_sidereal_time(longitude);
    IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LAT", &latitude);
    currentDEC = latitude > 0 ? 90 : -90;

    return true;
}

void SkywatcherAZGTIMount::ISGetProperties(const char *dev)
{
    IDLog("SkywatcherAZGTIMount::ISGetProperties\n");
    INDI::Telescope::ISGetProperties(dev);

    if (isConnected())
    {
        // Fill in any real values now available MCInit should have been called already
        UpdateDetailedMountInformation(false);

        // Define our connected only properties to the base driver
        // e.g. defineNumber(MyNumberVectorPointer);
        // This will register our properties and send a IDDefXXXX mewssage to any connected clients
        defineText(&BasicMountInfoV);
        defineNumber(&AxisOneInfoV);
        defineSwitch(&AxisOneStateV);
        defineNumber(&AxisTwoInfoV);
        defineSwitch(&AxisTwoStateV);
        defineNumber(&AxisOneEncoderValuesV);
        defineNumber(&AxisTwoEncoderValuesV);
        defineSwitch(&SlewModesSP);
        defineSwitch(&WedgeModeSP);
        defineSwitch(&TrackLogModeSP);
        defineNumber(&GuidingRatesNP);
        defineNumber(&TrackingValuesNP);
        defineSwitch(&ParkMovementDirectionSP);
        defineSwitch(&ParkPositionSP);
        defineSwitch(&UnparkPositionSP);
        defineSwitch(&TrackModeSP);
        defineNumber(&GuideNSNP);
        defineNumber(&GuideWENP);
        defineSwitch(&HemisphereSP);
    }
}

bool SkywatcherAZGTIMount::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                                      char *formats[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // It is for us
        ProcessAlignmentBLOBProperties(this, name, sizes, blobsizes, blobs, formats, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool SkywatcherAZGTIMount::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (strcmp(name, "GUIDE_RATES") == 0)
        {
            ResetGuidePulses();
            GuidingRatesNP.s = IPS_OK;
            IUUpdateNumber(&GuidingRatesNP, values, names, n);
            IDSetNumber(&GuidingRatesNP, nullptr);
            return true;
        }

        if (strcmp(name, "TRACKING_VALUES") == 0)
        {
            TrackingValuesNP.s = IPS_OK;
            IUUpdateNumber(&TrackingValuesNP, values, names, n);
            IDSetNumber(&TrackingValuesNP, nullptr);
            return true;
        }

        // Let our driver do sync operation in park position
        if (strcmp(name, "EQUATORIAL_EOD_COORD") == 0)
        {
            double ra  = -1;
            double dec = -100;

            for (int x = 0; x < n; x++)
            {
                INumber *eqp = IUFindNumber(&EqNP, names[x]);
                if (eqp == &EqN[AXIS_RA])
                {
                    ra = values[x];
                }
                else if (eqp == &EqN[AXIS_DE])
                {
                    dec = values[x];
                }
            }
            if ((ra >= 0) && (ra <= 24) && (dec >= -90) && (dec <= 90))
            {
                ISwitch *sw = IUFindSwitch(&CoordSP, "SYNC");

                if (sw != nullptr && sw->s == ISS_ON && isParked())
                {
                    return Sync(ra, dec);
                }
            }
        }

        processGuiderProperties(name, values, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool SkywatcherAZGTIMount::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    IUUpdateSwitch(getSwitch(name), states, names, n);
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // It is for us

        // Process alignment properties
        ProcessAlignmentSwitchProperties(this, name, states, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool SkywatcherAZGTIMount::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // It is for us
        ProcessAlignmentTextProperties(this, name, texts, names, n);
    }
    // Pass it up the chain
    bool Ret =  INDI::Telescope::ISNewText(dev, name, texts, names, n);

    // The scope config switch must be updated after the config is saved to disk
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (name && std::string(name) == "SCOPE_CONFIG_NAME")
        {
            UpdateScopeConfigSwitch();
        }
    }
    return Ret;
}

void SkywatcherAZGTIMount::UpdateScopeConfigSwitch()
{
    if (!CheckFile(ScopeConfigFileName, false))
    {
        DEBUGF(INDI::Logger::DBG_SESSION, "Can't open XML file (%s) for read", ScopeConfigFileName.c_str());
        return;
    }
    LilXML *XmlHandle      = newLilXML();
    FILE *FilePtr          = fopen(ScopeConfigFileName.c_str(), "r");
    XMLEle *RootXmlNode    = nullptr;
    XMLEle *CurrentXmlNode = nullptr;
    XMLAtt *Ap             = nullptr;
    bool DeviceFound       = false;
    char ErrMsg[512];

    RootXmlNode = readXMLFile(FilePtr, XmlHandle, ErrMsg);
    delLilXML(XmlHandle);
    XmlHandle = nullptr;
    if (!RootXmlNode)
    {
        DEBUGF(INDI::Logger::DBG_SESSION, "Failed to parse XML file (%s): %s", ScopeConfigFileName.c_str(), ErrMsg);
        return;
    }
    if (std::string(tagXMLEle(RootXmlNode)) != ScopeConfigRootXmlNode)
    {
        DEBUGF(INDI::Logger::DBG_SESSION, "Not a scope config XML file (%s)", ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return;
    }
    CurrentXmlNode = nextXMLEle(RootXmlNode, 1);
    // Find the current telescope in the config file
    while (CurrentXmlNode)
    {
        if (std::string(tagXMLEle(CurrentXmlNode)) != ScopeConfigDeviceXmlNode)
        {
            CurrentXmlNode = nextXMLEle(RootXmlNode, 0);
            continue;
        }
        Ap = findXMLAtt(CurrentXmlNode, ScopeConfigNameXmlNode.c_str());
        if (Ap && !strcmp(valuXMLAtt(Ap), getDeviceName()))
        {
            DeviceFound = true;
            break;
        }
        CurrentXmlNode = nextXMLEle(RootXmlNode, 0);
    }
    if (!DeviceFound)
    {
        DEBUGF(INDI::Logger::DBG_SESSION, "No a scope config found for %s in the XML file (%s)", getDeviceName(),
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return;
    }
    // Read the values
    XMLEle *XmlNode       = nullptr;
    XMLEle *DeviceXmlNode = CurrentXmlNode;
    std::string ConfigName;

    for (int i = 1; i < 7; ++i)
    {
        bool Found = true;

        CurrentXmlNode = findXMLEle(DeviceXmlNode, ("config"+std::to_string(i)).c_str());
        if (CurrentXmlNode)
        {
            XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigLabelApXmlNode.c_str());
            if (XmlNode)
            {
                ConfigName = pcdataXMLEle(XmlNode);
            }
        } else {
            Found = false;
        }
        // Change the switch label
        ISwitch *configSwitch = IUFindSwitch(&ScopeConfigsSP, ("SCOPE_CONFIG"+std::to_string(i)).c_str());

        if (configSwitch != nullptr)
        {
            // The config is not used yet
            if (!Found)
            {
                strncpy(configSwitch->label, ("Config #"+std::to_string(i)+" - Not used").c_str(), MAXINDILABEL);
                continue;
            }
            // Empty switch label
            if (ConfigName.empty())
            {
                strncpy(configSwitch->label, ("Config #"+std::to_string(i)+" - Untitled").c_str(), MAXINDILABEL);
                continue;
            }
            strncpy(configSwitch->label, ("Config #"+std::to_string(i)+" - "+ConfigName).c_str(), MAXINDILABEL);
        }
    }
    delXMLEle(RootXmlNode);
    // Delete the joystick control to get the telescope config switch to the bottom of the page
    deleteProperty("USEJOYSTICK");
    // Recreate the switch control
    deleteProperty(ScopeConfigsSP.name);
    defineSwitch(&ScopeConfigsSP);
}

double SkywatcherAZGTIMount::GetSlewRate()
{
    ISwitch *Switch = IUFindOnSwitch(&SlewRateSP);
    double Rate     = *((double *)Switch->aux);

    return Rate;
}

bool SkywatcherAZGTIMount::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::MoveNS");

    double speed =
        (dir == DIRECTION_NORTH) ? GetSlewRate() * LOW_SPEED_MARGIN / 2 : -GetSlewRate() * LOW_SPEED_MARGIN / 2;
    const char *dirStr = (dir == DIRECTION_NORTH) ? "North" : "South";

    if (IsMerlinMount())
    {
        speed = -speed;
    }

    switch (command)
    {
        case MOTION_START:
            DEBUGF(DBG_SCOPE, "Starting Slew %s", dirStr);
            // Ignore the silent mode because MoveNS() is called by the manual motion UI controls.
            Slew(AXIS2, speed, true);
            break;

        case MOTION_STOP:
            DEBUGF(DBG_SCOPE, "Stopping Slew %s", dirStr);
            SlowStop(AXIS2);
            break;
    }

    return true;
}

bool SkywatcherAZGTIMount::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::MoveWE");

    double speed =
        (dir == DIRECTION_WEST) ? GetSlewRate() * LOW_SPEED_MARGIN / 2 : -GetSlewRate() * LOW_SPEED_MARGIN / 2;
    const char *dirStr = (dir == DIRECTION_WEST) ? "West" : "East";

    if (IsVirtuosoMount()){
        speed = -speed;
    }

    switch (command)
    {
        case MOTION_START:
            DEBUGF(DBG_SCOPE, "Starting Slew %s", dirStr);
            // Ignore the silent mode because MoveNS() is called by the manual motion UI controls.
            Slew(AXIS1, speed, true);
            break;

        case MOTION_STOP:
            DEBUGF(DBG_SCOPE, "Stopping Slew %s", dirStr);
            SlowStop(AXIS1);
            break;
    }

    return true;
}

double SkywatcherAZGTIMount::GetParkDeltaAz(ParkDirection_t target_direction, ParkPosition_t target_position)
{
    double Result = 0;

    DEBUGF(DBG_SCOPE, "GetParkDeltaAz: direction %d - position: %d", (int)target_direction, (int)target_position);
    // Calculate delta degrees (target: NORTH)
    if (target_position == PARK_NORTH)
    {
        if (target_direction == PARK_COUNTERCLOCKWISE)
        {
            Result = -CurrentAltAz.az;
        }
        else
        {
            Result = 360 - CurrentAltAz.az;
        }
    }
    // Calculate delta degrees (target: EAST)
    if (target_position == PARK_EAST)
    {
        if (target_direction == PARK_COUNTERCLOCKWISE)
        {
            if (CurrentAltAz.az > 0 && CurrentAltAz.az < 90)
                Result = -270 - CurrentAltAz.az;
            else
                Result = -CurrentAltAz.az + 90;
        }
        else
        {
            if (CurrentAltAz.az > 0 && CurrentAltAz.az < 90)
                Result = 90 - CurrentAltAz.az;
            else
                Result = 360 - CurrentAltAz.az + 90;
        }
    }
    // Calculate delta degrees (target: SOUTH)
    if (target_position == PARK_SOUTH)
    {
        if (target_direction == PARK_COUNTERCLOCKWISE)
        {
            if (CurrentAltAz.az > 0 && CurrentAltAz.az < 180)
                Result = -180 - CurrentAltAz.az;
            else
                Result = -CurrentAltAz.az + 180;
        }
        else
        {
            if (CurrentAltAz.az > 0 && CurrentAltAz.az < 180)
                Result = 180 - CurrentAltAz.az;
            else
                Result = 360 - CurrentAltAz.az + 180;
        }
    }
    // Calculate delta degrees (target: WEST)
    if (target_position == PARK_WEST)
    {
        if (target_direction == PARK_COUNTERCLOCKWISE)
        {
            if (CurrentAltAz.az > 0 && CurrentAltAz.az < 270)
                Result = -90 - CurrentAltAz.az;
            else
                Result = -CurrentAltAz.az + 270;
        }
        else
        {
            if (CurrentAltAz.az > 0 && CurrentAltAz.az < 270)
                Result = 270 - CurrentAltAz.az;
            else
                Result = 360 - CurrentAltAz.az + 270;
        }
    }
    if (Result >= 360)
    {
        Result -= 360;
    }
    if (Result <= -360)
    {
        Result += 360;
    }
    return Result;
}

bool SkywatcherAZGTIMount::Park()
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::Park");
    ParkPosition_t TargetPosition   = PARK_NORTH;
    ParkDirection_t TargetDirection = PARK_COUNTERCLOCKWISE;
    double DeltaAlt                 = 0;
    double DeltaAz                  = 0;

    // Determinate the target position and direction
    if (IUFindSwitch(&ParkPositionSP, "PARK_NORTH") != nullptr &&
        IUFindSwitch(&ParkPositionSP, "PARK_NORTH")->s == ISS_ON)
    {
        TargetPosition = PARK_NORTH;
    }
    if (IUFindSwitch(&ParkPositionSP, "PARK_EAST") != nullptr &&
        IUFindSwitch(&ParkPositionSP, "PARK_EAST")->s == ISS_ON)
    {
        TargetPosition = PARK_EAST;
    }
    if (IUFindSwitch(&ParkPositionSP, "PARK_SOUTH") != nullptr &&
        IUFindSwitch(&ParkPositionSP, "PARK_SOUTH")->s == ISS_ON)
    {
        TargetPosition = PARK_SOUTH;
    }
    if (IUFindSwitch(&ParkPositionSP, "PARK_WEST") != nullptr &&
        IUFindSwitch(&ParkPositionSP, "PARK_WEST")->s == ISS_ON)
    {
        TargetPosition = PARK_WEST;
    }

    if (IUFindSwitch(&ParkMovementDirectionSP, "PMD_COUNTERCLOCKWISE") != nullptr &&
        IUFindSwitch(&ParkMovementDirectionSP, "PMD_COUNTERCLOCKWISE")->s == ISS_ON)
    {
        TargetDirection = PARK_COUNTERCLOCKWISE;
    }
    if (IUFindSwitch(&ParkMovementDirectionSP, "PMD_CLOCKWISE") != nullptr &&
        IUFindSwitch(&ParkMovementDirectionSP, "PMD_CLOCKWISE")->s == ISS_ON)
    {
        TargetDirection = PARK_CLOCKWISE;
    }
    DeltaAz = GetParkDeltaAz(TargetDirection, TargetPosition);

    // Move the telescope to the desired position
    long AltitudeOffsetMicrosteps = DegreesToMicrosteps(AXIS2, DeltaAlt);
    long AzimuthOffsetMicrosteps  = DegreesToMicrosteps(AXIS1, DeltaAz);

    DEBUGF(DBG_SCOPE, "Parking: Delta altitude %1.2f - delta azimuth %1.2f", DeltaAlt, DeltaAz);
    DEBUGF(DBG_SCOPE,"Parking: Altitude offset %ld microsteps Azimuth offset %ld microsteps",
           AltitudeOffsetMicrosteps, AzimuthOffsetMicrosteps);

    if (IUFindSwitch(&SlewModesSP, "SLEW_NORMAL")->s == ISS_ON)
    {
        SilentSlewMode = false;
    }
    else
    {
        SilentSlewMode = true;
    }
    SlewTo(AXIS1, AzimuthOffsetMicrosteps);
    SlewTo(AXIS2, AltitudeOffsetMicrosteps);

    TrackState = SCOPE_PARKING;
    if (NumPark == 0)
    {
        LOG_INFO("Parking Mount...");
    }
    return true;
}

bool SkywatcherAZGTIMount::UnPark()
{
    DEBUG(DBG_SCOPE, "SkywatcherAltAzSimple::UnPark");
    SetParked(false);
    NumPark = 0;
    return true;
}

bool SkywatcherAZGTIMount::SetCurrentPark()
{
    LOG_INFO("Setting arbitrary park positions is not supported yet.");
    return false;
}

bool SkywatcherAZGTIMount::SetDefaultPark()
{

    // By default az to north, and alt to pole
    LOG_DEBUG("Setting Park Data to Default.");

    // By default set RA to LST
    SetAxis1Park(get_local_sidereal_time(getLongitude()));

    // Set DEC to 90 or -90 depending on the hemisphere
    SetAxis2Park((HemisphereS[NORTH].s == ISS_ON) ? 90 : -90);



    return true;
}

bool SkywatcherAZGTIMount::ReadScopeStatus()
{
   DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::ReadScopeStatus");

    // leave the following stuff in for the time being it is mostly harmless

    // Quick check of the mount
    if (UpdateCount == 0 && !GetMotorBoardVersion(AXIS1))
        return false;

    if (!GetStatus(AXIS1))
        return false;

    if (!GetStatus(AXIS2))
        return false;

    // Update Axis Position
    if (!GetEncoder(AXIS1))
        return false;
    if (!GetEncoder(AXIS2))
        return false;

    if (UpdateCount % 5 == 0)
        UpdateDetailedMountInformation(true);

    UpdateCount++;
    if (TrackState == SCOPE_PARKING)
    {
        if (!IsInMotion(AXIS1) && !IsInMotion(AXIS2))
        {
            SetParked(true);
        }
    }

    // Calculate new RA DEC

   //Add logic based on EQ or AZ mode
    if((IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON)){
        double juliandate=0;
        double lst=0;
        juliandate = getJulianDate();
        lst        = getLst(juliandate, getLongitude());

        double longitude=0, latitude=90;
        // Get value from config file if it exists.
        IUGetConfigNumber(getDeviceName(), "GEOGRAPHIC_COORD", "LONG", &longitude);
        currentRA  = get_local_sidereal_time(longitude);

        const char *maligns[3] = { "ZENITH", "NORTH", "SOUTH" };
        struct ln_equ_posn RaDec;
        // Use HA/Dec as  telescope coordinate system
        RaDec.ra                     = ((lst - currentRA) * 360.0) / 24.0;
        RaDec.dec                    = currentDEC;
        TelescopeDirectionVector TDV = TelescopeDirectionVectorFromLocalHourAngleDeclination(RaDec);
        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT,
               "Status: Mnt. Algnt. %s Date %lf encoders RA=%ld DE=%ld Telescope RA %lf DEC %lf",
               maligns[GetApproximateMountAlignment()], juliandate, CurrentEncoders[AXIS1], CurrentEncoders[AXIS2], currentRA,
                currentDEC);
        DEBUGF(INDI::AlignmentSubsystem::DBG_ALIGNMENT, " Direction RA(deg.)  %lf DEC %lf TDV(x %lf y %lf z %lf)",
               RaDec.ra, RaDec.dec, TDV.x, TDV.y, TDV.z);
        //aligned = true;
        HAandDECfromEncoderValues(CurrentEncoders[AXIS1], CurrentEncoders[AXIS2],currentHA,currentDEC );
        currentRA = range24(currentHA+lst);
        NewRaDec(currentRA, currentDEC);

    }else{

    ln_hrz_posn AltAz { 0, 0 };

    AltAz.alt = MicrostepsToDegrees(AXIS2, CurrentEncoders[AXIS2] - ZeroPositionEncoders[AXIS2]);
    if (VerboseScopeStatus)
    {
        DEBUGF(DBG_SCOPE, "Axis2 encoder %ld initial %ld alt(degrees) %lf",
               CurrentEncoders[AXIS2], ZeroPositionEncoders[AXIS2], AltAz.alt);
    }
    AltAz.az = MicrostepsToDegrees(AXIS1, CurrentEncoders[AXIS1] - ZeroPositionEncoders[AXIS1]);
    CurrentAltAz = AltAz;
    if (VerboseScopeStatus)
    {
        DEBUGF(DBG_SCOPE, "Axis1 encoder %ld initial %ld az(degrees) %lf",
               CurrentEncoders[AXIS1], ZeroPositionEncoders[AXIS1], AltAz.az);
    }

    ln_equ_posn RaDec { 0, 0 };

    RaDec = GetRaDecPosition(AltAz.alt, AltAz.az);
    if (VerboseScopeStatus)
    {
        DEBUGF(DBG_SCOPE, "New RA %lf (hours) DEC %lf (degrees)", RaDec.ra, RaDec.dec);
    }
    LogMessage("STATUS: Ra %lf Dec %lf - Alt %lf Az %lf - microsteps %ld %ld", RaDec.ra, RaDec.dec,
               AltAz.alt, AltAz.az, CurrentEncoders[AXIS2]-ZeroPositionEncoders[AXIS2],
               CurrentEncoders[AXIS1]-ZeroPositionEncoders[AXIS1]);
    NewRaDec(RaDec.ra, RaDec.dec);

    }

    //VerboseScopeStatus = false;
    return true;
}


bool SkywatcherAZGTIMount::saveConfigItems(FILE *fp)
{
    IUSaveConfigSwitch(fp, &SlewModesSP);
    IUSaveConfigSwitch(fp, &WedgeModeSP);
    IUSaveConfigSwitch(fp, &TrackLogModeSP);
    IUSaveConfigNumber(fp, &GuidingRatesNP);
    IUSaveConfigNumber(fp, &TrackingValuesNP);
    IUSaveConfigSwitch(fp, &ParkMovementDirectionSP);
    IUSaveConfigSwitch(fp, &ParkPositionSP);
    IUSaveConfigSwitch(fp, &UnparkPositionSP);
    IUSaveConfigSwitch(fp, &TrackModeSP);

    return INDI::Telescope::saveConfigItems(fp);
}


bool SkywatcherAZGTIMount::Sync(double ra, double dec)
{
    DEBUG(DBG_SCOPE, "SkywatcherAZGTIMount::Sync");

    // Compute a telescope direction vector from the current encoders
    if (!GetEncoder(AXIS1))
        return false;
    if (!GetEncoder(AXIS2))
        return false;

    ln_hrz_posn AltAz { 0, 0 };

    AltAz = GetAltAzPosition(ra, dec);
    double DeltaAz = CurrentAltAz.az-AltAz.az;
    double DeltaAlt = CurrentAltAz.alt-AltAz.alt;

    LogMessage("SYNC: Ra %lf Dec %lf", ra, dec);
    MYDEBUGF(INDI::Logger::DBG_SESSION, "Sync ra: %lf dec: %lf => CurAz: %lf -> NewAz: %lf",
             ra, dec, CurrentAltAz.az, AltAz.az);
    PolarisPositionEncoders[AXIS1] += DegreesToMicrosteps(AXIS1, DeltaAz);
    PolarisPositionEncoders[AXIS2] += DegreesToMicrosteps(AXIS2, DeltaAlt);
    ZeroPositionEncoders[AXIS1] = PolarisPositionEncoders[AXIS1];
    ZeroPositionEncoders[AXIS2] = PolarisPositionEncoders[AXIS2];

    // The tracking seconds should be reset to restart the drift compensation
    ResetTrackingSeconds = true;

    // Stop any movements
    if (TrackState != SCOPE_IDLE && TrackState != SCOPE_PARKED)
    {
        Abort();
    }

    // Might as well do this
    UpdateDetailedMountInformation(true);
    return true;
}


void SkywatcherAZGTIMount::TimerHit()
{
    static bool Slewing    = false;
    static bool Tracking   = false;
    static int ElapsedTime = 0;
    char Direction ='0';
    if (!ReadScopeStatus())
    {
        SetTimer(TimeoutDuration);
        return;
    }

    LogMessage("SET TIMER: %d msec", TimeoutDuration);
    SetTimer(TimeoutDuration);
    ElapsedTime += TimeoutDuration;
    if (ElapsedTime >= 5000)
    {
        ElapsedTime        = 0;
        VerboseScopeStatus = true;
    }

    switch (TrackState)
    {
        case SCOPE_SLEWING:
            if (!Slewing)
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Slewing started");
                TrackingStartTimer = 0;
            }
            TrackingMsecs   = 0;
            GuideDeltaAlt   = 0;
            GuideDeltaAz    = 0;
            ResetGuidePulses();
            TimeoutDuration = 400;
            Tracking        = false;
            Slewing         = true;
            GuidingPulses.clear();
            if ((AxesStatus[AXIS1].FullStop) && (AxesStatus[AXIS2].FullStop))
            {
                TrackingStartTimer += TimeoutDuration;
                if (TrackingStartTimer < 3000)
                    return;

                if (IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON ||
                    IUFindSwitch(&CoordSP, "TRACK")->s == ISS_ON)

                {
                    // Goto has finished start tracking
                    TrackState = SCOPE_TRACKING;
                }
                else
                {
                    TrackState = SCOPE_IDLE;
                    break;
                }
            }
            break;

        case SCOPE_TRACKING:
        {
            if (!Tracking)
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Tracking started");
                TrackingMsecs   = 0;
                TimeoutDuration = (int)IUFindNumber(&TrackingValuesNP, "TRACKING_TIMEOUT")->value;
                GuideDeltaAlt   = 0;
                GuideDeltaAz    = 0;
                ResetGuidePulses();
            }

            // Restart the drift compensation after syncing
            if (ResetTrackingSeconds)
            {
                ResetTrackingSeconds = false;
                TrackingMsecs        = 0;
                GuideDeltaAlt        = 0;
                GuideDeltaAz         = 0;
                ResetGuidePulses();
            }
            TrackingMsecs += TimeoutDuration;
            if (TrackingMsecs % 60000 == 0)
            {
                DEBUGF(INDI::Logger::DBG_SESSION, "Tracking in progress (%d seconds elapsed)", TrackingMsecs / 1000);
            }
            Tracking = true;
            Slewing  = false;
            // Continue or start tracking


            if((IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON)){

            }else{


//            ln_hrz_posn AltAz { 0, 0 };
            ln_hrz_posn FutureAltAz { 0, 0 };
            FutureAltAz = GetAltAzPosition(CurrentTrackingTarget.ra, CurrentTrackingTarget.dec,
                                           (double)TimeoutDuration / 1000);

            // Calculate the auto-guiding delta degrees
            for (auto pulse : GuidingPulses)
            {
                GuideDeltaAlt += pulse.DeltaAlt;
                GuideDeltaAz += pulse.DeltaAz;
            }
            GuidingPulses.clear();

            long AltitudeOffsetMicrosteps = DegreesToMicrosteps(AXIS2, FutureAltAz.alt-CurrentAltAz.alt+GuideDeltaAlt);
            long AzimuthOffsetMicrosteps = DegreesToMicrosteps(AXIS1, FutureAltAz.az-CurrentAltAz.az+GuideDeltaAz);

            // When the Alt/Az mount is on the top of an EQ mount, the EQ mount already tracks in
            // sidereal speed. Only autoguiding is enabled in tracking mode.
            if (IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON)
            {

                SetMotionMode(AXIS1,'1',Direction);
                AltitudeOffsetMicrosteps = (long)((float)IUFindNumber(&GuidingRatesNP, "GUIDEDEC_RATE")->value*GuideDeltaAlt);
                AzimuthOffsetMicrosteps = (long)((float)IUFindNumber(&GuidingRatesNP, "GUIDERA_RATE")->value*GuideDeltaAz);
                GuideDeltaAlt = 0;
                GuideDeltaAz = 0;
                // Correct the movements of the EQ mount
                double DeltaAz = CurrentAltAz.az-FutureAltAz.az;
                double DeltaAlt = CurrentAltAz.alt-FutureAltAz.alt;

                PolarisPositionEncoders[AXIS1] += DegreesToMicrosteps(AXIS1, DeltaAz);
                PolarisPositionEncoders[AXIS2] += DegreesToMicrosteps(AXIS2, DeltaAlt);
                ZeroPositionEncoders[AXIS1] = PolarisPositionEncoders[AXIS1];
                ZeroPositionEncoders[AXIS2] = PolarisPositionEncoders[AXIS2];
            }

            if (AltitudeOffsetMicrosteps > MicrostepsPerRevolution[AXIS2] / 2)
            {
                // Going the long way round - send it the other way
                AltitudeOffsetMicrosteps -= MicrostepsPerRevolution[AXIS2];
            }
            if (AzimuthOffsetMicrosteps > MicrostepsPerRevolution[AXIS1] / 2)
            {
                // Going the long way round - send it the other way
                AzimuthOffsetMicrosteps -= MicrostepsPerRevolution[AXIS1];
            }
            if (AltitudeOffsetMicrosteps < -MicrostepsPerRevolution[AXIS2] / 2)
            {
                // Going the long way round - send it the other way
                AltitudeOffsetMicrosteps += MicrostepsPerRevolution[AXIS2];
            }
            if (AzimuthOffsetMicrosteps < -MicrostepsPerRevolution[AXIS1] / 2)
            {
                // Going the long way round - send it the other way
                AzimuthOffsetMicrosteps += MicrostepsPerRevolution[AXIS1];
            }

            AltitudeOffsetMicrosteps = (long)((double)AltitudeOffsetMicrosteps*IUFindNumber(&TrackingValuesNP, "TRACKING_RATE_ALT")->value);
            AzimuthOffsetMicrosteps = (long)((double)AzimuthOffsetMicrosteps*IUFindNumber(&TrackingValuesNP, "TRACKING_RATE_AZ")->value);

            LogMessage("TRACKING: now Alt %lf Az %lf - future Alt %lf Az %lf - microsteps_diff Alt %ld Az %ld",
                       CurrentAltAz.alt, CurrentAltAz.az, FutureAltAz.alt, FutureAltAz.az,
                       AltitudeOffsetMicrosteps, AzimuthOffsetMicrosteps);
            if (0 != AzimuthOffsetMicrosteps)
            {
                SlewTo(AXIS1, AzimuthOffsetMicrosteps, false);
            }
            else
            {
                // Nothing to do - stop the axis
                //SlowStop(AXIS1);
            }

            if (0 != AltitudeOffsetMicrosteps)
            {
                SlewTo(AXIS2, AltitudeOffsetMicrosteps, false);
            }
            else
            {
                // Nothing to do - stop the axis
                //SlowStop(AXIS2);
            }

            DEBUGF(DBG_SCOPE, "Tracking - AXIS1 error %d (offset: %ld) AXIS2 error %d (offset: %ld)",
                   OldTrackingTarget[AXIS1] - CurrentEncoders[AXIS1], AzimuthOffsetMicrosteps,
                   OldTrackingTarget[AXIS2] - CurrentEncoders[AXIS2], AltitudeOffsetMicrosteps);

            OldTrackingTarget[AXIS1] = AzimuthOffsetMicrosteps + CurrentEncoders[AXIS1];
            OldTrackingTarget[AXIS2] = AltitudeOffsetMicrosteps + CurrentEncoders[AXIS2];
            break;
            }
        }
        break;
        case SCOPE_PARKING:
        {

            if (NumPark++ < 2)
                {
                    Park();
                 }
            else
            {
                TrackState = SCOPE_PARKED;
                SetParked(true);
            }
        }break;
        default:
            if (Slewing)
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Slewing stopped");
            }
            if (Tracking)
            {
                DEBUG(INDI::Logger::DBG_SESSION, "Tracking stopped");
            }
            TrackingMsecs   = 0;
            GuideDeltaAlt   = 0;
            GuideDeltaAz    = 0;
            ResetGuidePulses();
            TimeoutDuration = 1000;
            Tracking        = false;
            Slewing         = false;
            GuidingPulses.clear();
            break;
    }
}

bool SkywatcherAZGTIMount::updateProperties()
{

    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        // Fill in any real values now available MCInit should have been called already

        UpdateDetailedMountInformation(false);

        // Define our connected only properties to the base driver
        // e.g. defineNumber(MyNumberVectorPointer);
        // This will register our properties and send a IDDefXXXX message to any connected clients
        // I have now idea why I have to do this here as well as in ISGetProperties. It makes me
        // concerned there is a design or implementation flaw somewhere.
        defineText(&BasicMountInfoV);
        defineNumber(&AxisOneInfoV);
        defineSwitch(&AxisOneStateV);
        defineNumber(&AxisTwoInfoV);
        defineSwitch(&AxisTwoStateV);
        defineNumber(&AxisOneEncoderValuesV);
        defineNumber(&AxisTwoEncoderValuesV);
        defineSwitch(&SlewModesSP);
        defineSwitch(&WedgeModeSP);
        defineSwitch(&TrackLogModeSP);
        defineNumber(&GuidingRatesNP);
        defineNumber(&TrackingValuesNP);
        defineSwitch(&ParkMovementDirectionSP);
        defineSwitch(&ParkPositionSP);
        defineSwitch(&UnparkPositionSP);
        defineSwitch(&TrackModeSP);
        defineSwitch(&HemisphereSP);
        defineNumber(&GuideNSNP);
        defineNumber(&GuideWENP);

        if (InitPark())
        {
            // If loading parking data is successful, we just set the default parking values.
            SetAxis1ParkDefault(ZeroPositionEncoders[AXIS1]);
            SetAxis2ParkDefault(ZeroPositionEncoders[AXIS2]);
        }
        else
        {
            // Otherwise, we set all parking data to default in case no parking data is found.
            SetAxis1Park(ZeroPositionEncoders[AXIS1]);
            SetAxis2Park(ZeroPositionEncoders[AXIS2]);
            SetAxis1ParkDefault(ZeroPositionEncoders[AXIS1]);
            SetAxis2ParkDefault(ZeroPositionEncoders[AXIS2]);
        }


      if(LocationN[LOCATION_LATITUDE].value <0.0){
            SetSouthernHemisphere(true);

        }else{
            SetSouthernHemisphere(false);
         }
        return true;
    }
    else
    {
        // Delete any connected only properties from the base driver's list
        // e.g. deleteProperty(MyNumberVector.name);
        deleteProperty(BasicMountInfoV.name);
        deleteProperty(AxisOneInfoV.name);
        deleteProperty(AxisOneStateV.name);
        deleteProperty(AxisTwoInfoV.name);
        deleteProperty(AxisTwoStateV.name);
        deleteProperty(AxisOneEncoderValuesV.name);
        deleteProperty(AxisTwoEncoderValuesV.name);
        deleteProperty(SlewModesSP.name);
        deleteProperty(WedgeModeSP.name);
        deleteProperty(TrackLogModeSP.name);
        deleteProperty(GuidingRatesNP.name);
        deleteProperty(TrackingValuesNP.name);
        deleteProperty(ParkMovementDirectionSP.name);
        deleteProperty(ParkPositionSP.name);
        deleteProperty(UnparkPositionSP.name);

        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        return true;
    }
}

IPState SkywatcherAZGTIMount::GuideNorth(uint32_t ms)
{
    GuidingPulse Pulse;

    TimeoutDuration = 250;
    CalculateGuidePulses();
    Pulse.DeltaAz = NorthPulse.DeltaAz;
    Pulse.DeltaAlt = NorthPulse.DeltaAlt;
    Pulse.Duration = (int)ms;
    Pulse.OriginalDuration = (int)ms;
    GuidingPulses.push_back(Pulse);
//    IDMessage(nullptr, "GUIDE NORTH: %1.2f msec - deltaalt: %1.6f deltaaz: %1.6f", ms,
//              Pulse.DeltaAlt, Pulse.DeltaAz);
    return IPS_OK;
}

IPState SkywatcherAZGTIMount::GuideSouth(uint32_t ms)
{
    GuidingPulse Pulse;

    TimeoutDuration = 250;
    CalculateGuidePulses();
    Pulse.DeltaAz = -NorthPulse.DeltaAz;
    Pulse.DeltaAlt = -NorthPulse.DeltaAlt;
    Pulse.Duration = (int)ms;
    Pulse.OriginalDuration = (int)ms;
    GuidingPulses.push_back(Pulse);
//    IDMessage(nullptr, "GUIDE SOUTH: %1.2f msec - deltaalt: %1.6f deltaaz: %1.6f", ms,
//              Pulse.DeltaAlt, Pulse.DeltaAz);
    return IPS_OK;
}

IPState SkywatcherAZGTIMount::GuideWest(uint32_t ms)
{
    GuidingPulse Pulse;

    TimeoutDuration = 250;
    CalculateGuidePulses();
    Pulse.DeltaAz = WestPulse.DeltaAz;
    Pulse.DeltaAlt = WestPulse.DeltaAlt;
    Pulse.Duration = (int)ms;
    Pulse.OriginalDuration = (int)ms;
    GuidingPulses.push_back(Pulse);
//    IDMessage(nullptr, "GUIDE WEST: %1.2f msec - deltaalt: %1.6f deltaaz: %1.6f", ms,
//              Pulse.DeltaAlt, Pulse.DeltaAz);
    return IPS_OK;
}

IPState SkywatcherAZGTIMount::GuideEast(uint32_t ms)
{
    GuidingPulse Pulse;

    TimeoutDuration = 250;
    CalculateGuidePulses();
    Pulse.DeltaAz = -WestPulse.DeltaAz;
    Pulse.DeltaAlt = -WestPulse.DeltaAlt;
    Pulse.Duration = (int)ms;
    Pulse.OriginalDuration = (int)ms;
    GuidingPulses.push_back(Pulse);
//    IDMessage(nullptr, "GUIDE EAST: %1.2f msec - deltaalt: %1.6f deltaaz: %1.6f", ms,
//              Pulse.DeltaAlt, Pulse.DeltaAz);
    return IPS_OK;
}

void SkywatcherAZGTIMount::CalculateGuidePulses()
{
    if (NorthPulse.Duration != 0 || WestPulse.Duration != 0)
        return;

    // Calculate the west reference delta
    // Note: The RA is multiplied by 3.75 (90/24) to be more comparable with DEC values.
    const double WestRate = IUFindNumber(&GuidingRatesNP, "GUIDERA_RATE")->value / 10*-(double)1 / 60 / 60*3.75 / 100;

    ConvertGuideCorrection(WestRate, 0, WestPulse.DeltaAlt, WestPulse.DeltaAz);
    WestPulse.Duration = 1;

    // Calculate the north reference delta
    // Note: By some reason, it must be multiplied by 100 to match with the RA values.
    const double NorthRate = IUFindNumber(&GuidingRatesNP, "GUIDEDEC_RATE")->value / 10*(double)1 / 60 / 60*100 / 100;

    ConvertGuideCorrection(0, NorthRate, NorthPulse.DeltaAlt, NorthPulse.DeltaAz);
    NorthPulse.Duration = 1;
}

void SkywatcherAZGTIMount::ConvertGuideCorrection(double delta_ra, double delta_dec, double &delta_alt, double &delta_az)
{
    ln_hrz_posn OldAltAz { 0, 0 };
    ln_hrz_posn NewAltAz { 0, 0 };
    TelescopeDirectionVector OldTDV;
    TelescopeDirectionVector NewTDV;

    TransformCelestialToTelescope(CurrentTrackingTarget.ra, CurrentTrackingTarget.dec, 0.0, OldTDV);
    AltitudeAzimuthFromTelescopeDirectionVector(OldTDV, OldAltAz);
    TransformCelestialToTelescope(CurrentTrackingTarget.ra+delta_ra, CurrentTrackingTarget.dec+delta_dec, 0.0, NewTDV);
    AltitudeAzimuthFromTelescopeDirectionVector(NewTDV, NewAltAz);
    delta_alt = NewAltAz.alt-OldAltAz.alt;
    delta_az = NewAltAz.az-OldAltAz.az;
}
// Private methods

void SkywatcherAZGTIMount::ResetGuidePulses()
{
    GuidingPulses.clear();
    NorthPulse.Duration = 0;
    WestPulse.Duration = 0;
}

int SkywatcherAZGTIMount::skywatcher_tty_read(int fd, char *buf, int nbytes, int timeout, int *nbytes_read)
{
        INDI_UNUSED(fd); INDI_UNUSED(timeout);INDI_UNUSED(nbytes);
      return skywatcher_azgti_Read(myFD,buf,nbytes_read);
}

int SkywatcherAZGTIMount::skywatcher_tty_write(int fd, const char *buffer, int nbytes, int *nbytes_written)
{
    INDI_UNUSED(fd); INDI_UNUSED(nbytes);
    return skywatcher_azgti_Write(myFD,buffer,nbytes_written);

}
void SkywatcherAZGTIMount::setUdpFd(int fd){
   myFD=fd;

}

int SkywatcherAZGTIMount::skywatcher_azgti_Write(int fd, const char *buffer, int *nbytes_written){

    //struct addrinfo *servinfo =&serverInfo;  // will point to the results
    //send the message to server
     int bytesWritten = sendto(fd, buffer, strlen(buffer), 0, serverInfo->ai_addr, serverInfo->ai_addrlen) ;
     *nbytes_written = bytesWritten;
     //DEBUGF(DBG_SCOPE, "bytesSent %d ", bytesWritten);
     return bytesWritten;
}

int SkywatcherAZGTIMount::skywatcher_azgti_Read(int fd,char *buf,int *nbytes_read){
    struct sockaddr_storage addr;
    socklen_t addrlen = sizeof(addr);
    std::string responseString;

    int BUFSIZE = 2048;
    char responseStr[BUFSIZE]={0}; /* receive buffer */
    tcflush(fd, TCIFLUSH);
    memset(responseStr, 0, BUFSIZE);

    //read the response from server
    int bytesReceived= -2;
    while (true){
        bytesReceived=recvfrom(fd, responseStr, BUFSIZE, 0, serverInfo->ai_addr, &addrlen);
        if (bytesReceived== -1){

            DEBUG(DBG_SCOPE, "recvfrom threw -1 error ");
            break;
        }else if(bytesReceived>0) {


            *nbytes_read=bytesReceived;

            responseString=responseStr;

            //handle mount response here:

            if (responseString[0] == '!')
                {
                    MYDEBUG(DBG_SCOPE, "Bad reponse from mount");
                    return 1;
                }else{
                        responseString.erase(0, 1);
                        responseString.erase(bytesReceived-2);
                }
                memcpy(buf,responseString.c_str(),sizeof(responseString));
                return bytesReceived;

        }
    }
     DEBUGF(DBG_SCOPE, "bytesReceived %d ", bytesReceived);
     return bytesReceived;
}

void SkywatcherAZGTIMount::UpdateDetailedMountInformation(bool InformClient)
{
    bool BasicMountInfoHasChanged = false;

    if (std::string(BasicMountInfo[MOTOR_CONTROL_FIRMWARE_VERSION].text) != std::to_string(MCVersion))
    {
        IUSaveText(&BasicMountInfo[MOTOR_CONTROL_FIRMWARE_VERSION], std::to_string(MCVersion).c_str());
        BasicMountInfoHasChanged = true;
    }
    if (std::string(BasicMountInfo[MOUNT_CODE].text) != std::to_string(MountCode))
    {
        IUSaveText(&BasicMountInfo[MOUNT_CODE], std::to_string(MountCode).c_str());
        BasicMountInfoHasChanged = true;
    }
    if (std::string(BasicMountInfo[IS_DC_MOTOR].text) != std::to_string(IsDCMotor))
    {
        IUSaveText(&BasicMountInfo[IS_DC_MOTOR], std::to_string(IsDCMotor).c_str());
        BasicMountInfoHasChanged = true;
    }
    if (BasicMountInfoHasChanged && InformClient)
        IDSetText(&BasicMountInfoV, nullptr);

    if (MountCode == 128)
        IUSaveText(&BasicMountInfo[MOUNT_NAME], "Merlin");
    else
    if (MountCode >= 129 && MountCode <= 143)
        IUSaveText(&BasicMountInfo[MOUNT_NAME], "Az Goto");
    else
    if (MountCode >= 144 && MountCode <= 159)
        IUSaveText(&BasicMountInfo[MOUNT_NAME], "Dob Goto");
    else
    if (MountCode == 161)
        IUSaveText(&BasicMountInfo[MOUNT_NAME], "Virtuoso");
    else
    if (MountCode >= 160 && MountCode<=164)
        IUSaveText(&BasicMountInfo[MOUNT_NAME], "AllView Goto");
    else
    if (MountCode >= 165)
        IUSaveText(&BasicMountInfo[MOUNT_NAME], "AZ-GTI Goto");


    bool AxisOneInfoHasChanged = false;

    if (AxisOneInfo[MICROSTEPS_PER_REVOLUTION].value != MicrostepsPerRevolution[0])
    {
        AxisOneInfo[MICROSTEPS_PER_REVOLUTION].value = MicrostepsPerRevolution[0];
        AxisOneInfoHasChanged                        = true;
    }
    if (AxisOneInfo[STEPPER_CLOCK_FREQUENCY].value != StepperClockFrequency[0])
    {
        AxisOneInfo[STEPPER_CLOCK_FREQUENCY].value = StepperClockFrequency[0];
        AxisOneInfoHasChanged                      = true;
    }
    if (AxisOneInfo[HIGH_SPEED_RATIO].value != HighSpeedRatio[0])
    {
        AxisOneInfo[HIGH_SPEED_RATIO].value = HighSpeedRatio[0];
        AxisOneInfoHasChanged               = true;
    }
    if (AxisOneInfo[MICROSTEPS_PER_WORM_REVOLUTION].value != MicrostepsPerWormRevolution[0])
    {
        AxisOneInfo[MICROSTEPS_PER_WORM_REVOLUTION].value = MicrostepsPerWormRevolution[0];
        AxisOneInfoHasChanged                             = true;
    }
    if (AxisOneInfoHasChanged && InformClient)
        IDSetNumber(&AxisOneInfoV, nullptr);

    bool AxisOneStateHasChanged = false;
    if (AxisOneState[FULL_STOP].s != (AxesStatus[0].FullStop ? ISS_ON : ISS_OFF))
    {
        AxisOneState[FULL_STOP].s = AxesStatus[0].FullStop ? ISS_ON : ISS_OFF;
        AxisOneStateHasChanged    = true;
    }
    if (AxisOneState[SLEWING].s != (AxesStatus[0].Slewing ? ISS_ON : ISS_OFF))
    {
        AxisOneState[SLEWING].s = AxesStatus[0].Slewing ? ISS_ON : ISS_OFF;
        AxisOneStateHasChanged  = true;
    }
    if (AxisOneState[SLEWING_TO].s != (AxesStatus[0].SlewingTo ? ISS_ON : ISS_OFF))
    {
        AxisOneState[SLEWING_TO].s = AxesStatus[0].SlewingTo ? ISS_ON : ISS_OFF;
        AxisOneStateHasChanged     = true;
    }
    if (AxisOneState[SLEWING_FORWARD].s != (AxesStatus[0].SlewingForward ? ISS_ON : ISS_OFF))
    {
        AxisOneState[SLEWING_FORWARD].s = AxesStatus[0].SlewingForward ? ISS_ON : ISS_OFF;
        AxisOneStateHasChanged          = true;
    }
    if (AxisOneState[HIGH_SPEED].s != (AxesStatus[0].HighSpeed ? ISS_ON : ISS_OFF))
    {
        AxisOneState[HIGH_SPEED].s = AxesStatus[0].HighSpeed ? ISS_ON : ISS_OFF;
        AxisOneStateHasChanged     = true;
    }
    if (AxisOneState[NOT_INITIALISED].s != (AxesStatus[0].NotInitialized ? ISS_ON : ISS_OFF))
    {
        AxisOneState[NOT_INITIALISED].s = AxesStatus[0].NotInitialized ? ISS_ON : ISS_OFF;
        AxisOneStateHasChanged          = true;
    }
    if (AxisOneStateHasChanged && InformClient)
        IDSetSwitch(&AxisOneStateV, nullptr);

    bool AxisTwoInfoHasChanged = false;
    if (AxisTwoInfo[MICROSTEPS_PER_REVOLUTION].value != MicrostepsPerRevolution[1])
    {
        AxisTwoInfo[MICROSTEPS_PER_REVOLUTION].value = MicrostepsPerRevolution[1];
        AxisTwoInfoHasChanged                        = true;
    }
    if (AxisTwoInfo[STEPPER_CLOCK_FREQUENCY].value != StepperClockFrequency[1])
    {
        AxisTwoInfo[STEPPER_CLOCK_FREQUENCY].value = StepperClockFrequency[1];
        AxisTwoInfoHasChanged                      = true;
    }
    if (AxisTwoInfo[HIGH_SPEED_RATIO].value != HighSpeedRatio[1])
    {
        AxisTwoInfo[HIGH_SPEED_RATIO].value = HighSpeedRatio[1];
        AxisTwoInfoHasChanged               = true;
    }
    if (AxisTwoInfo[MICROSTEPS_PER_WORM_REVOLUTION].value != MicrostepsPerWormRevolution[1])
    {
        AxisTwoInfo[MICROSTEPS_PER_WORM_REVOLUTION].value = MicrostepsPerWormRevolution[1];
        AxisTwoInfoHasChanged                             = true;
    }
    if (AxisTwoInfoHasChanged && InformClient)
        IDSetNumber(&AxisTwoInfoV, nullptr);

    bool AxisTwoStateHasChanged = false;
    if (AxisTwoState[FULL_STOP].s != (AxesStatus[1].FullStop ? ISS_ON : ISS_OFF))
    {
        AxisTwoState[FULL_STOP].s = AxesStatus[1].FullStop ? ISS_ON : ISS_OFF;
        AxisTwoStateHasChanged    = true;
    }
    if (AxisTwoState[SLEWING].s != (AxesStatus[1].Slewing ? ISS_ON : ISS_OFF))
    {
        AxisTwoState[SLEWING].s = AxesStatus[1].Slewing ? ISS_ON : ISS_OFF;
        AxisTwoStateHasChanged  = true;
    }
    if (AxisTwoState[SLEWING_TO].s != (AxesStatus[1].SlewingTo ? ISS_ON : ISS_OFF))
    {
        AxisTwoState[SLEWING_TO].s = AxesStatus[1].SlewingTo ? ISS_ON : ISS_OFF;
        AxisTwoStateHasChanged     = true;
    }
    if (AxisTwoState[SLEWING_FORWARD].s != (AxesStatus[1].SlewingForward ? ISS_ON : ISS_OFF))
    {
        AxisTwoState[SLEWING_FORWARD].s = AxesStatus[1].SlewingForward ? ISS_ON : ISS_OFF;
        AxisTwoStateHasChanged          = true;
    }
    if (AxisTwoState[HIGH_SPEED].s != (AxesStatus[1].HighSpeed ? ISS_ON : ISS_OFF))
    {
        AxisTwoState[HIGH_SPEED].s = AxesStatus[1].HighSpeed ? ISS_ON : ISS_OFF;
        AxisTwoStateHasChanged     = true;
    }
    if (AxisTwoState[NOT_INITIALISED].s != (AxesStatus[1].NotInitialized ? ISS_ON : ISS_OFF))
    {
        AxisTwoState[NOT_INITIALISED].s = AxesStatus[1].NotInitialized ? ISS_ON : ISS_OFF;
        AxisTwoStateHasChanged          = true;
    }
    if (AxisTwoStateHasChanged && InformClient)
        IDSetSwitch(&AxisTwoStateV, nullptr);

    bool AxisOneEncoderValuesHasChanged = false;
    if ((AxisOneEncoderValues[RAW_MICROSTEPS].value != CurrentEncoders[AXIS1]) ||
        (AxisOneEncoderValues[OFFSET_FROM_INITIAL].value != CurrentEncoders[AXIS1] - ZeroPositionEncoders[AXIS1]))
    {
        AxisOneEncoderValues[RAW_MICROSTEPS].value = CurrentEncoders[AXIS1];
        AxisOneEncoderValues[MICROSTEPS_PER_ARCSEC].value = MicrostepsPerDegree[AXIS1] / 3600.0;
        AxisOneEncoderValues[OFFSET_FROM_INITIAL].value = CurrentEncoders[AXIS1] - ZeroPositionEncoders[AXIS1];
        AxisOneEncoderValues[DEGREES_FROM_INITIAL].value =
            MicrostepsToDegrees(AXIS1, CurrentEncoders[AXIS1] - ZeroPositionEncoders[AXIS1]);
        AxisOneEncoderValuesHasChanged = true;
    }
    if (AxisOneEncoderValuesHasChanged && InformClient)
        IDSetNumber(&AxisOneEncoderValuesV, nullptr);

    bool AxisTwoEncoderValuesHasChanged = false;
    if ((AxisTwoEncoderValues[RAW_MICROSTEPS].value != CurrentEncoders[AXIS2]) ||
        (AxisTwoEncoderValues[OFFSET_FROM_INITIAL].value != CurrentEncoders[AXIS2] - ZeroPositionEncoders[AXIS2]))
    {
        AxisTwoEncoderValues[RAW_MICROSTEPS].value      = CurrentEncoders[AXIS2];
        AxisTwoEncoderValues[MICROSTEPS_PER_ARCSEC].value = MicrostepsPerDegree[AXIS2] / 3600.0;
        AxisTwoEncoderValues[OFFSET_FROM_INITIAL].value = CurrentEncoders[AXIS2] - ZeroPositionEncoders[AXIS2];
        AxisTwoEncoderValues[DEGREES_FROM_INITIAL].value =
            MicrostepsToDegrees(AXIS2, CurrentEncoders[AXIS2] - ZeroPositionEncoders[AXIS2]);
        AxisTwoEncoderValuesHasChanged = true;
    }
    if (AxisTwoEncoderValuesHasChanged && InformClient)
        IDSetNumber(&AxisTwoEncoderValuesV, nullptr);
}


ln_hrz_posn SkywatcherAZGTIMount::GetAltAzPosition(double ra, double dec, double offset_in_sec)
{
    ln_lnlat_posn Location { 0, 0 };
    ln_equ_posn Eq { 0, 0 };
    ln_hrz_posn AltAz { 0, 0 };
    double JulianOffset = offset_in_sec / (24.0*60*60);

    // Set the current location
    if (IUFindSwitch(&WedgeModeSP, "WEDGE_SIMPLE")->s == ISS_OFF &&
        IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_OFF)
    {
        Location.lat = LocationN[LOCATION_LATITUDE].value;
        Location.lng = LocationN[LOCATION_LONGITUDE].value;
    } else {
        if (LocationN[LOCATION_LATITUDE].value > 0)
        {
            Location.lat = 90;
            Location.lng = 0;
        } else {
            Location.lat = -90;
            Location.lng = 0;
        }
    }
    Eq.ra  = ra*360.0 / 24.0;
    Eq.dec = dec;
    ln_get_hrz_from_equ(&Eq, &Location, ln_get_julian_from_sys()+JulianOffset, &AltAz);
    AltAz.az -= 180;
    if (AltAz.az < 0)
        AltAz.az += 360;

    return AltAz;
}

bool SkywatcherAZGTIMount::updateLocation(double latitude, double longitude, double elevation)
{
    INDI_UNUSED(elevation);
    UpdateLocation(latitude, longitude, elevation);

// INDI Longitude is 0 to 360 increasing EAST. libnova East is Positive, West is negative
    if (longitude > 180){
        longitude -= 360;
        lnobserver.lng=longitude;
    }

    lnobserver.lat=latitude;

    char l[32]={0}, L[32]={0};
    fs_sexa(l, latitude, 3, 3600);
    fs_sexa(L, longitude, 4, 3600);

    if((IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_ON)){
        SetApproximateMountAlignmentFromMountType(EQUATORIAL);
    }


    LOGF_INFO("Site location updated to Lat %.32s - Long %.32s", l, L);

    return true;
}
ln_equ_posn SkywatcherAZGTIMount::GetRaDecPosition(double alt, double az)
{
    ln_lnlat_posn Location { 0, 0 };
    ln_equ_posn Eq { 0, 0 };
    ln_hrz_posn AltAz { az, alt };

    // Set the current location
    if (IUFindSwitch(&WedgeModeSP, "WEDGE_SIMPLE")->s == ISS_OFF &&
        IUFindSwitch(&WedgeModeSP, "WEDGE_EQ")->s == ISS_OFF)
    {
        Location.lat = LocationN[LOCATION_LATITUDE].value;
        Location.lng = LocationN[LOCATION_LONGITUDE].value;
    } else {
        if (LocationN[LOCATION_LATITUDE].value > 0)
        {
            Location.lat = 90;
            Location.lng = 0;
        } else {
            Location.lat = -90;
            Location.lng = 0;
        }
    }
    AltAz.az -= 180;
    if (AltAz.az < 0)
        AltAz.az += 360;

    ln_get_equ_from_hrz(&AltAz, &Location, ln_get_julian_from_sys(), &Eq);
    Eq.ra = Eq.ra / 360.0 * 24.0;
    return Eq;
}


void SkywatcherAZGTIMount::LogMessage(const char* format, ...)
{
    if (!format || IUFindSwitch(&TrackLogModeSP, "TRACKLOG_ENABLED")->s == ISS_OFF)
        return;

    va_list Ap;
    va_start(Ap, format);

    char TempStr[512];
    std::ofstream LogFile;

    LogFile.open(TrackLogFileName.c_str(), std::ios::out | std::ios::app);
    if (!LogFile.is_open())
    {
        return;
    }
    vsnprintf(TempStr, sizeof(TempStr), format, Ap);
    LogFile << GetLogTimestamp() << " | " << TempStr << "\n";
    LogFile.close();
    va_end(Ap);
}

bool SkywatcherAZGTIMount::startTracking(){

    StartRATracking(GetRATrackRate());
    StartDETracking(GetDETrackRate());

    return true;
}
double SkywatcherAZGTIMount::GetRATrackRate()
{
    double rate = 0.0;
    ISwitch *sw;
    sw = IUFindOnSwitch(&TrackModeSP);
    if (!sw)
        return 0.0;
    if (!strcmp(sw->name, "SIDEREAL"))
    {
        rate = TRACKRATE_SIDEREAL;
    }
    else if (!strcmp(sw->name, "LUNAR"))
    {
        rate = TRACKRATE_LUNAR;
    }
    else if (!strcmp(sw->name, "SOLAR"))
    {
        rate = TRACKRATE_SOLAR;
    }
    else
        return 0.0;
//    if (RAInverted)
//        rate = -rate;
    return rate;
}

double SkywatcherAZGTIMount::GetDETrackRate()
{
    double rate = 0.0;
    ISwitch *sw;
    sw = IUFindOnSwitch(&TrackModeSP);
    if (!sw)
        return 0.0;
    if (!strcmp(sw->name, "SIDEREAL"))
    {
        rate = 0.0;
    }
    else if (!strcmp(sw->name, "LUNAR"))
    {
         rate = 0.0;
    }
    else if (!strcmp(sw->name, "SOLAR"))
    {
        rate = 0.0;
    }
    else
        return 0.0;
//    if (DEInverted)
//        rate = -rate;
    return rate;
}

bool SkywatcherAZGTIMount::startGuiding(){

// guide with 0.5x

    StartRAGuiding('2');
    StartDEGuiding('2');

    return true;
}
