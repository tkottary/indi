/*!
 * \file skywatcherAPI.cpp
 *
 * \author Roger James
 * \author Gerry Rozema
 * \author Jean-Luc Geehalel
 * \date 13th November 2013
 *
 * This file contains an implementation in C++ of the Skywatcher API.
 * It is based on work from four sources.
 * A C++ implementation of the API by Roger James.
 * The indi_eqmod driver by Jean-Luc Geehalel.
 * The synscanmount driver by Gerry Rozema.
 * The C# implementation published by Skywatcher/Synta
 */

#include "skywatcherAPI.h"

#include <cmath>
#include <iomanip>
#include <memory>
#include <thread>
#include <termios.h>
#include "indicom.h"
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#define AZGTI_TIMEOUT                  1
#define AZGTI_TIMEOUT_SETUP            1

int ttySkywatcherUdpFormat = 0;
int setupMount = 0;

void SkywatcherAPI::setup_Mount(int setUpValue)
{
    setupMount = setUpValue;
}

void SkywatcherAPI::tty_set_skywatcher_udp_format(int enabled)
{
    ttySkywatcherUdpFormat = enabled;
}

void AXISSTATUS::SetFullStop()
{
    FullStop  = true;
    SlewingTo = Slewing = false;
}

void AXISSTATUS::SetSlewing(bool forward, bool highspeed)
{
    FullStop = SlewingTo = false;
    Slewing              = true;

    SlewingForward = forward;
    HighSpeed      = highspeed;
}

void AXISSTATUS::SetSlewingTo(bool forward, bool highspeed)
{
    FullStop = Slewing = false;
    SlewingTo          = true;

    SlewingForward = forward;
    HighSpeed      = highspeed;
}

SkywatcherAPI::SkywatcherAPI()
{
    // I add an additional debug level so I can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    RadiansPerMicrostep[AXIS1] = RadiansPerMicrostep[AXIS2] = 0;
    MicrostepsPerRadian[AXIS1] = MicrostepsPerRadian[AXIS2] = 0;
    DegreesPerMicrostep[AXIS1] = DegreesPerMicrostep[AXIS2] = 0;
    MicrostepsPerDegree[AXIS1] = MicrostepsPerDegree[AXIS2] = 0;
    CurrentEncoders[AXIS1] = CurrentEncoders[AXIS2] = 0;
    PolarisPositionEncoders[AXIS1] = PolarisPositionEncoders[AXIS2] = 0;
    ZeroPositionEncoders[AXIS1] = ZeroPositionEncoders[AXIS2] = 0;
    SlewingSpeed[AXIS1] = SlewingSpeed[AXIS2] = 0;
}

unsigned long SkywatcherAPI::BCDstr2long(std::string &String)
{
    if (String.size() != 6)
    {
        return 0;
    }
    unsigned long value = 0;

#define HEX(c) (((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)

    value = HEX(String[4]);
    value <<= 4;
    value |= HEX(String[5]);
    value <<= 4;
    value |= HEX(String[2]);
    value <<= 4;
    value |= HEX(String[3]);
    value <<= 4;
    value |= HEX(String[0]);
    value <<= 4;
    value |= HEX(String[1]);

#undef HEX

    return value;
}

unsigned long SkywatcherAPI::Highstr2long(std::string &String)
{
    if (String.size() < 2)
    {
        return 0;
    }
    unsigned long res = 0;

#define HEX(c) (((c) < 'A') ? ((c) - '0') : ((c) - 'A') + 10)

    res = HEX(String[0]);
    res <<= 4;
    res |= HEX(String[1]);

#undef HEX

    return res;
}

 bool SkywatcherAPI::CheckIfDCMotor()
{
    MYDEBUG(DBG_SCOPE, "CheckIfDCMotor");
    int  nbytes_read    = 0;
    int  nbytes_written = 0;

    std::string SendBuffer;

    SendBuffer.push_back(':');
    SendBuffer.push_back('\r');
    int  errcode = 0;
    char errmsg[MAXRBUF];
    char response[16];
        memset(response, 0, sizeof(response));
        tcflush(MyPortFD, TCIFLUSH);

        if ((errcode = tty_write(MyPortFD, SendBuffer.c_str(), 1, &nbytes_written)) != TTY_OK)
        {
            tty_error_msg(errcode, errmsg, MAXRBUF);
            return false;
        }

    tty_read(MyPortFD, response, 9, AZGTI_TIMEOUT, &nbytes_read);


    if (nbytes_read > 0)
    {
        tcflush(MyPortFD, TCIFLUSH);

        if (response[0] == '!')
        {
            MYDEBUG(DBG_SCOPE, "Bad reponse from mount");
            IsDCMotor=false;
            return false;
        }else if (':' == response[0]){
               IsDCMotor=true;
               return true;
        }
        return true;
    }

    return false;
}

bool SkywatcherAPI::IsVirtuosoMount() const
{
    return MountCode >= 0x90;
}

bool SkywatcherAPI::IsMerlinMount() const
{
    return MountCode >= 0x80 && MountCode < 0x90;
}
bool SkywatcherAPI::IsAZGTIMount() const
{
    return MountCode == 0xA5;
}

long SkywatcherAPI::DegreesPerSecondToClocksTicksPerMicrostep(AXISID Axis, double DegreesPerSecond)
{
    double MicrostepsPerSecond = DegreesPerSecond * MicrostepsPerDegree[Axis];

    return long((double(StepperClockFrequency[Axis]) / MicrostepsPerSecond));
}

long SkywatcherAPI::DegreesToMicrosteps(AXISID Axis, double AngleInDegrees)
{
    return (long)(AngleInDegrees * MicrostepsPerDegree[(int)Axis]);
}

bool SkywatcherAPI::GetEncoder(AXISID Axis)
{
//    MYDEBUG(DBG_SCOPE, "GetEncoder");
    std::string Parameters, Response;
    if (!TalkWithAxis(Axis, 'j', Parameters, Response))
        return false;

    long Microsteps            = BCDstr2long(Response);
    CurrentEncoders[(int)Axis] = Microsteps;

    return true;
}

bool SkywatcherAPI::GetHighSpeedRatio(AXISID Axis)
{
    MYDEBUG(DBG_SCOPE, "GetHighSpeedRatio");
    std::string Parameters, Response;

    if (!TalkWithAxis(Axis, 'g', Parameters, Response))
        return false;

    unsigned long highSpeedRatio = Highstr2long(Response);
    HighSpeedRatio[(int)Axis]    = highSpeedRatio;

    return true;
}

void SkywatcherAPI::InquireFeatures()
{

        GetFeature(AXIS1, '0');
}

void SkywatcherAPI::GetFeature(AXISID axis, unsigned long command)
{
    unsigned long features = 0;
    MYDEBUG(DBG_SCOPE, "GetFeature");
    std::string Parameters, Response;

    Parameters.push_back(command);

    Parameters.push_back('1');
    Parameters.push_back('0');
    Parameters.push_back('0');
    Parameters.push_back('0');
    Parameters.push_back('0');


    //IDLog("Setting target for axis %c  to %d\n", AxisCmd[axis], increment);
    TalkWithAxis(axis, 'q', Parameters, Response);

        features= BCDstr2long(Response);
      //  bool isAZEQ  = features & 0x00000008;
//        bool inPPECTraining         = features & 0x00000010;
//        bool inPPEC                 = features & 0x00000020;
//        bool hasEncoder             = features & 0x00000001;
//        bool hasPPEC                = features & 0x00000002;
//        bool hasHomeIndexer         = features & 0x00000004;
//        bool hasPolarLed            = features & 0x00001000;
//        bool hasCommonSlewStart     = features & 0x00002000; // supports :J3
//        bool hasHalfCurrentTracking = features & 0x00004000;

//    MYDEBUGF(DBG_SCOPE, "GetFeature isAZEQ  %ld",isAZEQ);
//    MYDEBUGF(DBG_SCOPE, "GetFeaturei nPPECTraining  %ld",inPPECTraining);

//    MYDEBUGF(DBG_SCOPE, "GetFeature inPPEC  %ld",inPPEC);
//    MYDEBUGF(DBG_SCOPE, "GetFeaturei hasEncoder  %ld",hasEncoder);
//    MYDEBUGF(DBG_SCOPE, "GetFeature hasPPEC  %ld",hasPPEC);
//    MYDEBUGF(DBG_SCOPE, "GetFeaturei hasHomeIndexer  %ld",hasHomeIndexer);
//    MYDEBUGF(DBG_SCOPE, "GetFeature hasPolarLed  %ld",hasPolarLed);
//    MYDEBUGF(DBG_SCOPE, "GetFeaturei hasCommonSlewStart  %ld",hasCommonSlewStart);
//     MYDEBUGF(DBG_SCOPE, "GetFeaturei hasHalfCurrentTracking  %ld",hasHalfCurrentTracking);
}


void SkywatcherAPI::SetFeature(AXISID axis, char command){

    MYDEBUG(DBG_SCOPE, "SetFeature");
    std::string Parameters, Response;
\
    Parameters.push_back('0');
    Parameters.push_back(command);
    Parameters.push_back('0');
    Parameters.push_back('0');
    Parameters.push_back('0');
    Parameters.push_back('0');


    //IDLog("Setting target for axis %c  to %d\n", AxisCmd[axis], increment);
    TalkWithAxis(axis, 'W', Parameters, Response);

}

void SkywatcherAPI::TurnEncoder(AXISID axis, bool on)
{
   char  command;
    if (on)
        command = '4';
    else
        command = '5';

    SetFeature(axis, command);
}

void SkywatcherAPI::TurnRAEncoder(bool on)
{
    TurnEncoder(AXIS1, on);
}

void SkywatcherAPI::TurnDEEncoder(bool on)
{
    TurnEncoder(AXIS2, on);
}


double SkywatcherAPI::get_min_rate()
{
    return MIN_RATE;
}

double SkywatcherAPI::get_max_rate()
{
    return MAX_RATE;
}

bool SkywatcherAPI::GetMicrostepsPerRevolution(AXISID Axis)
{

    //nanosleep(&timeout, nullptr);
    MYDEBUG(DBG_SCOPE, "GetMicrostepsPerRevolution");
    std::string Parameters, Response;

    if (!TalkWithAxis(Axis, 'a', Parameters, Response))
        return false;

    long tmpMicrostepsPerRevolution = BCDstr2long(Response);

    // There is a bug in the earlier version firmware(Before 2.00) of motor controller MC001.
    // Overwrite the MicrostepsPerRevolution reported by the MC for 80GT mount and 114GT mount.
    // kecsap: The Merlin mounts use the same mount code and it brakes the operation.
//    if (MountCode == GT)
//        tmpMicrostepsPerRevolution = 0x162B97; // for 80GT mount
    if (MountCode == _114GT)
        tmpMicrostepsPerRevolution = 0x205318; // for 114GT mount

    if (IsMerlinMount())
        tmpMicrostepsPerRevolution = (long)((double)tmpMicrostepsPerRevolution*0.655);

    MicrostepsPerRevolution[(int)Axis] = tmpMicrostepsPerRevolution;

    MicrostepsPerRadian[(int)Axis] = tmpMicrostepsPerRevolution / (2 * M_PI);
    RadiansPerMicrostep[(int)Axis] = 2 * M_PI / tmpMicrostepsPerRevolution;
    MicrostepsPerDegree[(int)Axis] = tmpMicrostepsPerRevolution / 360.0;
    DegreesPerMicrostep[(int)Axis] = 360.0 / tmpMicrostepsPerRevolution;

    MYDEBUGF(DBG_SCOPE, "Axis %d: %lf microsteps/degree, %lf microsteps/arcsec", Axis,
             (double)tmpMicrostepsPerRevolution / 360.0, (double)tmpMicrostepsPerRevolution / 360.0 / 60 / 60);

    return true;
}

bool SkywatcherAPI::GetMicrostepsPerWormRevolution(AXISID Axis)
{
    MYDEBUG(DBG_SCOPE, "GetMicrostepsPerWormRevolution");
    std::string Parameters, Response;

    if (!TalkWithAxis(Axis, 's', Parameters, Response))
        return false;

    MicrostepsPerWormRevolution[(int)Axis] = BCDstr2long(Response);

    return true;
}

bool SkywatcherAPI::GetMotorBoardVersion(AXISID Axis)
{
    MYDEBUG(DBG_SCOPE, "GetMotorBoardVersion");
    std::string Parameters, Response;

    if (!TalkWithAxis(Axis, 'e', Parameters, Response))
        return false;

    unsigned long tmpMCVersion = BCDstr2long(Response);

     MCVersion = ((tmpMCVersion & 0xFF) << 16) | ((tmpMCVersion & 0xFF00)) | ((tmpMCVersion & 0xFF0000) >> 16);
     MountCode = MCVersion & 0xFF;
     MYDEBUGF(DBG_SCOPE, "GetMotorBoardVersion MCVersion %ld MountCode %ld", MCVersion, MountCode);
     return true;
}

SkywatcherAPI::PositiveRotationSense_t SkywatcherAPI::GetPositiveRotationDirection(AXISID Axis)
{
    INDI_UNUSED(Axis);
    if (MountCode == _114GT)
        return CLOCKWISE;

    return ANTICLOCKWISE;
}

bool SkywatcherAPI::GetStepperClockFrequency(AXISID Axis)
{
    MYDEBUG(DBG_SCOPE, "GetStepperClockFrequency");
    std::string Parameters, Response;

    if (!TalkWithAxis(Axis, 'b', Parameters, Response))
        return false;

    StepperClockFrequency[(int)Axis] = BCDstr2long(Response);


    return true;
}

bool SkywatcherAPI::GetStatus(AXISID Axis)
{
//    MYDEBUG(DBG_SCOPE, "GetStatus");
    std::string Parameters, Response;

    if (!TalkWithAxis(Axis, 'f', Parameters, Response))
        return false;

    if ((Response[1] & 0x01) != 0)
    {
        // Axis is running
        AxesStatus[(int)Axis].FullStop = false;
        if ((Response[0] & 0x01) != 0)
        {
            AxesStatus[(int)Axis].Slewing   = true; // Axis in slewing(AstroMisc speed) mode.
            AxesStatus[(int)Axis].SlewingTo = false;
        }
        else
        {
            AxesStatus[(int)Axis].SlewingTo = true; // Axis in SlewingTo mode.
            AxesStatus[(int)Axis].Slewing   = false;
        }
    }
    else
    {
        // SlewTo Debugging
        if (AxesStatus[(int)Axis].SlewingTo)
        {
            // If the mount was doing a slew to
            GetEncoder(Axis);
//            MYDEBUGF(INDI::Logger::DBG_SESSION,
//                     "Axis %s SlewTo complete - offset to target %ld microsteps %lf arc seconds "
//                     "LastSlewToTarget %ld CurrentEncoder %ld",
//                     Axis == AXIS1 ? "AXIS1" : "AXIS2", LastSlewToTarget[Axis] - CurrentEncoders[Axis],
//                     MicrostepsToDegrees(Axis, LastSlewToTarget[Axis] - CurrentEncoders[Axis]) * 3600,
//                     LastSlewToTarget[Axis], CurrentEncoders[Axis]);
        }

        AxesStatus[(int)Axis].FullStop  = true; // FullStop = 1;	// Axis is fully stop.
        AxesStatus[(int)Axis].Slewing   = false;
        AxesStatus[(int)Axis].SlewingTo = false;
    }

    if ((Response[0] & 0x02) == 0)
        AxesStatus[(int)Axis].SlewingForward = true; // Angle increase = 1;
    else
        AxesStatus[(int)Axis].SlewingForward = false;

    if ((Response[0] & 0x04) != 0)
        AxesStatus[(int)Axis].HighSpeed = true; // HighSpeed running mode = 1;
    else
        AxesStatus[(int)Axis].HighSpeed = false;

    if ((Response[2] & 1) == 0)
        AxesStatus[(int)Axis].NotInitialized = true; // MC is not initialized.
    else
        AxesStatus[(int)Axis].NotInitialized = false;

    return true;
}

// Set initialization done ":F3", where '3'= Both CH1 and CH2.
bool SkywatcherAPI::InitializeMC()
{
    MYDEBUG(DBG_SCOPE, "InitializeMC");
    std::string Parameters, Response;

    if (!TalkWithAxis(AXIS1, 'F', Parameters, Response))
        return false;
    if (!TalkWithAxis(AXIS2, 'F', Parameters, Response))
        return false;
    return true;
}

bool SkywatcherAPI:: InitMount(bool recover)
{

        MYDEBUG(DBG_SCOPE, "InitMount2");

    if (!GetMotorBoardVersion(AXIS1))
             //return false;

        MountCode = MCVersion & 0xFF;
        CheckIfDCMotor();


    // Disable EQ mounts
    //    if (MountCode < 0x80)
    //        return false;

        if(IsAZGTIMount()){
            IsDCMotor = true;
        }

    //// NOTE: Simulator settings, Mount dependent Settings

        // Inquire Gear Rate
        GetMicrostepsPerRevolution(AXIS1);
        GetMicrostepsPerRevolution(AXIS2);

        // Get stepper clock frequency
        GetStepperClockFrequency(AXIS1);
        GetStepperClockFrequency(AXIS2);


        // Inquire motor high speed ratio
        GetHighSpeedRatio(AXIS1);
        GetHighSpeedRatio(AXIS2);

    // Inquire PEC period
    // DC motor controller does not support PEC
    if (!IsDCMotor)
    {
        GetMicrostepsPerWormRevolution(AXIS1);
        GetMicrostepsPerWormRevolution(AXIS2);
    }

    // Inquire Axis Position
        GetEncoder(AXIS1);
        GetEncoder(AXIS2);
        //return false;

        MYDEBUGF(DBG_SCOPE, "Encoders before init Axis1 %ld Axis2 %ld", CurrentEncoders[AXIS1], CurrentEncoders[AXIS2]);

    // Set initial axis positions
    // These are used to define the arbitrary zero position vector for the axis
    if (!recover)
    {
        PolarisPositionEncoders[AXIS1] = CurrentEncoders[AXIS1];
        PolarisPositionEncoders[AXIS2] = CurrentEncoders[AXIS2];
        ZeroPositionEncoders[AXIS1] = PolarisPositionEncoders[AXIS1];
        ZeroPositionEncoders[AXIS2] = PolarisPositionEncoders[AXIS2];
    }

    if (!InitializeMC())
        //return false;


    // These two LowSpeedGotoMargin are calculate from slewing for 5 seconds in 128x sidereal rate
    LowSpeedGotoMargin[(int)AXIS1] = (long)(640 * SIDEREALRATE * MicrostepsPerRadian[(int)AXIS1]);
    LowSpeedGotoMargin[(int)AXIS2] = (long)(640 * SIDEREALRATE * MicrostepsPerRadian[(int)AXIS2]);

    TurnRAEncoder(false);
    TurnDEEncoder(false);

       return true;
}

bool SkywatcherAPI::InstantStop(AXISID Axis)
{
    // Request a slow stop
    MYDEBUG(DBG_SCOPE, "InstantStop");
    std::string Parameters, Response;
    if (!TalkWithAxis(Axis, 'L', Parameters, Response))
        return false;
    AxesStatus[(int)Axis].SetFullStop();
    return true;
}

void SkywatcherAPI::Long2BCDstr(long Number, std::string &String)
{
    std::stringstream Temp;
    Temp << std::hex << std::setfill('0') << std::uppercase << std::setw(2) << (Number & 0xff) << std::setw(2)
         << ((Number & 0xff00) >> 8) << std::setw(2) << ((Number & 0xff0000) >> 16);
    String = Temp.str();
}

double SkywatcherAPI::MicrostepsToDegrees(AXISID Axis, long Microsteps)
{
    return Microsteps * DegreesPerMicrostep[(int)Axis];
}

double SkywatcherAPI::MicrostepsToRadians(AXISID Axis, long Microsteps)
{
    return Microsteps * RadiansPerMicrostep[(int)Axis];
}

void SkywatcherAPI::PrepareForSlewing(AXISID Axis, double Speed)
{
    // Update the axis status
    if (!GetStatus(Axis))
        return;

    if (!AxesStatus[Axis].FullStop)
    {
        // Axis is running
        if ((AxesStatus[Axis].SlewingTo)                            // slew to (GOTO) in progress
            || (AxesStatus[Axis].HighSpeed)                         // currently high speed slewing
            || (std::abs(Speed) >= LOW_SPEED_MARGIN)                // I am about to request high speed
            || ((AxesStatus[Axis].SlewingForward) && (Speed < 0))   // Direction change
            || (!(AxesStatus[Axis].SlewingForward) && (Speed > 0))) // Direction change
        {
            // I need to stop the axis first
            SlowStop(Axis);
        }
        else
            return; // NO need change motion mode

        // Horrible bit A POLLING LOOP !!!!!!!!!!
        while (true)
        {
            // Update status
            GetStatus(Axis);

            if (AxesStatus[Axis].FullStop)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 1/10 second
        }
    }

    char Direction;
    if (Speed > 0.0)
        Direction = '0';
    else
    {
        Direction = '1';
        Speed     = -Speed;
    }

    if (Speed > LOW_SPEED_MARGIN)
        SetMotionMode(Axis, '3', Direction);
    else
        SetMotionMode(Axis, '1', Direction);
}

long SkywatcherAPI::RadiansPerSecondToClocksTicksPerMicrostep(AXISID Axis, double RadiansPerSecond)
{
    double MicrostepsPerSecond = RadiansPerSecond * MicrostepsPerRadian[Axis];

    return long((double(StepperClockFrequency[Axis]) / MicrostepsPerSecond));
}

long SkywatcherAPI::RadiansToMicrosteps(AXISID Axis, double AngleInRadians)
{
    return (long)(AngleInRadians * MicrostepsPerRadian[(int)Axis]);
}

bool SkywatcherAPI::SetGuideSpeed(AXISID Axis, char speed)
{
    MYDEBUG(DBG_SCOPE, "SetGuideSpeed");
    std::string Parameters, Response;
    Parameters.push_back(speed);
    return TalkWithAxis(Axis, 'P', Parameters, Response);
}

bool SkywatcherAPI::SetEncoder(AXISID Axis, long Microsteps)
{
    MYDEBUG(DBG_SCOPE, "SetEncoder");
    std::string Parameters, Response;

    Long2BCDstr(Microsteps, Parameters);

    return TalkWithAxis(Axis, 'L', Parameters, Response);
}

bool SkywatcherAPI::SetGotoTargetOffset(AXISID Axis, long OffsetInMicrosteps)
{
//    MYDEBUG(DBG_SCOPE, "SetGotoTargetOffset");
    std::string Parameters, Response;

    Long2BCDstr(OffsetInMicrosteps, Parameters);

    return TalkWithAxis(Axis, 'H', Parameters, Response);
}

/// Func - 0 High speed slew to mode (goto)
/// Func - 1 Low speed slew mode
/// Func - 2 Low speed slew to mode (goto)
/// Func - 3 High speed slew mode
bool SkywatcherAPI::SetMotionMode(AXISID Axis, char Func, char Direction)
{
    MYDEBUG(DBG_SCOPE, "SetMotionMode");
    std::string Parameters, Response;

    Parameters.push_back(Func);
    Parameters.push_back(Direction);

    return TalkWithAxis(Axis, 'G', Parameters, Response);
}

bool SkywatcherAPI::SetClockTicksPerMicrostep(AXISID Axis, long ClockTicksPerMicrostep)
{
    MYDEBUG(DBG_SCOPE, "SetClockTicksPerMicrostep");
    std::string Parameters, Response;

    Long2BCDstr(ClockTicksPerMicrostep, Parameters);

    return TalkWithAxis(Axis, 'I', Parameters, Response);
}

bool SkywatcherAPI::SetSlewModeDeccelerationRampLength(AXISID Axis, long Microsteps)
{
//    MYDEBUG(DBG_SCOPE, "SetSlewModeDeccelerationRampLength");
    std::string Parameters, Response;

    Long2BCDstr(Microsteps, Parameters);

    return TalkWithAxis(Axis, 'U', Parameters, Response);
}

bool SkywatcherAPI::SetSlewToModeDeccelerationRampLength(AXISID Axis, long Microsteps)
{
//    MYDEBUG(DBG_SCOPE, "SetSlewToModeDeccelerationRampLength");
    std::string Parameters, Response;

    Long2BCDstr(Microsteps, Parameters);

    return TalkWithAxis(Axis, 'M', Parameters, Response);
}

bool SkywatcherAPI::SetSwitch(bool OnOff)
{
    MYDEBUG(DBG_SCOPE, "SetSwitch");
    std::string Parameters, Response;

    if (OnOff)
        Parameters = "1";
    else
        Parameters = "0";

    return TalkWithAxis(AXIS1, 'O', Parameters, Response);
}

void SkywatcherAPI::Slew(AXISID Axis, double SpeedInRadiansPerSecond, bool IgnoreSilentMode)
{
    MYDEBUGF(DBG_SCOPE, "Slew axis: %d speed: %1.6f", (int)Axis, SpeedInRadiansPerSecond);
    // Clamp to MAX_SPEED
    if (SpeedInRadiansPerSecond > MAX_SPEED)
        SpeedInRadiansPerSecond = MAX_SPEED;
    else if (SpeedInRadiansPerSecond < -MAX_SPEED)
        SpeedInRadiansPerSecond = -MAX_SPEED;

    double InternalSpeed = SpeedInRadiansPerSecond;

    if (std::abs(InternalSpeed) <= SIDEREALRATE / 1000.0)
    {
        SlowStop(Axis);
        return;
    }

    // Stop motor and set motion mode if necessary
    PrepareForSlewing(Axis, InternalSpeed);

    bool Forward;
    if (InternalSpeed > 0.0)
        Forward = true;
    else
    {
        InternalSpeed = -InternalSpeed;
        Forward       = false;
    }

    bool HighSpeed = false;

    if (InternalSpeed > LOW_SPEED_MARGIN && (IgnoreSilentMode || !SilentSlewMode))
    {
        InternalSpeed = InternalSpeed / (double)HighSpeedRatio[Axis];
        HighSpeed     = true;
    }
    long SpeedInt = RadiansPerSecondToClocksTicksPerMicrostep(Axis, InternalSpeed);
    if ((MCVersion == 0x010600) || (MCVersion == 0x0010601)) // Cribbed from Mount_Skywatcher.cs
        SpeedInt -= 3;
    if (SpeedInt < 6)
        SpeedInt = 6;
    SetClockTicksPerMicrostep(Axis, SpeedInt);

    StartMotion(Axis);

    AxesStatus[Axis].SetSlewing(Forward, HighSpeed);
    SlewingSpeed[Axis] = SpeedInRadiansPerSecond;
}

void SkywatcherAPI::SlewTo(AXISID Axis, long OffsetInMicrosteps, bool verbose)
{
    if (verbose)
    {
        MYDEBUGF(INDI::Logger::DBG_SESSION, "SlewTo axis: %d offset: %ld", (int)Axis, OffsetInMicrosteps);
    }
    if (0 == OffsetInMicrosteps)
        // Nothing to do
        return;

    // Debugging
    LastSlewToTarget[Axis] = CurrentEncoders[Axis] + OffsetInMicrosteps;
    if (verbose)
    {
        MYDEBUGF(INDI::Logger::DBG_SESSION, "SlewTo axis %d Offset %ld CurrentEncoder %ld SlewToTarget %ld", Axis,
                 OffsetInMicrosteps, CurrentEncoders[Axis], LastSlewToTarget[Axis]);
    }

    char Direction;
    bool Forward;

    if (OffsetInMicrosteps > 0)
    {
        Forward   = true;
        Direction = '0';
    }
    else
    {
        Forward            = false;
        Direction          = '1';
        OffsetInMicrosteps = -OffsetInMicrosteps;
    }

    bool HighSpeed = false;

    if (OffsetInMicrosteps > LowSpeedGotoMargin[Axis] && !SilentSlewMode)
        HighSpeed = true;

    if (!GetStatus(Axis))
        return;

    if (!AxesStatus[Axis].FullStop)
    {
        // Axis is running
        if ((AxesStatus[Axis].SlewingTo)                        // slew to (GOTO) in progress
            || (AxesStatus[Axis].HighSpeed)                     // currently high speed slewing
            || HighSpeed                                        // I am about to request high speed
            || ((AxesStatus[Axis].SlewingForward) && !Forward)  // Direction change
            || (!(AxesStatus[Axis].SlewingForward) && Forward)) // Direction change
        {
            // I need to stop the axis first
            SlowStop(Axis);
            // Horrible bit A POLLING LOOP !!!!!!!!!!
            while (true)
            {
                // Update status
                GetStatus(Axis);

                if (AxesStatus[Axis].FullStop)
                    break;

                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep for 1/10 second
            }
        }
    }

    if (HighSpeed)
        SetMotionMode(Axis, '0', Direction);
    else
        SetMotionMode(Axis, '2', Direction);

    SetGotoTargetOffset(Axis, OffsetInMicrosteps);

    if (HighSpeed)
        SetSlewToModeDeccelerationRampLength(Axis, OffsetInMicrosteps > 3200 ? 3200 : OffsetInMicrosteps);
    else
        SetSlewToModeDeccelerationRampLength(Axis, OffsetInMicrosteps > 200 ? 200 : OffsetInMicrosteps);
    StartMotion(Axis);

    AxesStatus[Axis].SetSlewingTo(Forward, HighSpeed);
}

bool SkywatcherAPI::SlowStop(AXISID Axis)
{
    // Request a slow stop
//    MYDEBUG(DBG_SCOPE, "SlowStop");
    std::string Parameters, Response;

    return TalkWithAxis(Axis, 'K', Parameters, Response);
}

bool SkywatcherAPI::StartMotion(AXISID Axis)
{
//    MYDEBUG(DBG_SCOPE, "StartMotion");
    std::string Parameters, Response;

    return TalkWithAxis(Axis, 'J', Parameters, Response);
}


bool SkywatcherAPI::TalkWithAxis(AXISID Axis, char Command, std::string &cmdDataStr, std::string &responseStr)
{
    //MYDEBUGF(DBG_SCOPE, "TalkWithAxis Axis %s Command %c Data (%s)", Axis == AXIS1 ? "AXIS1" : "AXIS2", Command,
      //       cmdDataStr.c_str());
    int udpTimeOutinSec =0;
    if(setupMount==1){
        udpTimeOutinSec =AZGTI_TIMEOUT_SETUP;
    }else{
        udpTimeOutinSec=AZGTI_TIMEOUT;
    }

    int  nbytes_read    = 0;
    int  nbytes_written = 0;

    std::string SendBuffer;
    bool StartReading   = false;
    bool EndReading     = false;
    bool mount_response = false;

    SendBuffer.push_back(':');
    SendBuffer.push_back(Command);
    SendBuffer.push_back(Axis == AXIS1 ? '1' : '2');
    SendBuffer.append(cmdDataStr);
    SendBuffer.push_back('\r');
    if(ttySkywatcherUdpFormat == 0){

    skywatcher_tty_write(MyPortFD, SendBuffer.c_str(), SendBuffer.size(), &nbytes_written);

    while (!EndReading)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        char c;

        int rc = skywatcher_tty_read(MyPortFD, &c, 1, 10, &nbytes_read);
        if ((rc != TTY_OK) || (nbytes_read != 1))
            return false;

        if ((c == '=') || (c == '!'))
        {
            mount_response = (c == '=');
            StartReading = true;
            continue;
        }

        if ((c == '\r') && StartReading)
        {
            EndReading = true;
            continue;
        }

        if (StartReading)
            responseStr.push_back(c);
    }

    }else if(ttySkywatcherUdpFormat==1){


        int sockbufsize = 2048;
        timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 50000;
        setsockopt(MyPortFD, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
        setsockopt(MyPortFD, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(struct timeval));
        setsockopt(MyPortFD, SOL_SOCKET, SO_RCVBUF,(char *)&sockbufsize,  (int)sizeof(sockbufsize));
        setsockopt(MyPortFD, SOL_SOCKET, SO_SNDBUF,(char *)&sockbufsize,  (int)sizeof(sockbufsize));

            char cmd[11];
            int  errcode = 0;
            char errmsg[MAXRBUF];
            //char response[16];
            char response[2048];
                memset(response, 0, sizeof(response));
                snprintf(cmd, 11, "%s",SendBuffer.c_str());

                //MYDEBUGF(DBG_SCOPE, "TalkWithAxis comand is %s", cmd);


                tcflush(MyPortFD, TCIFLUSH);

                if ((errcode = tty_write(MyPortFD, cmd, strlen(cmd), &nbytes_written)) != TTY_OK)
                {
                    tty_error_msg(errcode, errmsg, MAXRBUF);
                    MYDEBUGF(DBG_SCOPE, "TalkWithAxis tty_error_msg  %s", errmsg);
                    return false;
                }

            tty_read(MyPortFD, response, 9, udpTimeOutinSec, &nbytes_read);


            if (nbytes_read > 0)
            {
                //MYDEBUGF(DBG_SCOPE, "TalkWithAxis Final Response before trimming %s", response);
                tcflush(MyPortFD, TCIFLUSH);

                responseStr = response;
                if (response[0] == '!')
                {
                    MYDEBUG(DBG_SCOPE, "Bad reponse from mount");
                    return false;
                }else{
                        responseStr.erase(0, 1);
                        responseStr.erase(responseStr.size()-1);
                }
                //MYDEBUGF(DBG_SCOPE, "TalkWithAxis Final Response for %s is --->  %s", cmd,response);
                return true;
            }else{
                MYDEBUG(DBG_SCOPE, "no response from mount");
                //tty_read(MyPortFD, response, 9, 1.0, &nbytes_read);
            }

            return false;

    }else{
    char response[2048];
    memset(response, 0, sizeof(response));
        skywatcher_tty_write(MyPortFD, SendBuffer.c_str(), SendBuffer.size(), &nbytes_written);
        int rc = skywatcher_tty_read(MyPortFD, response, 1, 10, &nbytes_read);
       if(rc >1){
        responseStr =response;
        responseStr.erase(nbytes_read-2);
       }else{

           //may be retry?

           return false;
       }
    }

 return true;
}

bool SkywatcherAPI::IsInMotion(AXISID Axis)
{
    MYDEBUG(DBG_SCOPE, "IsInMotion");

    return AxesStatus[(int)Axis].Slewing || AxesStatus[(int)Axis].SlewingTo;
}

void SkywatcherAPI::SetRARate(double rate)
{
    double absrate       = fabs(rate);
    unsigned long period = 0;
    bool useHighspeed    = false;
    SkywatcherAxisStatus newstatus;

   char motion,direction;


    //LOGF_DEBUG("%s() : rate = %g", __FUNCTION__, rate);

    if ((absrate < get_min_rate()) || (absrate > get_max_rate()))
    {
        //throw EQModError(EQModError::ErrInvalidParameter, s  "Speed rate out of limits: %.2fx Sidereal (min=%.2f, max=%.2f)", absrate, MIN_RATE, MAX_RATE);
    }
    //if (MountCode != 0xF0) {
    if (absrate > SKYWATCHER_LOWSPEED_RATE)
    {
        absrate      = absrate / HighSpeedRatio[AXIS1];
        useHighspeed = true;
    }
    //}//StepperClockFrequency MicrostepsPerRevolution

    period              = (long)(((SKYWATCHER_STELLAR_DAY * (double)StepperClockFrequency[AXIS1]) / (double)MicrostepsPerRevolution[AXIS1]) / absrate);

    newstatus.direction = ((rate >= 0.0) ? FORWARD : BACKWARD);
    if (newstatus.direction == FORWARD)
       direction = '0';
    else
        direction = '1';

    //newstatus.slewmode=RAStatus.slewmode;
    newstatus.slewmode = SLEW;
    if (useHighspeed){
        newstatus.speedmode = HIGHSPEED;
        motion = '3';
    }
    else{
        motion = '1';
        newstatus.speedmode = LOWSPEED;
    }
    if (IsInMotion(AXIS1))
    {
        //if (newstatus.speedmode != RAStatus.speedmode)
            //throw EQModError(EQModError::ErrInvalidParameter, "Can not change rate while motor is running (speedmode differs).");
        //if (newstatus.direction != RAStatus.direction)
            //throw EQModError(EQModError::ErrInvalidParameter,"Can not change rate while motor is running (direction differs).");
    }

    SetMotionMode(AXIS1, motion,direction);
    SetClockTicksPerMicrostep(AXIS1, period);

}


void SkywatcherAPI::SetDERate(double rate)
{
    double absrate       = fabs(rate);
    unsigned long period = 0;
    bool useHighspeed    = false;
    SkywatcherAxisStatus newstatus;

    //LOGF_DEBUG("%s() : rate = %g", __FUNCTION__, rate);

    if ((absrate < get_min_rate()) || (absrate > get_max_rate()))
    {
        //throw EQModError(EQModError::ErrInvalidParameter, "Speed rate out of limits: %.2fx Sidereal (min=%.2f, max=%.2f)", absrate, MIN_RATE, MAX_RATE);
    }
    //if (MountCode != 0xF0) {
    if (absrate > SKYWATCHER_LOWSPEED_RATE)
    {
        absrate      = absrate / HighSpeedRatio[AXIS2];
        useHighspeed = true;
    }
    //}
    period              = (long)(((SKYWATCHER_STELLAR_DAY * (double)StepperClockFrequency[AXIS2]) / (double)MicrostepsPerRevolution[AXIS2]) / absrate);
    newstatus.direction = ((rate >= 0.0) ? FORWARD : BACKWARD);
    //newstatus.slewmode=DEStatus.slewmode;
    newstatus.slewmode = SLEW;
    if (useHighspeed)
        newstatus.speedmode = HIGHSPEED;
    else
        newstatus.speedmode = LOWSPEED;
    if (IsInMotion(AXIS2))
    {
       // if (newstatus.speedmode != DEStatus.speedmode)
            //throw EQModError(EQModError::ErrInvalidParameter,  "Can not change rate while motor is running (speedmode differs).");
        //if (newstatus.direction != DEStatus.direction)
            //throw EQModError(EQModError::ErrInvalidParameter,"Can not change rate while motor is running (direction differs).");
    }

    //StartMotion(AXIS2, newstatus);
        SetMotionMode(AXIS2, newstatus.speedmode, newstatus.direction);
        SetClockTicksPerMicrostep(AXIS2, period);
}

void SkywatcherAPI::StartRATracking(double trackspeed)
{
    double rate;
    if (trackspeed != 0.0)
        rate = trackspeed / SKYWATCHER_STELLAR_SPEED;
    else
        rate = 0.0;
    //LOGF_DEBUG("%s() : trackspeed = %g arcsecs/s, computed rate = %g", __FUNCTION__, trackspeed, rate);
    if (rate != 0.0)
    {
        SetRARate(rate);
        if (!IsInMotion(AXIS1))
            StartMotion(AXIS1);
    }
    else
        SlowStop(AXIS1);
}

void SkywatcherAPI::StartDETracking(double trackspeed)
{
    double rate;
    if (trackspeed != 0.0)
        rate = trackspeed / SKYWATCHER_STELLAR_SPEED;
    else
        rate = 0.0;
    //LOGF_DEBUG("%s() : trackspeed = %g arcsecs/s, computed rate = %g", __FUNCTION__, trackspeed,rate);
    if (rate != 0.0)
    {
        SetDERate(rate);
        if (!IsInMotion(AXIS2))
            StartMotion(AXIS2);
    }
    else
        SlowStop(AXIS2);
}


void SkywatcherAPI::StartRAGuiding(char  trackspeed)
{

    SetGuideSpeed(AXIS1,trackspeed);
}


void SkywatcherAPI::StartDEGuiding(char  trackspeed)
{

    SetGuideSpeed(AXIS2,trackspeed);
}
