#pragma once
#include "skywatcherAPI.h"
#include "inditelescope.h"
#include "indiguiderinterface.h"
#include "alignment/AlignmentSubsystemForDrivers.h"

typedef enum { PARK_COUNTERCLOCKWISE = 0, PARK_CLOCKWISE } ParkDirection_t;
typedef enum { PARK_NORTH = 0, PARK_EAST, PARK_SOUTH, PARK_WEST } ParkPosition_t;

#define SKYWATCHER_SIDEREAL_DAY 86164.09053083288
#define SKYWATCHER_SIDEREAL_SPEED 15.04106864

#define SKYWATCHER_LOWSPEED_RATE 128      // Times Sidereal Speed
#define SKYWATCHER_HIGHSPEED_RATE 800     // Times Siderdeal Speed
#define SKYWATCHER_MINSLEW_RATE  0.0001   // Times Sidereal Speed
#define SKYWATCHER_MAXSLEW_RATE  800      // Times Sidereal Speed
struct GuidingPulse
{
    double DeltaAlt { 0 };
    double DeltaAz { 0 };
    int Duration { 0 };
    int OriginalDuration { 0 };
};


class SkywatcherAZGTIMount: public SkywatcherAPI,
                           public INDI::Telescope,
                           public INDI::GuiderInterface,public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers
{
public:
    SkywatcherAZGTIMount();
    virtual ~SkywatcherAZGTIMount() = default;

    //  overrides of base class virtual functions
    virtual bool Abort() override;
    virtual bool Handshake() override;
    virtual bool Connect() override;

    virtual const char *getDefaultName() override;
    virtual bool Goto(double ra, double dec) override;
    bool EQGoto(double ra, double dec);
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                           char *formats[], char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    virtual bool updateLocation(double latitude, double longitude, double elevation) override;


    bool startTracking();
    bool startGuiding();

    double GetRATrackRate();
    double GetDETrackRate();
    double GetSlewRate();
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    double GetParkDeltaAz(ParkDirection_t target_direction, ParkPosition_t target_position);

    virtual bool Park() override;
    virtual bool UnPark() override;
    virtual bool SetCurrentPark() override;
    virtual bool SetDefaultPark() override;

    virtual bool ReadScopeStatus() override;
    virtual bool saveConfigItems(FILE *fp) override;
    virtual bool Sync(double ra, double dec) override;
    virtual void TimerHit() override;

    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;
    int myFD=-1;

private:

   SkywatcherAPI *mount;

    void CalculateGuidePulses();
    void ConvertGuideCorrection(double delta_ra, double delta_dec, double &delta_alt, double &delta_az);
    void ResetGuidePulses();
    void UpdateScopeConfigSwitch();

    double getJulianDate();
    double getLst(double jd, double lng);
    double getLongitude();
    double getLatitude();

    /* Time variables */
    struct tm utc;
    struct ln_date lndate;
    struct timeval lasttimeupdate;
    struct timespec lastclockupdate;
    double juliandate;

    double currentRA, currentHA;
    double currentDEC;
    double targetRA;
    double targetDEC;
    // Overrides for the pure virtual functions in SkyWatcherAPI

    virtual int skywatcher_tty_read(int fd, char *buf, int nbytes, int timeout, int *nbytes_read) override;
    virtual int skywatcher_tty_write(int fd, const char *buffer, int nbytes, int *nbytes_written) override;

    virtual int skywatcher_azgti_Write(int fd, const char *buffer,int *nbytes_written);
    virtual int skywatcher_azgti_Read(int fd,char *buf,int *nbytes_read);


    void UpdateDetailedMountInformation(bool InformClient);
    ln_hrz_posn GetAltAzPosition(double ra, double dec, double offset_in_sec = 0);
    ln_equ_posn GetRaDecPosition(double alt, double az);
    void LogMessage(const char* format, ...);

    bool detectScope();
    void setUdpFd(int fd);

    // Properties
    int NumPark { 0 };
    static constexpr const char *DetailedMountInfoPage { "Detailed Mount Information" };
    enum
    {
        MOTOR_CONTROL_FIRMWARE_VERSION,
        MOUNT_CODE,
        MOUNT_NAME,
        IS_DC_MOTOR
    };
    enum
    {
        TRACK_SIDEREAL,
        TRACK_KING,
        TRACK_LUNAR,
        TRACK_SOLAR

    };
    IText BasicMountInfo[4] {};
    ITextVectorProperty BasicMountInfoV;

    enum
    {
        MICROSTEPS_PER_REVOLUTION,
        STEPPER_CLOCK_FREQUENCY,
        HIGH_SPEED_RATIO,
        MICROSTEPS_PER_WORM_REVOLUTION
    };
    INumber AxisOneInfo[4];
    INumberVectorProperty AxisOneInfoV;
    INumber AxisTwoInfo[4];
    INumberVectorProperty AxisTwoInfoV;
    enum
    {
        FULL_STOP,
        SLEWING,
        SLEWING_TO,
        SLEWING_FORWARD,
        HIGH_SPEED,
        NOT_INITIALISED
    };
    ISwitch AxisOneState[6];
    ISwitchVectorProperty AxisOneStateV;
    ISwitch AxisTwoState[6];
    ISwitchVectorProperty AxisTwoStateV;
    enum
    {
        RAW_MICROSTEPS,
        MICROSTEPS_PER_ARCSEC,
        OFFSET_FROM_INITIAL,
        DEGREES_FROM_INITIAL
    };
    INumber AxisOneEncoderValues[4];
    INumberVectorProperty AxisOneEncoderValuesV;
    INumber AxisTwoEncoderValues[4];
    INumberVectorProperty AxisTwoEncoderValuesV;

    // A switch for silent/highspeed slewing modes
    enum
    {
        SLEW_SILENT,
        SLEW_NORMAL
    };
    ISwitch SlewModes[2];
    ISwitchVectorProperty SlewModesSP;

    // A switch for wedge mode
    enum
    {
        WEDGE_SIMPLE,
        WEDGE_EQ,
        WEDGE_DISABLED
    };
    ISwitch WedgeMode[3];
    ISwitchVectorProperty WedgeModeSP;

    // A switch for tracking logging
    enum
    {
        TRACKLOG_ENABLED,
        TRACKLOG_DISABLED
    };
    ISwitch TrackLogMode[2];
    ISwitchVectorProperty TrackLogModeSP;

    // Guiding rates (RA/Dec)
    INumber GuidingRatesN[2];
    INumberVectorProperty GuidingRatesNP;

    // Tracking values
    INumber TrackingValuesN[3];
    INumberVectorProperty TrackingValuesNP;

    // A switch for park movement directions (clockwise/counterclockwise)
    ISwitch ParkMovementDirection[2];
    ISwitchVectorProperty ParkMovementDirectionSP;

    // A switch for park positions
    ISwitch ParkPosition[4];
    ISwitchVectorProperty ParkPositionSP;

    // A switch for unpark positions
    ISwitch UnparkPosition[4];
    ISwitchVectorProperty UnparkPositionSP;

    // A switch for track positions
    ISwitch TrackModeS[3];
    ISwitchVectorProperty TrackModeSP;

    /* Hemisphere */
    ISwitch HemisphereS[2];
    ISwitchVectorProperty HemisphereSP;

    enum Hemisphere
    {
        NORTH = 0,
        SOUTH = 1
    };

    typedef struct GotoParams
    {
        double ratarget, detarget, racurrent, decurrent;
        unsigned long ratargetencoder, detargetencoder, racurrentencoder, decurrentencoder;
        unsigned long limiteast, limitwest;
        unsigned int iterative_count;
        bool forcecwup, checklimits, outsidelimits, completed;
    } GotoParams;

    Hemisphere Hemisphere;
    bool RAInverted, DEInverted;
    bool ForceCwUp = false;
    GotoParams gotoparams;

    // Tracking
    ln_equ_posn CurrentTrackingTarget { 0, 0 };
    long OldTrackingTarget[2] { 0, 0 };
    struct ln_hrz_posn CurrentAltAz { 0, 0 };
    bool ResetTrackingSeconds { false };
    int TrackingMsecs { 0 };
    int TrackingStartTimer { 0 };
    double GuideDeltaAlt { 0 };
    double GuideDeltaAz { 0 };
    int TimeoutDuration { 500 };
    const std::string TrackLogFileName;
    int UpdateCount { 0 };

    /// Save the serial port name
    std::string SerialPortName;
    /// Recover after disconnection
    bool RecoverAfterReconnection { false };
    bool VerboseScopeStatus { false };

    struct addrinfo *serverInfo;
    GuidingPulse NorthPulse;
    GuidingPulse WestPulse;
    std::vector<GuidingPulse> GuidingPulses;

    /* for use with libnova */
    struct ln_equ_posn lnradec;
    struct ln_lnlat_posn lnobserver;
    struct ln_hrz_posn lnaltaz;


    void SetSouthernHemisphere(bool southern);

    void HAandDECfromEncoderValues(unsigned long RAEncoder, unsigned long DEEncoder, double &dHA, double &dDec);
        void EncoderValuesfromHAanDEC(double dHa, double dDec, unsigned long &RAEncoder, unsigned long &DEEncoder);
     long abs(long a);
};
