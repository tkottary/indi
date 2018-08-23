#include "inditelescope.h"
#include "indiguiderinterface.h"
#include "skywatcherAPI.h"

typedef enum { PARK_COUNTERCLOCKWISE = 0, PARK_CLOCKWISE } ParkDirection_t;
typedef enum { PARK_NORTH = 0, PARK_EAST, PARK_SOUTH, PARK_WEST } ParkPosition_t;

struct GuidingPulse
{
    float DeltaAlt { 0 };
    float DeltaAz { 0 };
};

class SkywatcherAZGTIMount:public SkywatcherAPI,
        public INDI::Telescope,
        public INDI::GuiderInterface
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
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                           char *formats[], char *names[], int n) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;

    double GetSlewRate();
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    double GetParkDeltaAz(ParkDirection_t target_direction, ParkPosition_t target_position);

    virtual bool Park() override;
    virtual bool UnPark() override;
    virtual bool ReadScopeStatus() override;
    virtual bool saveConfigItems(FILE *fp) override;
    virtual bool Sync(double ra, double dec) override;
    virtual void TimerHit() override;


    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;
private:
    void ResetGuidePulses();
    void UpdateScopeConfigSwitch();
    // Overrides for the pure virtual functions in SkyWatcherAPI

    virtual int skywatcher_tty_read(int fd, char *buf, int nbytes, int timeout, int *nbytes_read) override;
    virtual int skywatcher_tty_write(int fd, const char *buffer, int nbytes, int *nbytes_written) override;

    void UpdateDetailedMountInformation(bool InformClient);
    ln_hrz_posn GetAltAzPosition(double ra, double dec, double offset_in_sec = 0);
    ln_equ_posn GetRaDecPosition(double alt, double az);
    void LogMessage(const char* format, ...);

    // Properties

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

    // A switch for unpark positions
    ISwitch TrackModeS[4];
    ISwitchVectorProperty TrackModeSSP;

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

    std::vector<GuidingPulse> GuidingPulses;
};
