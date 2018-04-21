/*******************************************************************************
  Copyright(c) 2011 Gerry Rozema, Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#pragma once

#include "defaultdevice.h"

#include <libnova/julian_day.h>

#include <string>

/**
 * \class Mount
 * \brief Class to provide general functionality of a mount device.
 *
 * Developers need to subclass Mount to implement any driver for mounts within INDI.
 *
 * Implementing a basic mount driver involves the child class performing the following steps:
 * <ul>
 * <li>The child class should define the mount capabilities via the MountCapability structure
 * and sets in the default constructor.</li>
 * <li>If the mount has additional properties, the child class should override initProperties and
 * initialize the respective additional properties.</li>
 * <li>The child class can optionally set the connection mode in initProperties(). By default the driver
 * provide controls for both serial and TCP/IP connections.</li>
 * <li>Once the parent class calls Connect(), the child class attempts to connect to the mount and
 * return either success of failure</li>
 * <li>Mount calls updateProperties() to enable the child class to define which properties to
 * send to the client upon connection</li>
 * <li>Mount calls ReadScopeStatus() to check the link to the mount and update its state
 * and position. The child class should call newRaDec() whenever
 * a new value is read from the mount.</li>
 * <li>The child class should implement Goto() and Sync(), and Park()/UnPark() if applicable.</li>
 * <li>Mount calls disconnect() when the client request a disconnection. The child class
 * should remove any additional properties it defined in updateProperties() if applicable</li>
 * </ul>
 *
 * TrackState is used to monitor changes in Tracking state. There are three main tracking properties:
 * + TrackMode: Changes tracking mode or rate. Common modes are TRACK_SIDEREAL, TRACK_LUNAR, TRACK_SOLAR, and TRACK_CUSTOM
 * + TrackRate: If the mount supports custom tracking rates, it should set the capability flag MOUNT_HAS_TRACK_RATE. If the user
 *              changes the custom tracking rates while the mount is tracking, it it sent to the child class via SetTrackRate(...) function.
 *              The base class will reject any track rates that switch from positive to negative (reverse) tracking rates as the mount must be stopped before
 *              such change takes place.
 * + TrackState: Engages or Disengages tracking. When engaging tracking, the child class should take the necessary steps to set the appropiate TrackMode and TrackRate
 *               properties before or after engaging tracking as governed by the mount protocol.
 *
 * Ideally, the child class should avoid changing property states directly within a function call from the base class as such state changes take place in the base class
 * after checking the return values of such functions.
 * \author Jasem Mutlaq, Gerry Rozema
 * \see MountSimulator and SynScan drivers for examples of implementations of Mount.
 */
namespace INDI
{

class Mount : public DefaultDevice
{
  public:
    enum MountStatus
    {
        MOUNT_IDLE,
        MOUNT_SLEWING,
        MOUNT_TRACKING,
        MOUNT_PARKING,
        MOUNT_PARKED
    };
    enum MountMotionCommand
    {
        MOTION_START = 0,
        MOTION_STOP
    };
    enum MountSlewRate
    {
        SLEW_GUIDE,
        SLEW_CENTERING,
        SLEW_FIND,
        SLEW_MAX
    };
    enum MountTrackMode
    {
        TRACK_SIDEREAL,
        TRACK_SOLAR,
        TRACK_LUNAR,
        TRACK_CUSTOM
    };
    enum MountTrackState
    {
        TRACK_ON,
        TRACK_OFF,
        TRACK_UNKNOWN
    };
    enum MountParkData
    {
        PARK_NONE,
        PARK_RA_DEC,
        PARK_HA_DEC,
        PARK_AZ_ALT,
        PARK_RA_DEC_ENCODER,
        PARK_AZ_ALT_ENCODER
    };
    enum MountLocation
    {
        LOCATION_LATITUDE,
        LOCATION_LONGITUDE,
        LOCATION_ELEVATION
    };
    enum MountPierSide
    {
        PIER_UNKNOWN = -1,
        PIER_WEST    = 0,
        PIER_EAST    = 1
    };

    enum MountPECState
    {
        PEC_UNKNOWN = -1,
        PEC_OFF     = 0,
        PEC_ON      = 1
    };

    /**
     * \struct MountConnection
     * \brief Holds the connection mode of the mount.
     */
    enum
    {
        CONNECTION_NONE   = 1 << 0, /** Do not use any connection plugin */
        CONNECTION_SERIAL = 1 << 1, /** For regular serial and bluetooth connections */
        CONNECTION_TCP    = 1 << 2  /** For Wired and WiFI connections */
    } MountConnection;

    /**
     * \struct MountCapability
     * \brief Holds the capabilities of a mount.
     */
    enum
    {
        MOUNT_CAN_GOTO          = 1 << 0, /** Can the mount go to to specific coordinates? */
        MOUNT_CAN_SYNC          = 1 << 1, /** Can the mount sync to specific coordinates? */
        MOUNT_CAN_PARK          = 1 << 2, /** Can the mount park? */
        MOUNT_CAN_ABORT         = 1 << 3, /** Can the mount abort motion? */
        MOUNT_HAS_TIME          = 1 << 4, /** Does the mount have configurable date and time settings? */
        MOUNT_HAS_LOCATION      = 1 << 5, /** Does the mount have configuration location settings? */
        MOUNT_HAS_PIER_SIDE     = 1 << 6, /** Does the mount have pier side property? */
        MOUNT_HAS_PEC           = 1 << 7,  /** Does the mount have PEC playback? */
        MOUNT_HAS_TRACK_MODE    = 1 << 8,  /** Does the mount have track modes (sidereal, lunar, solar..etc)? */
        MOUNT_CAN_CONTROL_TRACK = 1 << 9,  /** Can the mount engage and disengage tracking? */
        MOUNT_HAS_TRACK_RATE    = 1 << 10,  /** Does the mount have custom track rates? */
    } MountCapability;

    Mount();
    virtual ~Mount();

    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual void ISGetProperties(const char *dev);
    virtual bool ISSnoopDevice(XMLEle *root);

    /**
     * @brief GetMountCapability returns the capability of the Mount
     */
    uint32_t GetMountCapability() const { return capability; }

    /**
     * @brief SetMountCapability sets the Mount capabilities. All capabilities must be initialized.
     * @param cap ORed list of mount capabilities.
     * @param slewRateCount Number of slew rates supported by the mount. If < 4 (default is 0),
     * no slew rate properties will be defined to the client. If >=4, the driver will construct the default
     * slew rate property MOUNT_SLEW_RATE with SLEW_GUIDE, SLEW_CENTERING, SLEW_FIND, and SLEW_MAX
     * members where SLEW_GUIDE is the at the lowest setting and SLEW_MAX is at the highest.
     */
    void SetMountCapability(uint32_t cap, uint8_t slewRateCount = 0);

    /**
     * @return True if mount support goto operations
     */
    bool CanGOTO() { return capability & MOUNT_CAN_GOTO; }

    /**
     * @return True if mount support sync operations
     */
    bool CanSync() { return capability & MOUNT_CAN_SYNC; }

    /**
     * @return True if mount can abort motion.
     */
    bool CanAbort() { return capability & MOUNT_CAN_ABORT; }

    /**
     * @return True if mount can park.
     */
    bool CanPark() { return capability & MOUNT_CAN_PARK; }

    /**
     * @return True if mount can enagle and disengage tracking.
     */
    bool CanControlTrack() { return capability & MOUNT_CAN_CONTROL_TRACK; }

    /**
     * @return True if mount time can be updated.
     */
    bool HasTime() { return capability & MOUNT_HAS_TIME; }

    /**
     * @return True if mount location can be updated.
     */
    bool HasLocation() { return capability & MOUNT_HAS_LOCATION; }

    /**
     * @return True if mount supports pier side property
     */
    bool HasPierSide() { return capability & MOUNT_HAS_PIER_SIDE; }

    /**
     * @return True if mount supports PEC playback property
     */
    bool HasPECState() { return capability & MOUNT_HAS_PEC; }

    /**
     * @return True if mount supports track modes
     */
    bool HasTrackMode() { return capability & MOUNT_HAS_TRACK_MODE; }

    /**
     * @return True if mount supports custom tracking rates.
     */
    bool HasTrackRate() { return capability & MOUNT_HAS_TRACK_RATE; }

    /** \brief Called to initialize basic properties required all the time */
    virtual bool initProperties();
    /** \brief Called when connected state changes, to add/remove properties */
    virtual bool updateProperties();

    /** \brief perform handshake with device to check communication */
    virtual bool Handshake();

    /** \brief Called when setTimer() time is up */
    virtual void TimerHit();

    /**
     * \brief setParkDataType Sets the type of parking data stored in the park data file and
     * presented to the user.
     * \param type parking data type. If PARK_NONE then no properties will be presented to the
     * user for custom parking position.
     */
    void SetParkDataType(MountParkData type);

    /**
     * @brief InitPark Loads parking data (stored in ~/.indi/ParkData.xml) that contains parking status
     * and parking position.
     * @return True if loading is successful and data is read, false otherwise. On success, you must call
     * SetAxis1ParkDefault() and SetAxis2ParkDefault() to set the default parking values. On failure,
     * you must call SetAxis1ParkDefault() and SetAxis2ParkDefault() to set the default parking values
     * in addition to SetAxis1Park() and SetAxis2Park() to set the current parking position.
     */
    bool InitPark();

    /**
     * @brief isParked is mount currently parked?
     * @return True if parked, false otherwise.
     */
    bool isParked();

    /**
     * @brief SetParked Change the mount parking status. The data park file (stored in
     * ~/.indi/ParkData.xml) is updated in the process.
     * @param isparked set to true if parked, false otherwise.
     */
    void SetParked(bool isparked);

    /**
     * @return Get current RA/AZ parking position.
     */
    double GetAxis1Park() const;

    /**
     * @return Get default RA/AZ parking position.
     */
    double GetAxis1ParkDefault() const;

    /**
     * @return Get current DEC/ALT parking position.
     */
    double GetAxis2Park() const;

    /**
     * @return Get defailt DEC/ALT parking position.
     */
    double GetAxis2ParkDefault() const;

    /**
     * @brief SetRAPark Set current RA/AZ parking position. The data park file (stored in
     * ~/.indi/ParkData.xml) is updated in the process.
     * @param value current Axis 1 value (RA or AZ either in angles or encoder values as specified
     * by the MountParkData type).
     */
    void SetAxis1Park(double value);

    /**
     * @brief SetRAPark Set default RA/AZ parking position.
     * @param value Default Axis 1 value (RA or AZ either in angles or encoder values as specified
     * by the MountParkData type).
     */
    void SetAxis1ParkDefault(double steps);

    /**
     * @brief SetDEPark Set current DEC/ALT parking position. The data park file (stored in
     * ~/.indi/ParkData.xml) is updated in the process.
     * @param value current Axis 1 value (DEC or ALT either in angles or encoder values as specified
     * by the MountParkData type).
     */
    void SetAxis2Park(double steps);

    /**
     * @brief SetDEParkDefault Set default DEC/ALT parking position.
     * @param value Default Axis 2 value (DEC or ALT either in angles or encoder values as specified
     * by the MountParkData type).
     */
    void SetAxis2ParkDefault(double steps);

    /**
     * @brief isLocked is mount currently locked?
     * @return true if lock status equals true and DomeClosedLockTP is Dome Locks or Dome Locks and
     * Dome Parks (both).
     */
    bool isLocked() const;

    // Joystick helpers
    static void joystickHelper(const char *joystick_n, double mag, double angle, void *context);
    static void buttonHelper(const char *button_n, ISState state, void *context);

    /**
     * @brief setMountConnection Set mount connection mode. Child class should call this
     * in the constructor before Mount registers any connection interfaces
     * @param value ORed combination of MountConnection values.
     */
    void setMountConnection(const uint8_t &value);

    /**
     * @return Get current mount connection mode
     */
    uint8_t getMountConnection() const;

    void setPierSide(MountPierSide side);
    MountPierSide getPierSide() { return currentPierSide; }

    void setPECState(MountPECState state);
    MountPECState getPECState() { return currentPECState; }

  protected:
    virtual bool saveConfigItems(FILE *fp);

    /** \brief The child class calls this function when it has updates */
    void NewRaDec(double ra, double dec);

    /**
     * \brief Read mount status.
     *
     * This function checks the following:
     * <ol>
     *   <li>Check if the link to the mount is alive.</li>
     *   <li>Update mount status: Idle, Slewing, Parking..etc.</li>
     *   <li>Read coordinates</li>
     * </ol>
     * \return True if reading scope status is OK, false if an error is encounterd.
     * \note This function is not implemented in Mount, it must be implemented in the
     * child class
     */
    virtual bool ReadScopeStatus() = 0;

    /**
     * \brief Move the scope to the supplied RA and DEC coordinates
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool Goto(double ra, double dec);

    /**
     * \brief Set the mount current RA and DEC coordinates to the supplied RA and DEC coordinates
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool Sync(double ra, double dec);

    /**
     * \brief Start or Stop the mount motion in the direction dir.
     * \param dir direction of motion
     * \param command Start or Stop command
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool MoveNS(INDI_DIR_NS dir, MountMotionCommand command);

    /**
     * \brief Move the mount in the direction dir.
     * \param dir direction of motion
     * \param command Start or Stop command
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool MoveWE(INDI_DIR_WE dir, MountMotionCommand command);

    /**
     * \brief Park the mount to its home position.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool Park();

    /**
     * \brief Unpark the mount if already parked.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool UnPark();

    /**
     * \brief Abort any mount motion including tracking if possible.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool Abort();

    /**
     * @brief SetTrackMode Set active tracking mode. Do not change track state.
     * @param mode Index of track mode.
     * @return True if successful, false otherwise
     * @note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool SetTrackMode(uint8_t mode);

    /**
     * @brief SetTrackRate Set custom tracking rates.
     * @param raRate RA tracking rate in arcsecs/s
     * @param deRate DEC tracking rate in arcsecs/s
     * @return True if successful, false otherwise
     * @note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool SetTrackRate(double raRate, double deRate);

    /**
     * @brief AddTrackMode
     * @param name Name of track mode. It is recommended to use standard properties names such as TRACK_SIDEREAL..etc.
     * @param label Label of track mode that appears at the client side.
     * @param isDefault Set to true to mark the track mode as the default. Only one mode should be marked as default.
     * @return Index of added track mode
     * @note Child class should add all track modes be
     */
    virtual int AddTrackMode(const char *name, const char *label, bool isDefault=false);

    /**
     * @brief SetTrackEnabled Engages or disengages mount tracking. If there are no tracking modes available, it is assumed sidereal. Otherwise,
     * whatever tracking mode should be activated or deactivated accordingly.
     * @param enabled True to engage tracking, false to stop tracking completely.
     * @return True if successful, false otherwise
     * @note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool SetTrackEnabled(bool enabled);

    /**
     * \brief Update mount time, date, and UTC offset.
     * \param utc UTC time.
     * \param utc_offset UTC offset in hours.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool updateTime(ln_date *utc, double utc_offset);

    /**
     * \brief Update mount location settings
     * \param latitude Site latitude in degrees.
     * \param longitude Site latitude in degrees increasing eastward from Greenwich (0 to 360).
     * \param elevation Site elevation in meters.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool updateLocation(double latitude, double longitude, double elevation);

    /**
     * \brief SetParkPosition Set desired parking position to the supplied value. This ONLY sets the
     * desired park position value and does not perform parking.
     * \param Axis1Value First axis value
     * \param Axis2Value Second axis value
     * \return True if desired parking position is accepted and set. False otherwise.
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool SetParkPosition(double Axis1Value, double Axis2Value);

    /**
     * @brief SetCurrentPark Set current coordinates/encoders value as the desired parking position
     * @return True if current mount coordinates are set as parking position, false on error.
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool SetCurrentPark();

    /**
     * @brief SetDefaultPark Set default coordinates/encoders value as the desired parking position
     * @return True if default park coordinates are set as parking position, false on error.
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool SetDefaultPark();

    /**
     * @brief SetSlewRate Set desired slew rate index.
     * @param index Index of slew rate where 0 is slowest rate and capability.nSlewRate-1 is maximum rate.
     * @return True is operation successful, false otherwise.
     *
     * \note This function as implemented in Mount performs no function and always return
     * true. Only reimplement it if you need to issue a command to change the slew rate at the hardware
     * level. Most mount drivers only utilize slew rate when issuing a motion command.
     */
    virtual bool SetSlewRate(int index);

    /**
     * @brief callHandshake Helper function that sets the port file descriptor before calling the
     * actual Handshake function implenented in drivers
     * @return Result of actual device Handshake()
     */
    bool callHandshake();

    // Joystick
    void processNSWE(double mag, double angle);
    void processJoystick(const char *joystick_n, double mag, double angle);
    void processSlewPresets(double mag, double angle);
    void processButton(const char *button_n, ISState state);

    /**
     * @brief Validate a file name
     * @param file_name File name
     * @return True if the file name is valid otherwise false.
     */
    std::string GetHomeDirectory() const;

    /**
     * This is a variable filled in by the ReadStatus mount
     * low level code, used to report current state
     * are we slewing, tracking, or parked.
     */
    MountStatus TrackState;

    /**
     * @brief RememberTrackState Remember last state of Track State to fall back to in case of errors or aborts.
     */
    MountStatus RememberTrackState;

    // All mounts should produce equatorial co-ordinates
    INumberVectorProperty EqNP;
    INumber EqN[2];

    // When a goto is issued, domes will snoop the target property
    // to start moving the dome when a mount moves
    INumberVectorProperty TargetNP;
    INumber TargetN[2];

    // Abort motion
    ISwitchVectorProperty AbortSP;
    ISwitch AbortS[1];

    // On a coord_set message, sync, or slew
    ISwitchVectorProperty CoordSP;
    ISwitch CoordS[3];

    // A number vector that stores lattitude and longitude
    INumberVectorProperty LocationNP;
    INumber LocationN[3];

    // A Switch in the client interface to park the scope
    ISwitchVectorProperty ParkSP;
    ISwitch ParkS[2];

    // Custom parking position
    INumber ParkPositionN[2];
    INumberVectorProperty ParkPositionNP;

    // Custom parking options
    ISwitch ParkOptionS[3];
    ISwitchVectorProperty ParkOptionSP;

    // A switch for North/South motion
    ISwitch MovementNSS[2];
    ISwitchVectorProperty MovementNSSP;

    // A switch for West/East motion
    ISwitch MovementWES[2];
    ISwitchVectorProperty MovementWESP;

    // Slew Rate
    ISwitchVectorProperty SlewRateSP;
    ISwitch *SlewRateS;

    // UTC and UTC Offset
    IText TimeT[2] {};
    ITextVectorProperty TimeTP;
    void sendTimeFromSystem();

    // Active GPS/Dome device to snoop
    ITextVectorProperty ActiveDeviceTP;
    IText ActiveDeviceT[2] {};

    // Switch to lock if dome is closed, and or force parking if dome parks
    ISwitchVectorProperty DomeClosedLockTP;
    ISwitch DomeClosedLockT[4];

    // Lock Joystick Axis to one direciton only
    ISwitch LockAxisS[2];
    ISwitchVectorProperty LockAxisSP;

    // Pier Side
    ISwitch PierSideS[2];
    ISwitchVectorProperty PierSideSP;

    // Pier Side
    MountPierSide lastPierSide, currentPierSide;

    // PEC State
    ISwitch PECStateS[2];
    ISwitchVectorProperty PECStateSP;

    // Track Mode
    ISwitchVectorProperty TrackModeSP;
    ISwitch *TrackModeS { nullptr };

    // Track State
    ISwitchVectorProperty TrackStateSP;
    ISwitch TrackStateS[2];

    // Track Rate
    INumberVectorProperty TrackRateNP;
    INumber TrackRateN[2];

    // PEC State
    MountPECState lastPECState, currentPECState;


    uint32_t capability;
    int last_we_motion, last_ns_motion;

    //Park
    char *LoadParkData();
    bool WriteParkData();

    int PortFD                           = -1;
    Connection::Serial *serialConnection = NULL;
    Connection::TCP *tcpConnection       = NULL;

private:
    bool processTimeInfo(const char *utc, const char *offset);
    bool processLocationInfo(double latitude, double longitude, double elevation);

    void triggerSnoop(const char *driverName, const char *propertyName);

    MountParkData parkDataType;
    bool IsLocked;
    bool IsParked;
    const char *ParkDeviceName;
    const std::string ParkDataFileName;
    XMLEle *ParkdataXmlRoot, *ParkdeviceXml, *ParkstatusXml, *ParkpositionXml, *ParkpositionAxis1Xml,
           *ParkpositionAxis2Xml;

    double Axis1ParkPosition;
    double Axis1DefaultParkPosition;
    double Axis2ParkPosition;
    double Axis2DefaultParkPosition;

    uint8_t nSlewRate;

    IPState lastEqState;

    uint8_t mountConnection = CONNECTION_SERIAL | CONNECTION_TCP;
    Controller *controller;
};

}
