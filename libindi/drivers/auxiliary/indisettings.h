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

#include <string>
#include <libnova.h>

/**
 * \class Settings
 * \brief Class to provide general generic settings for INDI drivers
 *
 * \author Jasem Mutlaq
 */
namespace INDI
{

class Settings : public DefaultDevice
{
  public:
    Settings();
    virtual ~Settings();

    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual void ISGetProperties(const char *dev);
    virtual bool ISSnoopDevice(XMLEle *root);

  protected:
    virtual const char *getDefaultName();

    /** \brief Called to initialize basic properties required all the time */
    virtual bool initProperties();

    virtual bool saveConfigItems(FILE *fp);

    /**
     * \brief Update time, date, and UTC offset.
     * \param utc UTC time.
     * \param utc_offset UTC offset in hours.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool updateTime(ln_date *utc, double utc_offset);

    /**
     * \brief Update location settings
     * \param latitude Site latitude in degrees.
     * \param longitude Site latitude in degrees increasing eastward from Greenwich (0 to 360).
     * \param elevation Site elevation in meters.
     * \return True if successful, false otherwise
     * \note If not implemented by the child class, this function by default returns false with a
     * warning message.
     */
    virtual bool updateLocation(double latitude, double longitude, double elevation);

    /**
     * @brief Load scope settings from XML files.
     * @return True if all config values were loaded otherwise false.
     */
    bool LoadScopeConfig();

    /**
     * @brief Load scope settings from XML files.
     * @return True if Config #1 exists otherwise false.
     */
    bool HasDefaultScopeConfig();

    /**
     * \brief Save scope settings to XML files.
     */
    bool UpdateScopeConfig();

    /**
     * @brief Validate a file name
     * @param file_name File name
     * @return True if the file name is valid otherwise false.
     */
    std::string GetHomeDirectory() const;

    /**
     * @brief Get the scope config index
     * @return The scope config index
     */
    int GetScopeConfigIndex() const;

    /**
     * @brief Check if a file exists and it is readable
     * @param file_name File name
     * @param writable Additional check if the file is writable
     * @return True if the checks are successful otherwise false.
     */
    bool CheckFile(const std::string &file_name, bool writable) const;

    // A number vector that stores lattitude and longitude
    INumberVectorProperty LocationNP;
    INumber LocationN[3];
    enum
    {
        LOCATION_LATITUDE,
        LOCATION_LONGITUDE,
        LOCATION_ELEVATION
    };

    // Settings & guider aperture and focal length
    INumber ScopeParametersN[4];
    INumberVectorProperty ScopeParametersNP;

    // UTC and UTC Offset
    IText TimeT[2] {};
    ITextVectorProperty TimeTP;
    void sendTimeFromSystem();

    // Active GPS/Mount device to snoop
    ITextVectorProperty ActiveDeviceTP;
    IText ActiveDeviceT[2] {};
    enum
    {
        ACTIVE_GPS,
        ACTIVE_MOUNT
    };

  // XML node names for scope config
  const std::string ScopeConfigRootXmlNode { "scopeconfig" };
  const std::string ScopeConfigDeviceXmlNode { "device" };
  const std::string ScopeConfigNameXmlNode { "name" };
  const std::string ScopeConfigScopeFocXmlNode { "scopefoc" };
  const std::string ScopeConfigScopeApXmlNode { "scopeap" };
  const std::string ScopeConfigGScopeFocXmlNode { "gscopefoc" };
  const std::string ScopeConfigGScopeApXmlNode { "gscopeap" };
  const std::string ScopeConfigLabelApXmlNode { "label" };

  // A switch to apply custom aperture/focal length config
    enum
    {
        SCOPE_CONFIG1,
        SCOPE_CONFIG2,
        SCOPE_CONFIG3,
        SCOPE_CONFIG4,
        SCOPE_CONFIG5,
        SCOPE_CONFIG6
    };
    ISwitch ScopeConfigs[6];
    ISwitchVectorProperty ScopeConfigsSP;

    // Scope config name
    ITextVectorProperty ScopeConfigNameTP;
    IText ScopeConfigNameT[1] {};

    /// The telescope/guide scope configuration file name
    const std::string ScopeConfigFileName;

private:
    bool processTimeInfo(const char *utc, const char *offset);
    bool processLocationInfo(double latitude, double longitude, double elevation);

    void triggerSnoop(const char *driverName, const char *propertyName);

};

}
