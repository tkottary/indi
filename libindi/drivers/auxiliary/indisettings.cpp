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

#include "indisettings.h"
#include "indicom.h"

#include <cmath>
#include <cerrno>
#include <pwd.h>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <wordexp.h>
#include <limits>
#include <memory>

#define TELESCOPE_TAB "Telescope"

// We declare an auto pointer to indisettings.
std::unique_ptr<INDI::Settings> indisettings(new INDI::Settings());

void ISGetProperties(const char *dev)
{
    indisettings->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    indisettings->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    indisettings->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    indisettings->ISNewNumber(dev, name, values, names, n);
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
    indisettings->ISSnoopDevice(root);
}

namespace INDI
{

Settings::Settings()
    : DefaultDevice(), ScopeConfigFileName(GetHomeDirectory() + "/.indi/ScopeConfig.xml")
{    
}

Settings::~Settings()
{
}

const char *Settings::getDefaultName()
{
    return "INDI Settings";
}

bool Settings::initProperties()
{
    DefaultDevice::initProperties();

    // Active Devices
    IUFillText(&ActiveDeviceT[ACTIVE_GPS], "ACTIVE_GPS", "GPS", "GPS Simulator");
    IUFillText(&ActiveDeviceT[ACTIVE_MOUNT], "ACTIVE_MOUNT", "MOUNT", "Mount Simulator");
    IUFillTextVector(&ActiveDeviceTP, ActiveDeviceT, 2, getDeviceName(), "ACTIVE_DEVICES", "Snoop devices", OPTIONS_TAB,
                     IP_RW, 60, IPS_IDLE);

    IUFillText(&TimeT[0], "UTC", "UTC Time", nullptr);
    IUFillText(&TimeT[1], "OFFSET", "UTC Offset", nullptr);
    IUFillTextVector(&TimeTP, TimeT, 2, getDeviceName(), "TIME_UTC", "UTC", SITE_TAB, IP_RW, 60, IPS_IDLE);

    IUFillNumber(&LocationN[LOCATION_LATITUDE], "LAT", "Lat (dd:mm:ss)", "%010.6m", -90, 90, 0, 0.0);
    IUFillNumber(&LocationN[LOCATION_LONGITUDE], "LONG", "Lon (dd:mm:ss)", "%010.6m", 0, 360, 0, 0.0);
    IUFillNumber(&LocationN[LOCATION_ELEVATION], "ELEV", "Elevation (m)", "%g", -200, 10000, 0, 0);
    IUFillNumberVector(&LocationNP, LocationN, 3, getDeviceName(), "GEOGRAPHIC_COORD", "Scope Location", SITE_TAB,
                       IP_RW, 60, IPS_IDLE);

    IUFillNumber(&ScopeParametersN[0], "SCOPE_APERTURE", "Aperture (mm)", "%g", 10, 5000, 0, 0.0);
    IUFillNumber(&ScopeParametersN[1], "SCOPE_FOCAL_LENGTH", "Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
    IUFillNumber(&ScopeParametersN[2], "GUIDER_APERTURE", "Guider Aperture (mm)", "%g", 10, 5000, 0, 0.0);
    IUFillNumber(&ScopeParametersN[3], "GUIDER_FOCAL_LENGTH", "Guider Focal Length (mm)", "%g", 10, 10000, 0, 0.0);
    IUFillNumberVector(&ScopeParametersNP, ScopeParametersN, 4, getDeviceName(), "SCOPE_INFO", "Scope Properties",
                       TELESCOPE_TAB, IP_RW, 60, IPS_OK);

    // Scope config name
    IUFillText(&ScopeConfigNameT[0], "SCOPE_CONFIG_NAME", "Config Name", "");
    IUFillTextVector(&ScopeConfigNameTP, ScopeConfigNameT, 1, getDeviceName(), "SCOPE_CONFIG_NAME", "Scope Name",
                     TELESCOPE_TAB, IP_RW, 60, IPS_OK);

    // Switch for aperture/focal length configs
    IUFillSwitch(&ScopeConfigs[SCOPE_CONFIG1], "SCOPE_CONFIG1", "Config #1", ISS_ON);
    IUFillSwitch(&ScopeConfigs[SCOPE_CONFIG2], "SCOPE_CONFIG2", "Config #2", ISS_OFF);
    IUFillSwitch(&ScopeConfigs[SCOPE_CONFIG3], "SCOPE_CONFIG3", "Config #3", ISS_OFF);
    IUFillSwitch(&ScopeConfigs[SCOPE_CONFIG4], "SCOPE_CONFIG4", "Config #4", ISS_OFF);
    IUFillSwitch(&ScopeConfigs[SCOPE_CONFIG5], "SCOPE_CONFIG5", "Config #5", ISS_OFF);
    IUFillSwitch(&ScopeConfigs[SCOPE_CONFIG6], "SCOPE_CONFIG6", "Config #6", ISS_OFF);
    IUFillSwitchVector(&ScopeConfigsSP, ScopeConfigs, 6, getDeviceName(), "APPLY_SCOPE_CONFIG", "Scope Configs",
                       TELESCOPE_TAB, IP_RW, ISR_1OFMANY, 60, IPS_OK);

    setDriverInterface(AUX_INTERFACE);

    IDSnoopDevice(ActiveDeviceT[ACTIVE_GPS].text, "PRIVATE_GEOGRAPHIC_COORD");
    IDSnoopDevice(ActiveDeviceT[ACTIVE_GPS].text, "PRIVATE_TIME_UTC");

    IDSnoopDevice(ActiveDeviceT[ACTIVE_MOUNT].text, "PRIVATE_GEOGRAPHIC_COORD");
    IDSnoopDevice(ActiveDeviceT[ACTIVE_MOUNT].text, "PRIVATE_TIME_UTC");

    return true;
}

void Settings::ISGetProperties(const char *dev)
{
    //  First we let our parent populate
    DefaultDevice::ISGetProperties(dev);

    defineNumber(&LocationNP);
    defineText(&TimeTP);

    defineText(&ActiveDeviceTP);
    loadConfig(true, "ACTIVE_DEVICES");

    defineNumber(&ScopeParametersNP);
    defineText(&ScopeConfigNameTP);
    if (HasDefaultScopeConfig())
    {
        LoadScopeConfig();
    }
    else
    {
        loadConfig(true, "SCOPE_INFO");
        loadConfig(true, "SCOPE_CONFIG_NAME");
    }
}

bool Settings::ISSnoopDevice(XMLEle *root)
{    

    XMLEle *ep           = nullptr;
    const char *propName = findXMLAttValu(root, "name");
    const char *deviceName = findXMLAttValu(root, "device");

    // If GPS source is already registered, do not accept updates from other drivers
    if (useGPSSource && strstr(deviceName, "GPS") == nullptr)
        return false;

    useGPSSource = (strstr(deviceName, "GPS") != nullptr);

    if (isConnected())
    {
        if (!strcmp(propName, "PRIVATE_GEOGRAPHIC_COORD"))
        {
            // Only accept IPS_OK state
            if (strcmp(findXMLAttValu(root, "state"), "Ok"))
                return false;

            double longitude = -1, latitude = -1, elevation = -1;

            for (ep = nextXMLEle(root, 1); ep != nullptr; ep = nextXMLEle(root, 0))
            {
                const char *elemName = findXMLAttValu(ep, "name");

                if (!strcmp(elemName, "LAT"))
                    latitude = atof(pcdataXMLEle(ep));
                else if (!strcmp(elemName, "LONG"))
                    longitude = atof(pcdataXMLEle(ep));
                else if (!strcmp(elemName, "ELEV"))
                    elevation = atof(pcdataXMLEle(ep));
            }

            return processLocationInfo(latitude, longitude, elevation);
        }
        else if (!strcmp(propName, "PRIVATE_TIME_UTC"))
        {
            // Only accept IPS_OK state
            if (strcmp(findXMLAttValu(root, "state"), "Ok"))
                return false;

            char utc[MAXINDITSTAMP], offset[MAXINDITSTAMP];

            for (ep = nextXMLEle(root, 1); ep != nullptr; ep = nextXMLEle(root, 0))
            {
                const char *elemName = findXMLAttValu(ep, "name");

                if (!strcmp(elemName, "UTC"))
                    strncpy(utc, pcdataXMLEle(ep), MAXINDITSTAMP);
                else if (!strcmp(elemName, "OFFSET"))
                    strncpy(offset, pcdataXMLEle(ep), MAXINDITSTAMP);
            }

            return processTimeInfo(utc, offset);
        }
    }

    return DefaultDevice::ISSnoopDevice(root);
}

void Settings::triggerSnoop(const char *driverName, const char *snoopedProp)
{
    DEBUGF(Logger::DBG_DEBUG, "Active Snoop, driver: %s, property: %s", driverName, snoopedProp);
    IDSnoopDevice(driverName, snoopedProp);
}

bool Settings::saveConfigItems(FILE *fp)
{
    DefaultDevice::saveConfigItems(fp);

    IUSaveConfigText(fp, &ActiveDeviceTP);
    IUSaveConfigNumber(fp, &LocationNP);

    if (!HasDefaultScopeConfig())
    {
        if (ScopeParametersNP.s == IPS_OK)
            IUSaveConfigNumber(fp, &ScopeParametersNP);
        if (ScopeConfigNameTP.s == IPS_OK)
            IUSaveConfigText(fp, &ScopeConfigNameTP);
    }
    return true;
}


/**************************************************************************************
** Process Text properties
***************************************************************************************/
bool Settings::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (!strcmp(name, TimeTP.name))
        {
            int utcindex    = IUFindIndex("UTC", names, n);
            int offsetindex = IUFindIndex("OFFSET", names, n);

            return processTimeInfo(texts[utcindex], texts[offsetindex]);
        }

        if (!strcmp(name, ActiveDeviceTP.name))
        {
            ActiveDeviceTP.s = IPS_OK;
            IUUpdateText(&ActiveDeviceTP, texts, names, n);
            //  Update client display
            IDSetText(&ActiveDeviceTP, nullptr);

            IDSnoopDevice(ActiveDeviceT[0].text, "GEOGRAPHIC_COORD");
            IDSnoopDevice(ActiveDeviceT[0].text, "TIME_UTC");

            IDSnoopDevice(ActiveDeviceT[1].text, "DOME_PARK");
            IDSnoopDevice(ActiveDeviceT[1].text, "DOME_SHUTTER");
            return true;
        }

        if (name && std::string(name) == "SCOPE_CONFIG_NAME")
        {
            ScopeConfigNameTP.s = IPS_OK;
            IUUpdateText(&ScopeConfigNameTP, texts, names, n);
            IDSetText(&ScopeConfigNameTP, nullptr);
            UpdateScopeConfig();
            return true;
        }
    }

    return DefaultDevice::ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
**
***************************************************************************************/
bool Settings::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        ///////////////////////////////////
        // Geographic Coords
        ///////////////////////////////////
        if (strcmp(name, "GEOGRAPHIC_COORD") == 0)
        {
            int latindex       = IUFindIndex("LAT", names, n);
            int longindex      = IUFindIndex("LONG", names, n);
            int elevationindex = IUFindIndex("ELEV", names, n);

            if (latindex == -1 || longindex == -1 || elevationindex == -1)
            {
                LocationNP.s = IPS_ALERT;
                IDSetNumber(&LocationNP, "Location data missing or corrupted.");
            }

            double targetLat  = values[latindex];
            double targetLong = values[longindex];
            double targetElev = values[elevationindex];

            return processLocationInfo(targetLat, targetLong, targetElev);
        }

        ///////////////////////////////////
        // Settings Info
        ///////////////////////////////////
        if (strcmp(name, "SCOPE_INFO") == 0)
        {
            ScopeParametersNP.s = IPS_OK;

            IUUpdateNumber(&ScopeParametersNP, values, names, n);
            IDSetNumber(&ScopeParametersNP, nullptr);
            UpdateScopeConfig();
            return true;
        }
    }

    return DefaultDevice::ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
**
***************************************************************************************/
bool Settings::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        ///////////////////////////////////
        // Scope Apply Config
        ///////////////////////////////////
        if (name && std::string(name) == "APPLY_SCOPE_CONFIG")
        {
            IUUpdateSwitch(&ScopeConfigsSP, states, names, n);
            bool rc          = LoadScopeConfig();
            ScopeConfigsSP.s = (rc ? IPS_OK : IPS_ALERT);
            IDSetSwitch(&ScopeConfigsSP, nullptr);
            return true;
        }
    }

    //  Nobody has claimed this, so, ignore it
    return DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}


//void Settings::TimerHit()
//{
//    if (isConnected())
//    {
//        SetTimer(POLLMS);
//    }
//}

bool Settings::processTimeInfo(const char *utc, const char *offset)
{
    struct ln_date utc_date;
    double utc_offset = 0;

    if (extractISOTime(utc, &utc_date) == -1)
    {
        TimeTP.s = IPS_ALERT;
        IDSetText(&TimeTP, "Date/Time is invalid: %s.", utc);
        return false;
    }

    utc_offset = atof(offset);

    IUSaveText(&TimeT[0], utc);
    IUSaveText(&TimeT[1], offset);
    TimeTP.s = IPS_OK;
    IDSetText(&TimeTP, nullptr);

    // 2018-04-20 JM: Update system time on ARM architecture.
#ifdef __arm__
#ifdef __linux__
    struct tm utm;
    if (strptime(utc, "%Y-%m-%dT%H:%M:%S", &utm))
    {
        time_t raw_time = mktime(&utm);
        time_t now_time;
        time(&now_time);
        // Only sync if difference > 30 seconds
        if (labs(now_time - raw_time) > 30)
            stime(&raw_time);
    }
#endif
#endif

    return true;
}

bool Settings::processLocationInfo(double latitude, double longitude, double elevation)
{
    // Do not update if not necessary
    if (latitude == LocationN[LOCATION_LATITUDE].value && longitude == LocationN[LOCATION_LONGITUDE].value &&
            elevation == LocationN[LOCATION_ELEVATION].value)
    {
        LocationNP.s = IPS_OK;
        IDSetNumber(&LocationNP, nullptr);
    }

    LocationNP.s                        = IPS_OK;
    LocationN[LOCATION_LATITUDE].value  = latitude;
    LocationN[LOCATION_LONGITUDE].value = longitude;
    LocationN[LOCATION_ELEVATION].value = elevation;
    //  Update client display
    IDSetNumber(&LocationNP, nullptr);

    // Always save geographic coord config immediately.
    saveConfig(true, "GEOGRAPHIC_COORD");

    return true;

}

bool Settings::LoadScopeConfig()
{
    if (!CheckFile(ScopeConfigFileName, false))
    {
        DEBUGF(Logger::DBG_SESSION, "Can't open XML file (%s) for read", ScopeConfigFileName.c_str());
        return false;
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
        DEBUGF(Logger::DBG_SESSION, "Failed to parse XML file (%s): %s", ScopeConfigFileName.c_str(), ErrMsg);
        return false;
    }
    if (std::string(tagXMLEle(RootXmlNode)) != ScopeConfigRootXmlNode)
    {
        DEBUGF(Logger::DBG_SESSION, "Not a scope config XML file (%s)", ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
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
        DEBUGF(Logger::DBG_SESSION, "No a scope config found for %s in the XML file (%s)", getDeviceName(),
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    // Read the values
    XMLEle *XmlNode       = nullptr;
    const int ConfigIndex = GetScopeConfigIndex();
    double ScopeFoc = 0, ScopeAp = 0;
    double GScopeFoc = 0, GScopeAp = 0;
    std::string ConfigName;

    CurrentXmlNode = findXMLEle(CurrentXmlNode, ("config" + std::to_string(ConfigIndex)).c_str());
    if (!CurrentXmlNode)
    {
        DEBUGF(Logger::DBG_SESSION,
               "Config %d is not found in the XML file (%s). To save a new config, update and set scope properties and "
               "config name.",
               ConfigIndex, ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigScopeFocXmlNode.c_str());
    if (!XmlNode || sscanf(pcdataXMLEle(XmlNode), "%lf", &ScopeFoc) != 1)
    {
        DEBUGF(Logger::DBG_SESSION, "Can't read the telescope focal length from the XML file (%s)",
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigScopeApXmlNode.c_str());
    if (!XmlNode || sscanf(pcdataXMLEle(XmlNode), "%lf", &ScopeAp) != 1)
    {
        DEBUGF(Logger::DBG_SESSION, "Can't read the telescope aperture from the XML file (%s)",
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigGScopeFocXmlNode.c_str());
    if (!XmlNode || sscanf(pcdataXMLEle(XmlNode), "%lf", &GScopeFoc) != 1)
    {
        DEBUGF(Logger::DBG_SESSION, "Can't read the guide scope focal length from the XML file (%s)",
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigGScopeApXmlNode.c_str());
    if (!XmlNode || sscanf(pcdataXMLEle(XmlNode), "%lf", &GScopeAp) != 1)
    {
        DEBUGF(Logger::DBG_SESSION, "Can't read the guide scope aperture from the XML file (%s)",
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigLabelApXmlNode.c_str());
    if (!XmlNode)
    {
        DEBUGF(Logger::DBG_SESSION, "Can't read the telescope config name from the XML file (%s)",
               ScopeConfigFileName.c_str());
        delXMLEle(RootXmlNode);
        return false;
    }
    ConfigName = pcdataXMLEle(XmlNode);
    // Store the loaded values
    if (IUFindNumber(&ScopeParametersNP, "SCOPE_FOCAL_LENGTH"))
    {
        IUFindNumber(&ScopeParametersNP, "SCOPE_FOCAL_LENGTH")->value = ScopeFoc;
    }
    if (IUFindNumber(&ScopeParametersNP, "SCOPE_APERTURE"))
    {
        IUFindNumber(&ScopeParametersNP, "SCOPE_APERTURE")->value = ScopeAp;
    }
    if (IUFindNumber(&ScopeParametersNP, "GUIDER_FOCAL_LENGTH"))
    {
        IUFindNumber(&ScopeParametersNP, "GUIDER_FOCAL_LENGTH")->value = GScopeFoc;
    }
    if (IUFindNumber(&ScopeParametersNP, "GUIDER_APERTURE"))
    {
        IUFindNumber(&ScopeParametersNP, "GUIDER_APERTURE")->value = GScopeAp;
    }
    if (IUFindText(&ScopeConfigNameTP, "SCOPE_CONFIG_NAME"))
    {
        IUSaveText(IUFindText(&ScopeConfigNameTP, "SCOPE_CONFIG_NAME"), ConfigName.c_str());
    }
    ScopeParametersNP.s = IPS_OK;
    IDSetNumber(&ScopeParametersNP, nullptr);
    ScopeConfigNameTP.s = IPS_OK;
    IDSetText(&ScopeConfigNameTP, nullptr);
    delXMLEle(RootXmlNode);
    return true;
}

bool Settings::HasDefaultScopeConfig()
{
    if (!CheckFile(ScopeConfigFileName, false))
    {
        return false;
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
        return false;
    }
    if (std::string(tagXMLEle(RootXmlNode)) != ScopeConfigRootXmlNode)
    {
        delXMLEle(RootXmlNode);
        return false;
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
        delXMLEle(RootXmlNode);
        return false;
    }
    // Check the existence of Config #1 node
    CurrentXmlNode = findXMLEle(CurrentXmlNode, "config1");
    if (!CurrentXmlNode)
    {
        delXMLEle(RootXmlNode);
        return false;
    }
    return true;
}

bool Settings::UpdateScopeConfig()
{
    // Get the config values from the UI
    const int ConfigIndex = GetScopeConfigIndex();
    double ScopeFoc = 0, ScopeAp = 0;
    double GScopeFoc = 0, GScopeAp = 0;
    std::string ConfigName;

    if (IUFindNumber(&ScopeParametersNP, "SCOPE_FOCAL_LENGTH"))
    {
        ScopeFoc = IUFindNumber(&ScopeParametersNP, "SCOPE_FOCAL_LENGTH")->value;
    }
    if (IUFindNumber(&ScopeParametersNP, "SCOPE_APERTURE"))
    {
        ScopeAp = IUFindNumber(&ScopeParametersNP, "SCOPE_APERTURE")->value;
    }
    if (IUFindNumber(&ScopeParametersNP, "GUIDER_FOCAL_LENGTH"))
    {
        GScopeFoc = IUFindNumber(&ScopeParametersNP, "GUIDER_FOCAL_LENGTH")->value;
    }
    if (IUFindNumber(&ScopeParametersNP, "GUIDER_APERTURE"))
    {
        GScopeAp = IUFindNumber(&ScopeParametersNP, "GUIDER_APERTURE")->value;
    }
    if (IUFindText(&ScopeConfigNameTP, "SCOPE_CONFIG_NAME") &&
            IUFindText(&ScopeConfigNameTP, "SCOPE_CONFIG_NAME")->text)
    {
        ConfigName = IUFindText(&ScopeConfigNameTP, "SCOPE_CONFIG_NAME")->text;
    }
    // Save the values to the actual XML file
    if (!CheckFile(ScopeConfigFileName, true))
    {
        DEBUGF(Logger::DBG_SESSION, "Can't open XML file (%s) for write", ScopeConfigFileName.c_str());
        return false;
    }
    // Open the existing XML file for write
    LilXML *XmlHandle   = newLilXML();
    FILE *FilePtr       = fopen(ScopeConfigFileName.c_str(), "r");
    XMLEle *RootXmlNode = nullptr;
    XMLAtt *Ap          = nullptr;
    bool DeviceFound    = false;
    char ErrMsg[512];

    RootXmlNode = readXMLFile(FilePtr, XmlHandle, ErrMsg);
    delLilXML(XmlHandle);
    XmlHandle = nullptr;
    fclose(FilePtr);

    XMLEle *CurrentXmlNode = nullptr;
    XMLEle *XmlNode        = nullptr;

    if (!RootXmlNode || std::string(tagXMLEle(RootXmlNode)) != ScopeConfigRootXmlNode)
    {
        RootXmlNode = addXMLEle(nullptr, ScopeConfigRootXmlNode.c_str());
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
        CurrentXmlNode = addXMLEle(RootXmlNode, ScopeConfigDeviceXmlNode.c_str());
        addXMLAtt(CurrentXmlNode, ScopeConfigNameXmlNode.c_str(), getDeviceName());
    }
    // Add or update the config node
    XmlNode = findXMLEle(CurrentXmlNode, ("config" + std::to_string(ConfigIndex)).c_str());
    if (!XmlNode)
    {
        CurrentXmlNode = addXMLEle(CurrentXmlNode, ("config" + std::to_string(ConfigIndex)).c_str());
    }
    else
    {
        CurrentXmlNode = XmlNode;
    }
    // Add or update the telescope focal length
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigScopeFocXmlNode.c_str());
    if (!XmlNode)
    {
        XmlNode = addXMLEle(CurrentXmlNode, ScopeConfigScopeFocXmlNode.c_str());
    }
    editXMLEle(XmlNode, std::to_string(ScopeFoc).c_str());
    // Add or update the telescope focal aperture
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigScopeApXmlNode.c_str());
    if (!XmlNode)
    {
        XmlNode = addXMLEle(CurrentXmlNode, ScopeConfigScopeApXmlNode.c_str());
    }
    editXMLEle(XmlNode, std::to_string(ScopeAp).c_str());
    // Add or update the guide scope focal length
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigGScopeFocXmlNode.c_str());
    if (!XmlNode)
    {
        XmlNode = addXMLEle(CurrentXmlNode, ScopeConfigGScopeFocXmlNode.c_str());
    }
    editXMLEle(XmlNode, std::to_string(GScopeFoc).c_str());
    // Add or update the guide scope focal aperture
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigGScopeApXmlNode.c_str());
    if (!XmlNode)
    {
        XmlNode = addXMLEle(CurrentXmlNode, ScopeConfigGScopeApXmlNode.c_str());
    }
    editXMLEle(XmlNode, std::to_string(GScopeAp).c_str());
    // Add or update the config name
    XmlNode = findXMLEle(CurrentXmlNode, ScopeConfigLabelApXmlNode.c_str());
    if (!XmlNode)
    {
        XmlNode = addXMLEle(CurrentXmlNode, ScopeConfigLabelApXmlNode.c_str());
    }
    editXMLEle(XmlNode, ConfigName.c_str());
    // Save the final content
    FilePtr = fopen(ScopeConfigFileName.c_str(), "w");
    prXMLEle(FilePtr, RootXmlNode, 0);
    fclose(FilePtr);
    delXMLEle(RootXmlNode);
    return true;
}

std::string Settings::GetHomeDirectory() const
{
    // Check first the HOME environmental variable
    const char *HomeDir = getenv("HOME");

    // ...otherwise get the home directory of the current user.
    if (!HomeDir)
    {
        HomeDir = getpwuid(getuid())->pw_dir;
    }
    return (HomeDir ? std::string(HomeDir) : "");
}

int Settings::GetScopeConfigIndex() const
{
    if (IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG1") && IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG1")->s == ISS_ON)
    {
        return 1;
    }
    if (IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG2") && IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG2")->s == ISS_ON)
    {
        return 2;
    }
    if (IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG3") && IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG3")->s == ISS_ON)
    {
        return 3;
    }
    if (IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG4") && IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG4")->s == ISS_ON)
    {
        return 4;
    }
    if (IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG5") && IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG5")->s == ISS_ON)
    {
        return 5;
    }
    if (IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG6") && IUFindSwitch(&ScopeConfigsSP, "SCOPE_CONFIG6")->s == ISS_ON)
    {
        return 6;
    }
    return 0;
}

bool Settings::CheckFile(const std::string &file_name, bool writable) const
{
    FILE *FilePtr = fopen(file_name.c_str(), (writable ? "a" : "r"));

    if (FilePtr)
    {
        fclose(FilePtr);
        return true;
    }
    return false;
}

void Settings::sendTimeFromSystem()
{
    char ts[32]={0};

    std::time_t t = std::time(nullptr);
    struct std::tm *utctimeinfo = std::gmtime(&t);

    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", utctimeinfo);
    IUSaveText(&TimeT[0], ts);

    struct std::tm *localtimeinfo = std::localtime(&t);
    snprintf(ts, sizeof(ts), "%4.2f", (localtimeinfo->tm_gmtoff / 3600.0));
    IUSaveText(&TimeT[1], ts);

    TimeTP.s = IPS_OK;

    IDSetText(&TimeTP, nullptr);
}

}
