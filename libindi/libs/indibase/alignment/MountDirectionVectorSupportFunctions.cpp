/*!
 * \file MountDirectionVectorSupportFunctions.cpp
 *
 * \author Roger James
 * \date 13th November 2013
 *
 */

#include "MountDirectionVectorSupportFunctions.h"

namespace INDI
{
namespace AlignmentSubsystem
{
void MountDirectionVectorSupportFunctions::SphericalCoordinateFromMountDirectionVector(
    const MountDirectionVector MountDirectionVector, double &AzimuthAngle,
    AzimuthAngleDirection AzimuthAngleDirection, double &PolarAngle, PolarAngleDirection PolarAngleDirection)
{
    if (ANTI_CLOCKWISE == AzimuthAngleDirection)
    {
        if (FROM_AZIMUTHAL_PLANE == PolarAngleDirection)
        {
            AzimuthAngle = atan2(MountDirectionVector.y, MountDirectionVector.x);
            PolarAngle   = asin(MountDirectionVector.z);
        }
        else
        {
            AzimuthAngle = atan2(MountDirectionVector.y, MountDirectionVector.x);
            PolarAngle   = acos(MountDirectionVector.z);
        }
    }
    else
    {
        if (FROM_AZIMUTHAL_PLANE == PolarAngleDirection)
        {
            AzimuthAngle = atan2(-MountDirectionVector.y, MountDirectionVector.x);
            PolarAngle   = asin(MountDirectionVector.z);
        }
        else
        {
            AzimuthAngle = atan2(-MountDirectionVector.y, MountDirectionVector.x);
            PolarAngle   = acos(MountDirectionVector.z);
        }
    }
}

const MountDirectionVector
MountDirectionVectorSupportFunctions::MountDirectionVectorFromSphericalCoordinate(
    const double AzimuthAngle, AzimuthAngleDirection AzimuthAngleDirection, const double PolarAngle,
    PolarAngleDirection PolarAngleDirection)
{
    MountDirectionVector Vector;

    if (ANTI_CLOCKWISE == AzimuthAngleDirection)
    {
        if (FROM_AZIMUTHAL_PLANE == PolarAngleDirection)
        {
            Vector.x = cos(PolarAngle) * cos(AzimuthAngle);
            Vector.y = cos(PolarAngle) * sin(AzimuthAngle);
            Vector.z = sin(PolarAngle);
        }
        else
        {
            Vector.x = sin(PolarAngle) * sin(AzimuthAngle);
            Vector.y = sin(PolarAngle) * cos(AzimuthAngle);
            Vector.z = cos(PolarAngle);
        }
    }
    else
    {
        if (FROM_AZIMUTHAL_PLANE == PolarAngleDirection)
        {
            Vector.x = cos(PolarAngle) * cos(-AzimuthAngle);
            Vector.y = cos(PolarAngle) * sin(-AzimuthAngle);
            Vector.z = sin(PolarAngle);
        }
        else
        {
            Vector.x = sin(PolarAngle) * sin(-AzimuthAngle);
            Vector.y = sin(PolarAngle) * cos(-AzimuthAngle);
            Vector.z = cos(PolarAngle);
        }
    }

    return Vector;
}

} // namespace AlignmentSubsystem
} // namespace INDI
