
#pragma once

#include "AlignmentSubsystemForMathPlugins.h"
#include "ConvexHull.h"

namespace INDI
{
namespace AlignmentSubsystem
{
class DummyMathPlugin : public AlignmentSubsystemForMathPlugins
{
  public:
    DummyMathPlugin();
    virtual ~DummyMathPlugin();

    virtual bool Initialise(InMemoryDatabase *pInMemoryDatabase);

    virtual bool TransformCelestialToMount(const double RightAscension, const double Declination,
                                               double JulianOffset,
                                               MountDirectionVector &ApparentMountDirectionVector);

    virtual bool TransformMountToCelestial(const MountDirectionVector &ApparentMountDirectionVector,
                                               double &RightAscension, double &Declination);
};

} // namespace AlignmentSubsystem
} // namespace INDI
