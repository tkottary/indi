/*!
 * \file AlignmentSubsystemForDrivers.cpp
 *
 * \author Roger James
 * \date 13th November 2013
 *
 */

#include "AlignmentSubsystemForDrivers.h"

namespace INDI
{
namespace AlignmentSubsystem
{
AlignmentSubsystemForDrivers::AlignmentSubsystemForDrivers()
{
    // Set up the in memory database pointer for math plugins
    SetCurrentInMemoryDatabase(this);
    // Tell the built in math plugin about it
    Initialise(this);
    // Fix up the database load callback
    SetLoadDatabaseCallback(&MyDatabaseLoadCallback, this);
}

// Public methods

void AlignmentSubsystemForDrivers::InitAlignmentProperties(Mount *pMount)
{
    MapPropertiesToInMemoryDatabase::InitProperties(pMount);
    MathPluginManagement::InitProperties(pMount);
}

void AlignmentSubsystemForDrivers::ProcessAlignmentBLOBProperties(Mount *pMount, const char *name, int sizes[],
                                                                  int blobsizes[], char *blobs[], char *formats[],
                                                                  char *names[], int n)
{
    MapPropertiesToInMemoryDatabase::ProcessBlobProperties(pMount, name, sizes, blobsizes, blobs, formats, names,
                                                           n);
}

void AlignmentSubsystemForDrivers::ProcessAlignmentNumberProperties(Mount *pMount, const char *name,
                                                                    double values[], char *names[], int n)
{
    MapPropertiesToInMemoryDatabase::ProcessNumberProperties(pMount, name, values, names, n);
}

void AlignmentSubsystemForDrivers::ProcessAlignmentSwitchProperties(Mount *pMount, const char *name,
                                                                    ISState *states, char *names[], int n)
{
    MapPropertiesToInMemoryDatabase::ProcessSwitchProperties(pMount, name, states, names, n);
    MathPluginManagement::ProcessSwitchProperties(pMount, name, states, names, n);
}

void AlignmentSubsystemForDrivers::ProcessAlignmentTextProperties(Mount *pMount, const char *name,
                                                                  char *texts[], char *names[], int n)
{
    MathPluginManagement::ProcessTextProperties(pMount, name, texts, names, n);
}

void AlignmentSubsystemForDrivers::SaveAlignmentConfigProperties(FILE *fp)
{
    MathPluginManagement::SaveConfigProperties(fp);
}

// Private methods

void AlignmentSubsystemForDrivers::MyDatabaseLoadCallback(void *ThisPointer)
{
    ((AlignmentSubsystemForDrivers *)ThisPointer)->Initialise((AlignmentSubsystemForDrivers *)ThisPointer);
}

} // namespace AlignmentSubsystem
} // namespace INDI
