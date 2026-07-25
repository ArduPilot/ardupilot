#include "ProjectBuildTime.h"

ProjectBuildTime::ProjectBuildTime()
: dateTime(__DATE__ " " __TIME__)
{
}

ProjectBuildTime::~ProjectBuildTime()
{
}

const char* ProjectBuildTime::GetDateTime()
{
    return dateTime;
}

