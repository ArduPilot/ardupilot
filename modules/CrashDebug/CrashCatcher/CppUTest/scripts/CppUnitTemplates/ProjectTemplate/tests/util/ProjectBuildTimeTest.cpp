#include "CppUTest/TestHarness.h"
#include "ProjectBuildTime.h"

TEST_GROUP(ProjectBuildTime)
{
  ProjectBuildTime* projectBuildTime;

  void setup()
  {
    projectBuildTime = new ProjectBuildTime();
  }
  void teardown()
  {
    delete projectBuildTime;
  }
};

TEST(ProjectBuildTime, Create)
{
  CHECK(0 != projectBuildTime->GetDateTime());
}

