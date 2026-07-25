#ifndef D_ProjectBuildTime_H
#define D_ProjectBuildTime_H

///////////////////////////////////////////////////////////////////////////////
//
//  ProjectBuildTime is responsible for recording and reporting when
//  this project library was built
//
///////////////////////////////////////////////////////////////////////////////

class ProjectBuildTime
  {
  public:
    explicit ProjectBuildTime();
    virtual ~ProjectBuildTime();
    
    const char* GetDateTime();

  private:
      
    const char* dateTime;

    ProjectBuildTime(const ProjectBuildTime&);
    ProjectBuildTime& operator=(const ProjectBuildTime&);

  };

#endif  // D_ProjectBuildTime_H
