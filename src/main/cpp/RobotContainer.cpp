#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <RobotContainer.h>

using namespace pathplanner;

frc2::CommandPtr RobotContainer::getAutonomousCommand(){
    std::string name = "note2";
    
    // Load the path you want to follow using its name in the GUI
    auto path = PathPlannerPath::fromPathFile(name);

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder::followPath(path);
}
