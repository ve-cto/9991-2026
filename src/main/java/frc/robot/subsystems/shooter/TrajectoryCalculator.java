package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


public class TrajectoryCalculator extends SubsystemBase {
    
    TrajectoryCalculator() {}

    public Translation2d getHubCenterTranslation2dRobotRelative() {
        
        return new Translation2d();
    }

}
