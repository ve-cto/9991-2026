package frc.robot;

/**
 * Robot-wide numerical or boolean constants.
 *
 * <p>Keep this class free of any dependencies on WPILib classes so it can be used by
 * robot code before hardware is constructed.
 */
public final class Constants {
    public static final class Hardware {
        public static final int kIntakerId = 5;
        public static final int kArmId = 6;
        public static final int kArmEncoderAChannel = 0;
        public static final int kArmEncoderBChannel = 1;
    }

    public static final class Intake {
        public static final double kIntakeForwardSpeed = 0.5;
        public static final double kIntakeReverseSpeed = -0.5;
        public static final double kArmExtendedPosition = 1000;
        public static final double kArmRetractedPosition = 0;
    }

    public static final class Swerve {
        // Fractional deadband applied to requested velocities (10%)
        public static final double kDeadbandFraction = 0.1;

        // Maximum angular rate (rotations per second) used to compute rad/s in RobotContainer
        public static final double kMaxAngularRps = 0.75; // 3/4 rotation per second
    }

    public static final class Controller {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class Vision {
        public static final String kCameraNameAlpha = "limelight3g";
        public static final String kCameraNameBeta = "null";
    }

    public static class Led {
        public static final int l_ledID = 0;
        public static final int kLedLength = 50;
        
        public static enum StatusList {
            DISCONNECT,
            DISABLED,
            IDLE,           
            AUTONOMOUS,
            ALIGNED,        // Aligned to the hub?
            UNALIGNED,
            VISIBLE,        // Is the hub visible?
            NOTVISIBLE,
            BLANK
        }
    }

    
    private Constants() {}
}
