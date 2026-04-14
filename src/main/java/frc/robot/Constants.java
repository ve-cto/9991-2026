package frc.robot;

/**
 * Robot-wide numerical or boolean constants.
 *
 * <p>Keep this class free of any dependencies on WPILib classes so it can be used by
 * robot code before hardware is constructed.
 */
public final class Constants {
    public static final class Hardware {
        // Swerve takes up the CAN network up to value 12, thus, all motors must be assigned ID's of 13 or higher.
        public static final int kIntakeId = 17; // minicim
        public static final int kArmId = 16; // minicim
        public static final int kShooterLId = 19; // kraken
        public static final int kShooterRId = 18; // kraken
        public static final int kLoaderId = 21; // probably a cim or a redline
        
        // rev through bore encoder
        public static final int kArmEncoderAChannel = 2; // blue wire
        public static final int kArmEncoderBChannel = 3; // yellow(?) wire
    
        public static final int kDebugMotor1 = 15;
        public static final int kDebugMotor2 = 99;
        public static final int kDebugMotor3 = 99;
        public static final int kDebugMotor4 = 99; // kraken
        public static final int kDebugMotor5 = 99; // kraken

        public static final int kLedId = 0; // PWM port
        public static final int kLedLength = 101;

        public static final double kMaxKrakenFreeSpeed = 6000;
    }

    public static final class Intake {
        public static final double kIntakeForwardSpeed = 0.6;
        public static final double kIntakeReverseSpeed = -0.9;
        public static final double kArmExtendedPosition = 1000; // TODO: tune
        public static final double kArmRetractedPosition = 0;
    }

    public static final class Swerve {
        // Fractional deadband applied to requested velocities (10%)
        // (if the request is below this value, for example, 5% [0.05], the request will be ignored)
        public static final double kDeadbandFraction = 0.1;

        // Maximum angular rate (rotations per second) used to compute rad/s
        public static final double kMaxAngularRps = 0.75; // 3/4 rotation per second
    }

    public static final class Controller {
        // sticks first, buttons next
        // always play with the sticks before you play with the buttons amirite
        // absolute cinema
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class Vision {
        public static final String kCameraAlphaName = "limelight3g";
        public static final String kCameraBetaName = "PLACEHOLDER"; // currently unused

        public final Integer[] tagsHubRed = {8, 9, 10, 11};
        public final Integer[] tagsHubBlue = {24, 25, 26, 27};
    }

    public static class Led {
        // list of all things the leds could display
        public static enum StatusList {
            DISCONNECT,
            DISABLED,
            TELEOP,           
            AUTONOMOUS,
            ALIGNED,        // Aligned to the hub?
            UNALIGNED,
            ESTOPPED,
            BLANK
        }
    }

    public static class Shooter {
        public static final double kControlRatio = (1.636); // Every 4 rotations of the motor, one rotation of the mechanism
        public static final double kMaxOutput = 0.975; // Limit the maximum percentage output applied to the shooter motors to avoid overdraw
        public static final double setpointDeadband = 50;
    }

    public static class Loader {
        public static final double kLoadSpeed = 0.8;
    }
    
    public static class DS {
        public static enum GameState {
            NONE,
            AUTONOMOUS,
            TRANSITION,
            SHIFT1,
            SHIFT2,
            SHIFT3,
            SHIFT4,
            ENDGAME
        } 
    }

    private Constants() {}
}
