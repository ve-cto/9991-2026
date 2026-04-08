package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase {
    private Timer flashTimer;
    private boolean isFlashing;
    private int flashCount;
    private int totalFlashes;
    private double flashSpeed;
    private Constants.Led.StatusList flashStatus;
    private boolean ledOn;
    
    private AddressableLED l_led;

    private AddressableLEDBuffer l_ledBuffer;

    private Constants.Led.StatusList Status = Constants.Led.StatusList.DISCONNECT; // set to disconnect at start to avoid nullpointer

    // DISCONNECT,
    // DISABLED,
    // TELEOP,           
    // AUTONOMOUS,
    // ALIGNED,        // Aligned to the hub?
    // UNALIGNED,
    // BLANK

    private LEDPattern robotDisconnectMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.1, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(5));
    private LEDPattern robotDisconnectBase = LEDPattern.solid(Color.kDarkRed); 
    private LEDPattern robotDisconnect = robotDisconnectBase.mask(robotDisconnectMask).atBrightness(Percent.of(100));

    private LEDPattern robotDisabledMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern robotDisabledBase = LEDPattern.gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kDarkRed);
    private LEDPattern robotDisabled = robotDisabledBase.mask(robotDisabledMask).atBrightness(Percent.of(100));

    private LEDPattern robotEStoppedBase = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kOrange).atBrightness(Percent.of(100));
    // private LEDPattern robotEStopped = robotEStoppedBase.breathe(Seconds.of(3)).atBrightness(Percent.of(100));
    private LEDPattern robotEStopped = robotEStoppedBase.atBrightness(Percent.of(100));

    private LEDPattern robotTeleopMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.4, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern robotTeleopBase = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kPurple).scrollAtRelativeSpeed(Percent.per(Second).of(20));
    private LEDPattern robotTeleop = robotTeleopBase.mask(robotTeleopMask).atBrightness(Percent.of(100));

    private LEDPattern robotAutonomousMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.8, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    private LEDPattern robotAutonomousBase = LEDPattern.rainbow(255, 200).scrollAtRelativeSpeed(Percent.per(Second).of(25));
    private LEDPattern robotAutonomous = robotAutonomousBase.mask(robotAutonomousMask).atBrightness(Percent.of(100));

    private LEDPattern robotAligned = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(100));
    private LEDPattern robotUnaligned = LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(100));

    private LEDPattern ledBlank = LEDPattern.solid(Color.kBlack);

    // construct
    // !! only do this once, the robot will literally kill itself if multiple instances are made !!
    public Led() {
        l_led = new AddressableLED(Constants.Hardware.kLedId);
        l_ledBuffer = new AddressableLEDBuffer(Constants.Hardware.kLedLength);
        l_led.setLength(l_ledBuffer.getLength());
        l_led.setData(l_ledBuffer);
        this.isFlashing = false;
        this.flashTimer = new Timer();
        l_led.start();
    }

    public void periodic() {
        updateFlashing();
        try {
                switch (this.Status) {
                    case DISCONNECT:
                        robotDisconnect.applyTo(this.l_ledBuffer);
                        break;
                    case DISABLED:
                        robotDisabled.applyTo(this.l_ledBuffer);
                        break;
                    case TELEOP:
                        robotTeleop.applyTo(this.l_ledBuffer);
                        break;
                    case AUTONOMOUS:
                        robotAutonomous.applyTo(this.l_ledBuffer);
                        break;
                    case ALIGNED:
                        robotAligned.applyTo(this.l_ledBuffer);
                        break;
                    case UNALIGNED:
                        robotUnaligned.applyTo(this.l_ledBuffer);
                        break;
                    case ESTOPPED:
                        robotEStopped.applyTo(this.l_ledBuffer);
                        break;
                    case BLANK:
                        ledBlank.applyTo(this.l_ledBuffer);
                        break;
                }
        } catch (NullPointerException e) {}

        l_led.setData(l_ledBuffer);
    }

    /*
     * Set the LEDs to a status.
     */
    public void setStatus(Constants.Led.StatusList desiredStatus) {
        this.Status = desiredStatus;
    }

    /*
     * flash the led's with a specific status a specific number of times at a specific speed (seconds)
     */
    public void startFlashing(Constants.Led.StatusList desiredStatus, int numFlashes, double speed) {
        // If it's retriggered - ignore it
        // if (!isFlashing) {
        //     isFlashing = true;
        //     flashCount = 0;
        //     totalFlashes = numFlashes;
        //     flashSpeed = speed;
        //     flashStatus = desiredStatus;
        //     ledOn = false;
        //     flashTimer.reset();
        //     flashTimer.start();
        //     System.out.println("Started flashing: " + desiredStatus);
        // }
        // System.out.println("Started flashing!");
        // If startFlashing is retriggered, reset.
        isFlashing = true;
        flashCount = 0;
        totalFlashes = numFlashes+1;
        flashSpeed = speed;
        flashStatus = desiredStatus;
        ledOn = false;
        flashTimer.reset();
        flashTimer.start();
        // System.out.println("Started flashing: " + desiredStatus);
    }

    /*
     * update the flashing logic, called in subsystem periodic
     */
    public void updateFlashing() {
        if (isFlashing) {
            if (flashTimer.get() >= flashSpeed) {
                // System.out.println("Timer reset!");
                flashTimer.reset();
                ledOn = !ledOn;

                if (ledOn) {
                    setStatus(flashStatus);
                } else {
                    setStatus(frc.robot.Constants.Led.StatusList.BLANK);
                    flashCount++;
                }

                if (flashCount >= totalFlashes) {
                    isFlashing = false;
                    setStatus(flashStatus);
                    // System.out.println("Flashing completed");
                }
            }
        }
    }

    /*
     * if the led's are flashing, cancel it
     * if they aren't flashing, does nothing
     */
    public void stopFlashing() {
        if (isFlashing) {
            isFlashing = false;
        }
    }

    /*
     * if the led's are flashing, return true
     */
    public boolean getFlashing() {
        return this.isFlashing;
    }
    
    /*
     * return what the led's are currently displaying
     */
    public Constants.Led.StatusList getStatus() {
        return this.Status;
    }

    /*
     * first causes the led's to stop flashing, and then sets them to be blank
     */
    public Command clear() {
        return runOnce(() -> this.stopFlashing()).andThen(() -> this.setStatus(Constants.Led.StatusList.BLANK));
    }

    /*
     * tell the led's to show a status
     */
    public Command display(Constants.Led.StatusList status) {
        return runOnce(() -> this.setStatus(status));
    }

    /*
     * flash the led's with a specific status a specific number of times at a specific speed (seconds)
     */
    public Command flash(Constants.Led.StatusList status, int numFlashes, double speed) {
        return runOnce(() -> this.startFlashing(status, numFlashes, speed)).andThen(() -> this.setStatus(Constants.Led.StatusList.ESTOPPED));
    }

    /*
     * display either the teleop status OR the autonomous status depending on which mode is enabled.
     * fetched from the DriverStation when called.
     */
    public Command displayTeleAuto() {
        return runOnce(() -> {
            if (DriverStation.isTeleop()) {
                this.setStatus(Constants.Led.StatusList.TELEOP); 

            } else {
                this.setStatus(Constants.Led.StatusList.AUTONOMOUS);
            }
        });
    }

    /* 
     * inform the led's that the robot has been estopped
     * causes them to flash, then hold a specific status 
     */
    public Command estop() {
        return runOnce(() -> {
            startFlashing(Constants.Led.StatusList.ESTOPPED, 5, 0.1);
        });
    }

    /*
     * display the default status for whatever state the robot is in
     * if DS is disconnected, show disconnect
     * if disabled, show disabled
     * if auto, show auto
     * if teleop, show teleop
     * shocking, i know, my pants have been blown off
     */
    public Command handleDefault() {
        return runOnce(() -> {
            if (DriverStation.isEStopped()) {
                return;
            } else {
                if (!DriverStation.isDSAttached()) {
                    this.setStatus(Constants.Led.StatusList.DISCONNECT);
                } else if (DriverStation.isDisabled()) {
                    this.setStatus(Constants.Led.StatusList.DISABLED);
                } else if (DriverStation.isAutonomousEnabled()) {
                    this.setStatus(Constants.Led.StatusList.AUTONOMOUS);
                } else if (DriverStation.isTeleopEnabled()) {
                    this.setStatus(Constants.Led.StatusList.TELEOP);
                }
            }
        }).ignoringDisable(true);
    }
}
