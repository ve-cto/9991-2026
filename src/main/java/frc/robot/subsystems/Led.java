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
    // IDLE,           
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

    private LEDPattern robotIdleMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.4, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern robotIdleBase = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kPurple).scrollAtRelativeSpeed(Percent.per(Second).of(20));
    private LEDPattern robotIdle = robotIdleBase.mask(robotIdleMask).atBrightness(Percent.of(100));

    private LEDPattern robotAutonomousMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.8, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    private LEDPattern robotAutonomousBase = LEDPattern.rainbow(255, 200).scrollAtRelativeSpeed(Percent.per(Second).of(25));
    private LEDPattern robotAutonomous = robotAutonomousBase.mask(robotAutonomousMask).atBrightness(Percent.of(100));

    private LEDPattern robotAligned = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(100));
    private LEDPattern robotUnaligned = LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(100));

    private LEDPattern ledBlank = LEDPattern.solid(Color.kBlack);

    /*
     * Construct a new Led instance.
     * Only call this once, as the robot cannot handle multiple instances. 
     */
    public Led() {
        l_led = new AddressableLED(Constants.Led.l_ledID);
        l_ledBuffer = new AddressableLEDBuffer(Constants.Led.kLedLength);
        l_led.setLength(l_ledBuffer.getLength());
        l_led.setData(l_ledBuffer);
        this.isFlashing = false;
        this.flashTimer = new Timer();
        l_led.start();
    }

    /*
     * Called every 20ms.
     */
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
                case IDLE:
                    robotIdle.applyTo(this.l_ledBuffer);
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
     * Cause the LEDs to flash a specific status a specific number of times, and with a certain delay between each flash.
     * New startFlashing requests override old ones.
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
     * Handles timing flashes on the LED's. This method should be called periodically.
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
     * If the LEDs are currently flashing, end their flash cycle.
     */
    public void stopFlashing() {
        if (isFlashing) {
            isFlashing = false;
        }
    }

    /*
     * Returns true if the LEDs are currently executing a flash command
     */
    public boolean getFlashing() {
        return this.isFlashing;
    }
    
    /*
     * Return the status currently being displayed by the LEDs.
     */
    public Constants.Led.StatusList getStatus() {
        return this.Status;
    }

    /*
     * Schedule a command to clear the LED's. Overrides and stops any flash requests.
     */
    public Command clear() {
        return runOnce(() -> this.stopFlashing()).andThen(() -> this.setStatus(Constants.Led.StatusList.BLANK));
    }

    /*
     * Display a status on the LEDs
     */
    public Command display(Constants.Led.StatusList status) {
        return runOnce(() -> this.setStatus(status));
    }

    /*
     * Flash the LEDs with a specific status. Overrides any other setstatus commands while running.
     */
    public Command flash(Constants.Led.StatusList status, int numFlashes, double speed) {
        return runOnce(() -> this.startFlashing(status, numFlashes, speed));
    }

    /*
     * Display either the teleop status OR the autonomous status depending on which mode is enabled.
     * Fetched from the DriverStation when called.
     */
    public Command displayTeleAuto() {
        return runOnce(() -> {
            if (DriverStation.isTeleop()) {
                this.setStatus(Constants.Led.StatusList.IDLE); 

            } else {
                this.setStatus(Constants.Led.StatusList.AUTONOMOUS);
            }
        });
    }
}


