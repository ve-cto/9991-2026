package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase{
    private Timer flashTimer;
    private boolean isFlashing;
    private int flashCount;
    private int totalFlashes;
    private double flashSpeed;
    private Constants.Led.StatusList flashStatus;
    private boolean ledOn;
    
    private AddressableLED l_led;

    private AddressableLEDBuffer l_ledBuffer;

    private Constants.Led.StatusList Status;

    // DISCONNECT,
    // DISABLED,
    // IDLE,           
    // AUTONOMOUS,
    // ALIGNED,        // Aligned to the hub?
    // UNALIGNED,
    // VISIBLE,        // Is the hub visible?
    // NOTVISIBLE,
    // BLANK

    private LEDPattern robotDisconnectMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.07, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(5));
    private LEDPattern robotDisconnectBase = LEDPattern.solid(Color.kDarkRed); 
    private LEDPattern robotDisconnect = robotDisconnectBase.mask(robotDisconnectMask).atBrightness(Percent.of(20));

    private LEDPattern robotDisabledMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.3, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern robotDisabledBase = LEDPattern.gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kDarkRed);
    private LEDPattern robotDisabled = robotDisabledBase.mask(robotDisabledMask).atBrightness(Percent.of(10));

    private LEDPattern robotIdleMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.4, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern robotIdleBase = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kPurple).scrollAtRelativeSpeed(Percent.per(Second).of(20));
    private LEDPattern robotIdle = robotIdleBase.mask(robotIdleMask).atBrightness(Percent.of(30));

    private LEDPattern robotAutonomousMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.6, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    private LEDPattern robotAutonomousBase = LEDPattern.rainbow(255, 200).scrollAtRelativeSpeed(Percent.per(Second).of(25));
    private LEDPattern robotAutonomous = robotAutonomousBase.mask(robotAutonomousMask).atBrightness(Percent.of(50));

    private LEDPattern robotAligned = LEDPattern.solid(Color.kGreen).atBrightness(Percent.of(70));

    private LEDPattern robotUnaligned = LEDPattern.solid(Color.kYellow).atBrightness(Percent.of(50));

    private LEDPattern robotVisible = LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(50));

    private LEDPattern robotNotVisible = LEDPattern.solid(Color.kPurple).atBrightness(Percent.of(50));

    private LEDPattern ledBlank = LEDPattern.solid(Color.kBlack);

    public Led() {
        l_led = new AddressableLED(Constants.Led.l_ledID);
        l_ledBuffer = new AddressableLEDBuffer(Constants.Led.kLedLength);
        l_led.setLength(l_ledBuffer.getLength());
        l_led.setData(l_ledBuffer);
        this.isFlashing = false;
        this.flashTimer = new Timer();
        l_led.start();
    }

    public void periodic() {
        updateFlashing();
    }

    public Constants.Led.StatusList getStatus() {
        return this.Status;
    }

    public void setAndApplyStatus(Constants.Led.StatusList desiredStatus) {
        setStatus(desiredStatus);
    }

    public void setStatus(Constants.Led.StatusList desiredStatus) {
        this.Status = desiredStatus;
        switch (desiredStatus) {
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
            case VISIBLE:
                robotVisible.applyTo(this.l_ledBuffer);
                break;
            case NOTVISIBLE:
                robotNotVisible.applyTo(this.l_ledBuffer);
                break;
            case BLANK:
                ledBlank.applyTo(this.l_ledBuffer);
                break;
        }

        l_led.setData(this.l_ledBuffer);
    }

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
    
        // If startFlashing is retriggered, reset.
        isFlashing = true;
        flashCount = 0;
        totalFlashes = numFlashes;
        flashSpeed = speed;
        flashStatus = desiredStatus;
        ledOn = false;
        flashTimer.reset();
        flashTimer.start();
        // System.out.println("Started flashing: " + desiredStatus);
    }

    public void updateFlashing() {
        if (isFlashing) {
            if (flashTimer.get() >= flashSpeed) {
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
                    setStatus(frc.robot.Constants.Led.StatusList.BLANK);
                    // System.out.println("Flashing completed");
                }
            }
        }
    }

    public boolean getFlashing() {
        return this.isFlashing;
    }

    public void reset() {}
}


