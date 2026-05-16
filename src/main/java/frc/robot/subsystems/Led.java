package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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

    private LEDPattern pDisconnectMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.1, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(5));
    private LEDPattern pDisconnectBase = LEDPattern.solid(Color.kDarkRed); 
    private LEDPattern pDisconnect = pDisconnectBase.mask(pDisconnectMask).atBrightness(Percent.of(100));

    private LEDPattern pDisabledMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern pDisabledBase = LEDPattern.gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kDarkRed);
    private LEDPattern pDisabled = pDisabledBase.mask(pDisabledMask).atBrightness(Percent.of(100));

    private LEDPattern pEStoppedBase = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kOrange).atBrightness(Percent.of(100));
    // private LEDPattern pEStopped = pEStoppedBase.breathe(Seconds.of(3)).atBrightness(Percent.of(100));
    private LEDPattern pEStopped = pEStoppedBase.atBrightness(Percent.of(100));

    private LEDPattern pTeleopMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.4, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(10));
    private LEDPattern pTeleopBase = LEDPattern.gradient(GradientType.kContinuous, Color.kBlue, Color.kPurple).scrollAtRelativeSpeed(Percent.per(Second).of(20));
    private LEDPattern pTeleop = pTeleopBase.mask(pTeleopMask).atBrightness(Percent.of(100));

    private LEDPattern pAutonomousMask = LEDPattern.steps(Map.of(0, Color.kWhite, 0.8, Color.kBlack)).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    private LEDPattern pAutonomousBase = LEDPattern.rainbow(255, 200).scrollAtRelativeSpeed(Percent.per(Second).of(25));
    private LEDPattern pAutonomous = pAutonomousBase.mask(pAutonomousMask).atBrightness(Percent.of(100));

    private LEDPattern pAligned = LEDPattern.solid(Color.kLimeGreen).atBrightness(Percent.of(100));
    private LEDPattern pNotready = LEDPattern.solid(Color.kCrimson).atBrightness(Percent.of(100));

    private double progressVar1 = 0;
    private double progressVar2 = 0;
    private double progressVar3 = 0;
    private java.util.function.DoubleSupplier progressSup1 = () -> 0.0;
    private java.util.function.DoubleSupplier progressSup2 = () -> 0.0;
    private java.util.function.DoubleSupplier progressSup3 = () -> 0.0;
    private LEDPattern progressMask1 = LEDPattern.progressMaskLayer(() -> progressVar1);
    private LEDPattern progressMask2 = LEDPattern.progressMaskLayer(() -> progressVar2);
    private LEDPattern progressMask3 = LEDPattern.progressMaskLayer(() -> progressVar3);

    private LEDPattern topBottomBorderBase = LEDPattern.steps(Map.of(0, Color.kRed, 0.05, Color.kBlack, 0.49, Color.kGreen, 0.51, Color.kBlack));
    private LEDPattern pShooterSetpointBase = LEDPattern.solid(Color.kMediumTurquoise).atBrightness(Percent.of(100));
    private LEDPattern pShooterSetpoint = topBottomBorderBase.overlayOn(pShooterSetpointBase.mask(progressMask1));

    // ready
    // shooting - progressmask for shooter setpoint +- 200? red at bottom green at top?
    // reverse

    // LEDPattern sycned = base.synchronizedBlink(RobotController::getRSLState);

    public void setProgressVar(int id, DoubleSupplier prog) {
        // store the supplier so the progress value can be updated continuously
        if (id == 1) {
            this.progressSup1 = prog;
            this.progressVar1 = prog.getAsDouble();
        } else if (id == 2) {
            this.progressSup2 = prog;
            this.progressVar2 = prog.getAsDouble();
        } else if (id == 3) {
            this.progressSup3 = prog;
            this.progressVar3 = prog.getAsDouble();
        }
    }

    public Command setProgressVarCommand(int id, DoubleSupplier progress) {
        return runOnce(() -> this.setProgressVar(id, progress));
    }

    public Command displayShooterSepoint(DoubleSupplier progress) {
        return new FunctionalCommand(() -> setProgressVar(1, progress), () -> setStatus(Constants.Led.StatusList.SHOOTING), interrupted -> nothing(), () -> false, this);
    }

    public void nothing() {}

    // public Command blank() {
    //     return new FunctionalCommand(null, null, null, null, null);
    // }
    
    // private double progressVar2 = 0;
    // private double progressVar3 = 0;

    private LEDPattern ledBlank = LEDPattern.solid(Color.kBlack);

    // construct
    // !! only do this once, the p will literally kill itself if multiple instances are made !!
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
        // update progress vars from their suppliers so progress masks reflect live values
        try {
            this.progressVar1 = this.progressSup1.getAsDouble();
        } catch (Exception e) {
            // keep previous value on error
        }
        try {
            this.progressVar2 = this.progressSup2.getAsDouble();
        } catch (Exception e) {}
        try {
            this.progressVar3 = this.progressSup3.getAsDouble();
        } catch (Exception e) {}

        updateFlashing();
        try {
                switch (this.Status) {
                    case DISCONNECT:
                        pDisconnect.applyTo(this.l_ledBuffer);
                        break;
                    case DISABLED:
                        pDisabled.applyTo(this.l_ledBuffer);
                        break;
                    case ESTOPPED:
                        pEStopped.applyTo(this.l_ledBuffer);
                        break;
                    case BLANK:
                        ledBlank.applyTo(this.l_ledBuffer);
                        break;
                    case TELEOP:
                        pTeleop.applyTo(this.l_ledBuffer);
                        break;
                    case AUTONOMOUS:
                        pAutonomous.applyTo(this.l_ledBuffer);
                        break;
                    case ALIGNED:
                        pAligned.applyTo(this.l_ledBuffer);
                        break;
                    case NOTREADY:
                        pNotready.applyTo(this.l_ledBuffer);
                        break;
                    case SHOOTING:
                        pShooterSetpoint.applyTo(this.l_ledBuffer);
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
                    setStatus(Constants.Led.StatusList.BLANK);
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
        return run(() -> this.stopFlashing()).andThen(() -> this.setStatus(Constants.Led.StatusList.BLANK));
    }

    /*
     * tell the led's to show a status
     */
    public Command display(Constants.Led.StatusList status) {
        return run(() -> this.setStatus(status));
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
        return run(() -> {
            if (DriverStation.isTeleop()) {
                this.setStatus(Constants.Led.StatusList.TELEOP); 

            } else {
                this.setStatus(Constants.Led.StatusList.AUTONOMOUS);
            }
        });
    }

    /* 
     * inform the led's that the p has been estopped
     * causes them to flash, then hold a specific status 
     */
    public Command estop() {
        return runOnce(() -> {
            startFlashing(Constants.Led.StatusList.ESTOPPED, 5, 0.1);
        });
    }

    /*
     * display the default status for whatever state the p is in
     * if DS is disconnected, show disconnect
     * if disabled, show disabled
     * if auto, show auto
     * if teleop, show teleop
     * shocking, i know, my pants have been blown off
     */
    public Command handleDefault() {
        return run(() -> {
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
                } else if (DriverStation.isTestEnabled()) {
                    this.setStatus(Constants.Led.StatusList.TELEOP);
                }
            }
        }).ignoringDisable(true);
    }
}
