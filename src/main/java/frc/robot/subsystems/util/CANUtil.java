// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

@SuppressWarnings("resource")
public class CANUtil extends SubsystemBase {
  private static final CANUtil instance = new CANUtil();
  public static CANUtil getInstance() { 
    return instance;
  }

  private Timer refreshTimer = new Timer();

  public record CANDeviceEntry(
    String name,
    int canId,
    Constants.Hardware.DeviceType type,
    Object device,
    Alert alert,
    Alert alert2,
    Alert alert3,
    Alert alert4
  ) {}

  private static final List<CANDeviceEntry> devices = Collections.synchronizedList(new ArrayList<>());

  // alert for when >=1 devices are disconencted (does not go to CANAlerts_ list, goes to main alert list)
  private Alert anyDisconnectAlert = new Alert("", AlertType.kError);
  private boolean pigeonChecked = false;

  public CANUtil() {
    SmartDashboard.putBoolean("Clear CAN Alerts", false);
    refreshTimer.reset();
    refreshTimer.start();
  }

  public void clearAlerts() {
    System.out.println("press");
    synchronized (devices) {
      for (CANDeviceEntry entry : devices) {
        entry.alert.set(false);
        entry.alert2.set(false);
        entry.alert3.set(false);
      }
    }
    anyDisconnectAlert.set(false);
  }

  public void periodic() {
    if (SmartDashboard.getBoolean("Clear CAN Alerts", false)) {
      clearAlerts();
      SmartDashboard.putBoolean("Clear CAN Alerts", false);
    }

    if (refreshTimer.hasElapsed(1)) {
      boolean anyDisconnect = false;
      int disconnectCount = 0;

      synchronized (devices) {
        for (CANDeviceEntry entry : devices) {
          checkAlive(entry);
          checkCANcoderMagnetFault(entry);
          checkUnstableSupplyVoltageFault(entry);
          if (!pigeonChecked) {
            checkPigeonStartupFault(entry);
          }
          if (entry.alert.get()) { // if disconnected
            disconnectCount += 1;
            anyDisconnect = true;
          }
        }
      }
      if (anyDisconnect) {
        anyDisconnectAlert.setText(String.format("%s device/s are disconnected from the CAN bus.", disconnectCount));
      }
      anyDisconnectAlert.set(anyDisconnect);
      refreshTimer.restart();
    } else {
      return;
    }
  }

  public void checkAlive(CANDeviceEntry entry) {
    if (entry.type == Constants.Hardware.DeviceType.TalonFX) {
      TalonFX fx = (TalonFX) entry.device();
      if (!fx.isConnected()) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
      return;
    } else if (entry.type == Constants.Hardware.DeviceType.VictorSPX) {
      VictorSPX spx = (VictorSPX) entry.device();
      if (spx.getBusVoltage() == 0 || Double.isNaN(spx.getBusVoltage())) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
      return;
    } else if (entry.type == Constants.Hardware.DeviceType.CANcoder) {
      CANcoder cc = (CANcoder) entry.device();
      if (!cc.isConnected()) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
    } else if (entry.type == Constants.Hardware.DeviceType.Pigeon) {
      Pigeon2 pig = (Pigeon2) entry.device();
      if (!pig.isConnected()) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
    }
  }

  public void updateDisconnectAlert(CANDeviceEntry entry, boolean disconnected) {
    if (!disconnected && entry.alert.get()) {
      entry.alert2.setText(String.format("Device '%s' (ID %s) was previously disconnected from the CAN bus, but has been reconnected. Timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
      entry.alert2.set(true);
    }

    if (disconnected) {
      entry.alert2.set(false);
      entry.alert.setText(String.format("Device '%s' (ID %s) is disconnected from the CAN bus. Timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
      entry.alert.set(true);
    } else {
      entry.alert.set(false);
    }
  }

  public void registerDevice(String deviceName, int deviceId, Constants.Hardware.DeviceType deviceType, Object device) {
    if (deviceType == Constants.Hardware.DeviceType.TalonFX) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, device, new Alert("CANAlerts_","Connected?", AlertType.kError), new Alert("CANAlerts_","Last time connected", AlertType.kWarning), new Alert("CANAlerts_","undervoltage", AlertType.kWarning), new Alert("CANAlerts_","", AlertType.kWarning));
      devices.add(entry);
    } else if (deviceType == Constants.Hardware.DeviceType.VictorSPX) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, device, new Alert("CANAlerts_","Connected?", AlertType.kError), new Alert("CANAlerts_","Last time connected", AlertType.kWarning), new Alert("CANAlerts_","undervoltage", AlertType.kWarning), new Alert("CANAlerts_","", AlertType.kWarning));
      devices.add(entry);
    } else if (deviceType == Constants.Hardware.DeviceType.CANcoder) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, device, new Alert("CANAlerts_","Connected?", AlertType.kError), new Alert("CANAlerts_","Last time connected", AlertType.kWarning), new Alert("CANAlerts_","undervoltage", AlertType.kWarning), new Alert("CANAlerts_","incorrect magnet positioning", AlertType.kWarning));
      devices.add(entry);
    } else if (deviceType == Constants.Hardware.DeviceType.Pigeon) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, device, new Alert("CANAlerts_","Connected?", AlertType.kError), new Alert("CANAlerts_","Last time connected", AlertType.kWarning), new Alert("CANAlerts_","undervoltage", AlertType.kWarning), new Alert("CANAlerts_","problem during startup", AlertType.kWarning));
      devices.add(entry);
    }
    // System.out.println(String.format("Device %s with ID %s of type %s has been successfully registered to CANUtil", deviceName, deviceId, deviceType));
  }

  // public void checkFeatureUnlicensedFault(CANDeviceEntry entry) {
  //   if (entry.type == Constants.Hardware.DeviceType.TalonFX) {
  //     TalonFX fx = (TalonFX) entry.device();
  //     fx.getStickyFault_UnlicensedFeatureInUse();
  //   }
  // }

  public void checkUnstableSupplyVoltageFault(CANDeviceEntry entry) {
    if (entry.type == Constants.Hardware.DeviceType.TalonFX) {
      TalonFX fx = (TalonFX) entry.device();
      if (fx.getFault_UnstableSupplyV().getValue() || fx.getFault_Undervoltage().getValue()) {
        entry.alert3.setText(String.format("Device '%s' (ID %s) has reported unstable power supply and/or undervoltage most recently at timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
        entry.alert3.set(true);
      } 
      return;
    } else if (entry.type == Constants.Hardware.DeviceType.VictorSPX) {
      VictorSPX spx = (VictorSPX) entry.device();
      Faults f = new Faults();
      spx.getFaults(f);
      if (f.SupplyUnstable || f.SupplyOverV || f.UnderVoltage) {
        entry.alert3.setText(String.format("Device '%s' (ID %s) has reported unstable power supply and/or undervoltage most recently at timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
        entry.alert3.set(true);
      }
      return;
    } else if (entry.type == Constants.Hardware.DeviceType.CANcoder) {
      CANcoder cc = (CANcoder) entry.device();
      if (cc.getFault_Undervoltage().getValue()) {
        entry.alert3.setText(String.format("Device '%s' (ID %s) has reported undervoltage most recently at timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
        entry.alert3.set(true);
      }
    } else if (entry.type == Constants.Hardware.DeviceType.Pigeon) {
      Pigeon2 pig = (Pigeon2) entry.device();
      if (pig.getFault_Undervoltage().getValue()) {
        entry.alert3.setText(String.format("Device '%s' (ID %s) has reported undervoltage most recently at timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
        entry.alert3.set(true);
      }
    }
    return;
  }
  
  public void checkCANcoderMagnetFault(CANDeviceEntry entry) {
    if (entry.type == Constants.Hardware.DeviceType.CANcoder) {
      CANcoder cc = (CANcoder) entry.device();
      if (cc.getFault_BadMagnet().getValue()) {
        entry.alert4.setText(String.format("CANcoder '%s' (ID %s) has detected improper magnet positioning most recently at timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
        entry.alert4.set(true);
        return;
      }
    }
    return;
  }

  public void checkPigeonStartupFault(CANDeviceEntry entry) {
    if (!pigeonChecked) {
      if (entry.type == Constants.Hardware.DeviceType.Pigeon) {
        Pigeon2 pig = (Pigeon2) entry.device();
        if (pig.getStickyFault_BootIntoMotion().getValue() || pig.getStickyFault_BootupAccelerometer().getValue() || pig.getStickyFault_BootupGyroscope().getValue() || pig.getStickyFault_BootupMagnetometer().getValue()) {
          entry.alert4.setText(String.format("Pigeon '%s' (ID %s) has reported one or more issues during startup. Timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
          entry.alert4.set(true);
          pig.clearStickyFaults();
        }
        pigeonChecked = true;
      }
      return;
    }
  }
}
