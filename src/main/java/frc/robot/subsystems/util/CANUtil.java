// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.jni.CANCommonJNI;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    Object container,
    Alert alert,
    Alert alertPrev
  ) {}

  private static final List<CANDeviceEntry> devices = Collections.synchronizedList(new ArrayList<>());

  // alert for when >=1 devices are disconencted (does not go to CANAlerts_) list, goes to main alert list
  private Alert anyDisconnectAlert = new Alert("", AlertType.kError);

  public CANUtil() {
    refreshTimer.reset();
    refreshTimer.start();
  }

  @Override
  public void periodic() {
    if (refreshTimer.hasElapsed(0.25)) {

      boolean anyDisconnect = false;
      int disconnectCount = 0;

      synchronized (devices) {
        for (CANDeviceEntry entry : devices) {
          checkAlive(entry);
          if (entry.alert.get()) {
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
      TalonFX fx = (TalonFX) entry.container();
      if (!fx.isConnected()) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
      return;
    } else if (entry.type == Constants.Hardware.DeviceType.VictorSPX) {
      VictorSPX spx = (VictorSPX) entry.container();
      if (spx.getBusVoltage() == 0 || Double.isNaN(spx.getBusVoltage())) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
      return;
    } else if (entry.type == Constants.Hardware.DeviceType.CANcoder) {
      CANcoder cc = (CANcoder) entry.container();
      if (!cc.isConnected()) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
    } else if (entry.type == Constants.Hardware.DeviceType.Pigeon) {
      Pigeon2 pig = (Pigeon2) entry.container();
      if (!pig.isConnected()) {
        updateDisconnectAlert(entry, true);
      } else {
        updateDisconnectAlert(entry, false);
      }
    }
  }

  public void updateDisconnectAlert(CANDeviceEntry entry, boolean disconnected) {
    if (!disconnected && entry.alert.get()) {
      entry.alertPrev.setText(String.format("CAN device '%s' (ID %s) was previously disconnected from the CAN bus, but has been reconnected. Timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
      entry.alertPrev.set(true);
    }

    if (disconnected) {
      entry.alertPrev.set(false);
      entry.alert.setText(String.format("CAN device '%s' (ID %s) is disconnected from the CAN bus. Timestamp: %s", entry.name, entry.canId, Timer.getFPGATimestamp()));
      entry.alert.set(true);
    } else {
      entry.alert.set(false);
    }
  }

  public void registerDevice(String deviceName, int deviceId, Constants.Hardware.DeviceType deviceType) {
    if (deviceType == Constants.Hardware.DeviceType.TalonFX) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, new TalonFX(deviceId), new Alert("CANAlerts_","", AlertType.kError), new Alert("CANAlerts_","", AlertType.kWarning));
      devices.add(entry);
    } else if (deviceType == Constants.Hardware.DeviceType.VictorSPX) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, new VictorSPX(deviceId), new Alert("CANAlerts_","", AlertType.kError), new Alert("CANAlerts_","", AlertType.kWarning));
      devices.add(entry);
    } else if (deviceType == Constants.Hardware.DeviceType.CANcoder) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, new CANcoder(deviceId), new Alert("CANAlerts_","", AlertType.kError), new Alert("CANAlerts_","", AlertType.kWarning));
      devices.add(entry);
    } else if (deviceType == Constants.Hardware.DeviceType.Pigeon) {
      CANDeviceEntry entry = new CANDeviceEntry(deviceName, deviceId, deviceType, new Pigeon2(deviceId), new Alert("CANAlerts_","", AlertType.kError), new Alert("CANAlerts_","", AlertType.kWarning));
      devices.add(entry);
    }
    // System.out.println(String.format("Device %s with ID %s of type %s has been successfully registered to CANUtil", deviceName, deviceId, deviceType));
  }
}
