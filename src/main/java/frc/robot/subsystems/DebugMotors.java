// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DebugMotors extends SubsystemBase {
  private final WPI_VictorSPX m_motor1;
  private final WPI_VictorSPX m_motor2;
  private final WPI_VictorSPX m_motor3;
  private final WPI_VictorSPX m_motor4;
  private final WPI_VictorSPX m_motor5;

  /** Creates a new DebugMotors. */
  public DebugMotors() {
    m_motor1 = new WPI_VictorSPX(Constants.Hardware.kDebugMotor1);
    m_motor2 = new WPI_VictorSPX(Constants.Hardware.kDebugMotor2);
    m_motor3 = new WPI_VictorSPX(Constants.Hardware.kDebugMotor3);
    m_motor4 = new WPI_VictorSPX(Constants.Hardware.kDebugMotor4);
    m_motor5 = new WPI_VictorSPX(Constants.Hardware.kDebugMotor5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /*
   * motorID == value from 1-5, corresponding to debug motor 1-5.
   */
  public void runMotor(int motorID, double speed) {
    if (motorID >= 1 && motorID <= 5) {
      switch (motorID) {
        case 1:
          m_motor1.set(speed);
          break;
        case 2:
          m_motor2.set(speed);
          break;
        case 3:
          m_motor3.set(speed);
          break;
        case 4:
          m_motor4.set(speed);
          break;
        case 5:
          m_motor5.set(speed);
          break;
      }
    } // does nothing if id is out of range
  }

  public void stopMotor(int motorID) {
    switch (motorID) {
      case 1:
        m_motor1.stopMotor();
        break;
      case 2:
        m_motor2.stopMotor();
        break;
      case 3:
        m_motor3.stopMotor();
        break;
      case 4:
        m_motor4.stopMotor();
        break;
      case 5:
        m_motor5.stopMotor();
        break;
    }
  } // does nothing if id is out of range
}
