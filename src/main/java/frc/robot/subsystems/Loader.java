// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {
  private final WPI_VictorSPX m_loader;
  
  /** Creates a new Loader. */
  public Loader() {
    m_loader = new WPI_VictorSPX(Constants.Hardware.kLoaderId);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed) {
    m_loader.set(speed);
  }

  public void stop() {
    m_loader.stopMotor();
  }
}
