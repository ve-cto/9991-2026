// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkTablesIO extends SubsystemBase {
  Pose2d drivetrainPose = new Pose2d();

  double[] networkPose = new double[] {0.0, 0.0, 0.0};
  private boolean isRedAlliance = true;

  NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  NetworkTable fmsTable = ntInst.getTable("FMSInfo");
  NetworkTable drivetrainTable = ntInst.getTable("Pose");
  DoubleArraySubscriber poseSubscriber = drivetrainTable.getDoubleArrayTopic("robotPose").subscribe(new double[] {0.0, 0.0, 0.0});
  BooleanSubscriber allianceSubscriber = fmsTable.getBooleanTopic("IsRedAlliance").subscribe(isRedAlliance);

  /** Creates a new NetworkTablesIO. */
  public NetworkTablesIO() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.networkPose = poseSubscriber.get();
    Translation2d translation = new Translation2d(networkPose[0], networkPose[1]);
    Rotation2d rotation = new Rotation2d(networkPose[2] * (Math.PI / 180));
    this.drivetrainPose = new Pose2d(translation, rotation);
    
    this.isRedAlliance = allianceSubscriber.get();
  }

  public Pose2d getNetworkPose() {
    return this.drivetrainPose;
  }

  public double[] getNetworkPoseArray() {
    return this.networkPose;
  }

  public boolean getAlliance() {
    return this.isRedAlliance;
  }
}
