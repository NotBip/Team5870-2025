package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;

/**
 * Basic simulation of a swerve subsystem with the methods needed by PathPlanner
 */
public class SwerveSim extends SubsystemBase {
  private SimSwerveModule[] modules;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  public SimGyro gyro;
   private double yaw = 0; 

  
  public SwerveSim() {
    gyro = new SimGyro();
    modules = new SimSwerveModule[]{
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule(),
      new SimSwerveModule()
    };

    kinematics = DriveConstants.kDriveKinematics; 
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
    // Set up custom logging to add the current path to a field 2d widget
    // PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    // field.getRobotObject().setPose(PathPlannerAuto.getStaringPoseFromAutoFile("New Auto"));
  }

  @Override
  public void periodic() {
    // Update the simulated gyro, not needed in a real project
    gyro.updateRotation(getSpeeds().omegaRadiansPerSecond);
    odometry.update(gyro.getRotation2d(), getPositions());
    Logger.recordOutput("RPose2", odometry.getPoseMeters());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
  }

  public void updatePose3d(boolean reversed, double move) { 
    if(yaw <= 0 && reversed)
      return;

    if(reversed)
      yaw -= move;  
    else 
      yaw += move; 
  }

  public Pose3d getPose3d() { 
    return new Pose3d(
      odometry.getPoseMeters().getX(),
      odometry.getPoseMeters().getY(), 
      0, 
      new Rotation3d(0,0,0)); 
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, 4.5);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setTargetState(targetStates[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public void resetGyro() { 
    gyro.resetGyro();
  }

  /**
   * Basic simulation of a swerve module, will just hold its current state and not use any hardware
   */
  class SimSwerveModule {
    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    public SwerveModulePosition getPosition() {
      return currentPosition;
    }

    public SwerveModuleState getState() {
      return currentState;
    }

    public void setTargetState(SwerveModuleState targetState) {
      // Optimize the state
      currentState = SwerveModuleState.optimize(targetState, currentState.angle);

      currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
    }
  }

  /**
   * Basic simulation of a gyro, will just hold its current state and not use any hardware
   */
  class SimGyro {
    private Rotation2d currentRotation = new Rotation2d();

    public Rotation2d getRotation2d() {
      return currentRotation;
    }

    public void resetGyro() { 
      currentRotation = new Rotation2d(0); 
    }

    public void updateRotation(double angularVelRps){
      currentRotation = currentRotation.plus(new Rotation2d(angularVelRps * 0.02));
    }
  }
}