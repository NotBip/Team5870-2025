// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final SendableChooser<String> pidTuner; 
  private static Robot instance;
  double tunePID_KP; 
  double tunePID_KI; 
  double tunePID_KD; 


  

  //https://github.com/Woodland4678/CyberCavs2024/blob/main/src/main/java/frc/robot/commands/AutoGrabNote_NoR.java

  public Robot() { 

    pidTuner = new SendableChooser<String>();
    instance = this; 
    pidTuner.addOption("Arm PID", "Arm PID");
    pidTuner.addOption("Elevator PID", "Elevator PID");
    pidTuner.addOption("Intake PID", "Intake PID");
    pidTuner.setDefaultOption("Null", "Null");
  }


  @Override
  public void robotInit() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    m_robotContainer = new RobotContainer();

    // tunePID_KP = 0; 
    // tunePID_KI = 0; 
    // tunePID_KD = 0; 

    // SmartDashboard.putNumber("Tune PID P Gain", tunePID_KP); 
    // SmartDashboard.putNumber("Tune PID I Gain", tunePID_KI); 
    // SmartDashboard.putNumber("Tune PID D Gain", tunePID_KD); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  //   SmartDashboard.putData("PID tuning Choices", pidTuner);

  //   double tunePID_Dashboard_P = SmartDashboard.getNumber("Tune PID P Gain", 0); 
  //   double tunePID_Dashboard_I = SmartDashboard.getNumber("Tune PID I Gain", 0); 
  //   double tunePID_Dashboard_D = SmartDashboard.getNumber("Tune PID D Gain", 0); 

  //  if (tunePID_Dashboard_P != tunePID_KP || tunePID_Dashboard_I != tunePID_KI || tunePID_Dashboard_D != tunePID_KD) {
  //     if (pidTuner.getSelected().equals("Arm PID")){
  //        m_robotContainer.updateArmPID(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D); 
  //     }
  //     else if (pidTuner.getSelected().equals("Elevator PID")){
  //        m_robotContainer.updateElevatorPID(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D);
  //     }
  //     // else if (pidTuner.getSelected().equals("Intake PID")){
  //     //    m_robotContainer.updateIntakePID(tunePID_Dashboard_P, tunePID_Dashboard_I, tunePID_Dashboard_D); 
  //     // }
  //   }

  //     tunePID_KP = tunePID_Dashboard_P;
  //     tunePID_KI = tunePID_Dashboard_I;
  //     tunePID_KD = tunePID_Dashboard_D;

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    m_autonomousCommand = m_robotContainer.selfTestCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void testPeriodic() {


  }

  @Override
  public void testExit() {}
}
