// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  //https://github.com/Woodland4678/CyberCavs2024/blob/main/src/main/java/frc/robot/commands/AutoGrabNote_NoR.java

  @Override
  public void robotInit() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    m_robotContainer = new RobotContainer();
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
  public void teleopPeriodic() {}

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
