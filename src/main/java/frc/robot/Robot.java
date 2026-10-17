// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_limelight;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Robot extends TimedRobot {

  private final RobotContainer m_robotContainer;


  public Robot() {  


    m_robotContainer = new RobotContainer();

    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.robotPeriodic();
    m_limelight.updateOdometry();
    
  }

  @Override
  public void disabledInit() {
    m_intakeActuation.setCoastMode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }


}
