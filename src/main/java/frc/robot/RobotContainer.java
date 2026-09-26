// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.DrivetrainConst;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopShootingCommand;

import static frc.robot.Controls.joystick;

public class RobotContainer {

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1)
                        .withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


        private final Telemetry logger = new Telemetry(DrivetrainConst.MaxSpeed);

        public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        
        public static SwerveModulePosition frontRight;

        public static SwerveModulePosition backRight;

        public static SwerveModulePosition frontLeft;

        public static SwerveModulePosition backLeft;

        public static final Translation2d m_frontLeftLocation = new Translation2d(TunerConstants.kFrontLeftXPos,
                        TunerConstants.kFrontLeftYPos);

        public static final Translation2d m_frontRightLocation = new Translation2d(TunerConstants.kFrontRightXPos,
                        TunerConstants.kFrontRightYPos);

        public static final Translation2d m_backLeftLocation = new Translation2d(TunerConstants.kBackLeftXPos,
                        TunerConstants.kBackLeftYPos);

        public static final Translation2d m_backRightLocation = new Translation2d(TunerConstants.kBackRightXPos,
                        TunerConstants.kBackRightYPos);

        public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
                        m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        public static Field2d m_field = new Field2d();

        public static double xPose;

        public static double yPose;

        public static final PowerDistribution m_pdh = new PowerDistribution(30, ModuleType.kRev);

        public RobotContainer() {
                configureBindings();
        }

        public void robotPeriodic(){
        }

        private void configureBindings() {
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(Controls.driveRequest()));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));


                drivetrain.registerTelemetry(logger::telemeterize);

                //joystick.povDown().onTrue(drivetrain.applyRequest());

                //INTAKE CONTROL
                /*
                joystick.b().onTrue(m_intakeActuation.setPositionUntilSupply(1.47));
                joystick.x().onTrue(m_intakeActuation.setPositionUntilSupply(0).andThen(new WaitCommand(1)).andThen(m_intakeRollers.stopRollerCommand()));
                joystick.leftBumper().whileTrue(m_intakeRollers.runRollerCommand(90, 100)).whileFalse(m_intakeRollers.stopRollerCommand());
                */
                joystick.b().onTrue(m_intakeActuation.setPositionUntilSupply(1.47));
                joystick.x().onTrue(m_intakeActuation.setPositionUntilSupply(0));
                joystick.leftBumper().whileTrue(m_intakeRollers.runRollerCommand(90, 100)).whileFalse(m_intakeRollers.stopRollerCommand());


                //SHOOTING

        //        joystick.rightBumper().toggleOnTrue(new ShootCommand(()->0, 11, 75).alongWith(m_intakeActuation.setPositionAtSpeedUntilSupply(0.5)))
        //                 .toggleOnFalse(m_angleController.runOnce(() -> m_angleController.setHoodAngleCommand(2)).alongWith(
        //                                 new InstantCommand(() -> m_angleController.setPosition(0)),
        //                                 new InstantCommand(() -> m_shooter.stopMotors()),
        //                                 new InstantCommand(() -> m_ballTunnel.stopAllMotors()),
        //                                 m_shooter.stopShooterCommand()));

                joystick.rightBumper().toggleOnTrue(new ShootCommand(()->0, 11, 75)).toggleOnFalse(new StopShootingCommand());


                // joystick.leftTrigger().whileTrue(m_ballTunnel.spiltCommand(15, 62, 14, 100)
                // .alongWith(m_angleController.runOnce(() -> m_angleController.setHoodAngleCommand(2)))
                // .alongWith(m_intakeRollers.runRollerCommand(85, 100))
                // .alongWith(m_intakeActuation.setPositionAtSpeedUntilSupply(0.5)))
                // .whileFalse(new InstantCommand(() -> m_ballTunnel.stopAllMotorsCommand()));

                joystick.leftTrigger().whileTrue(m_ballTunnel.spiltCommand(15, 62, 14, 100)
                .alongWith(m_angleController.setHoodAngleCommand(2))
                .alongWith(m_intakeRollers.runRollerCommand(85, 100))
                .alongWith(m_intakeActuation.setPositionAtSpeedUntilSupply(0.5)))
                .whileFalse(m_ballTunnel.stopAllMotorsCommand());

                joystick.a().onTrue(m_ballTunnel.runBallTunnelCommand(-62.5, 100).alongWith(m_ballTunnel.runHopperCommand(-62.5, 100))).onFalse(m_ballTunnel.stopAllMotorsCommand());
                joystick.y().onTrue(m_ballTunnel.runBallTunnelCommand(62.5, 100).alongWith( m_ballTunnel.runHopperCommand(62.5, 100))).onFalse(m_ballTunnel.stopAllMotorsCommand());

                // joystick.a().onTrue(new InstantCommand(() -> m_ballTunnel.runBallTunnelCommand(-62.5, 100)).alongWith(new InstantCommand(() -> m_ballTunnel.runHopperCommand(-62.5, 100)))).onFalse(new InstantCommand(() -> m_ballTunnel.stopAllMotorsCommand()));
                // joystick.y().onTrue(new InstantCommand(() -> m_ballTunnel.runBallTunnelCommand(62.5, 100)).alongWith(new InstantCommand(() -> m_ballTunnel.runHopperCommand(62.5, 100)))).onFalse(new InstantCommand(() -> m_ballTunnel.stopAllMotorsCommand()));

                // Reset Heading
                joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).alongWith(new InstantCommand(() -> m_angleController.setHoodAngleCommand(0))));
        }
}