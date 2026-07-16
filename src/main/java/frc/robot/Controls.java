package frc.robot;

import java.lang.reflect.Field;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.DrivetrainConst;
import frc.robot.Constants.FieldConst;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
       
        public static final CommandXboxController joystick = new CommandXboxController(0);


        public static final boolean isRightStickDrive = false;

        private static final SwerveRequest.FieldCentric driveF = new SwerveRequest.FieldCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1)
                        .withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private static final SwerveRequest.RobotCentric driveR = new SwerveRequest.RobotCentric()
                        .withDeadband(DrivetrainConst.MaxSpeed * 0.1)
                        .withRotationalDeadband(DrivetrainConst.MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        public static Supplier<SwerveRequest> driveRequest() {

                return () -> driveF
                                .withVelocityX(
                                                -(isRightStickDrive ? joystick.getRightY() : joystick.getLeftY())
                                                                * DrivetrainConst.MaxSpeed)
                                .withVelocityY(
                                                -(isRightStickDrive ? joystick.getRightX() : joystick.getLeftX())
                                                                * DrivetrainConst.MaxSpeed)
                                .withRotationalRate(
                                                -(isRightStickDrive ? joystick.getLeftX() : joystick.getRightX())
                                                                * DrivetrainConst.MaxAngularRate);
        }



}