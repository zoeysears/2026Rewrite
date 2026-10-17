package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConsts;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConst;

public class LimelightSubsystem extends SubsystemBase {
  private final String limeLightName;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem(String name) {
    limeLightName = name;
  }

  @Override
  public void periodic() {

  }

  public double limelight_aim_proportional() {
    double kP = .035;
    double targetingAngularVelocity = LimelightHelpers.getTX(limeLightName) * kP * -DrivetrainConst.MaxAngularRate;
    // convert to radians per second for our drive method

    return targetingAngularVelocity;
  }

  public double limelight_range_proportional() {
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY(limeLightName) * kP * -DrivetrainConst.MaxSpeed;
    return targetingForwardSpeed;
  }

  public double getAprilTag() {
    SmartDashboard.putNumber("April Tag Number" + limeLightName, LimelightHelpers.getFiducialID(limeLightName));
    return LimelightHelpers.getFiducialID(VisionConsts.LIMELIGHT_NAME);
  }

  public double getAprilTagHeight() {
    double aprilTag = getAprilTag();
    if (aprilTag == 1 || aprilTag == 6 || aprilTag == 7 || aprilTag == 12) {
      SmartDashboard.putString("Tag Reading", "Trench");
      return VisionConsts.UP_TO_TRENCH_TAG;
    }
    if (aprilTag == 13 || aprilTag == 14) {
      SmartDashboard.putString("Tag Reading", "Corral");
      return VisionConsts.UP_TO_CORRAL_TAG;
    }
    if (aprilTag == 2 || aprilTag == 3 || aprilTag == 4 || aprilTag == 5 || aprilTag == 8 || aprilTag == 9
        || aprilTag == 10 || aprilTag == 11) {
      SmartDashboard.putString("Tag Reading", "Hub");
      return VisionConsts.UP_TO_HUB_TAG;
    }
    return -1;
  }

  public LimelightHelpers.PoseEstimate getMegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
  }

  public LimelightHelpers.PoseEstimate getMegaTag1() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
  }

  public String getLimelightName() {
    return limeLightName;
  }

  public void updateOdometry() {
    boolean useMegaTag2 = true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(getLimelightName());

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        RobotContainer.drivetrain.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(getLimelightName(),
          RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName());

      if (Math.abs(RobotContainer.drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()) > 720) // if
                                                                                                                // our
                                                                                                                // angular
                                                                                                                // velocity
                                                                                                                // is
                                                                                                                // greater
      // than 720 degrees per second, ignore
      // vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // technically just
                                                                                                 // need to set it once
                                                                                                 // if you trust it; can
                                                                                                 // change the dynamic
                                                                                                 // (n1, n2) for
                                                                                                 // confidence, like how
                                                                                                 // far they are, which
                                                                                                 // ones, etc
        RobotContainer.drivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds); // can you feed the std devs
      }
    }
  }

}