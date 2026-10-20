package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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


  public LimelightHelpers.PoseEstimate getMegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
  }

  public LimelightHelpers.PoseEstimate getMegaTag1() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
  }

  public String getLimelightName() {
    return limeLightName;
  }

  /*public void updateOdometry() {
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
  }*/

  public void updateOdometry() {
    final String name = getLimelightName();

    double yawDegrees = RobotContainer.drivetrain.getPigeon2().getYaw().getValueAsDouble();
    LimelightHelpers.SetRobotOrientation(name, yawDegrees, 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    boolean doRejectUpdate = false;

    if (mt2 == null || mt2.tagCount == 0) {
        doRejectUpdate = true;
    }

    double angularVel = Math.abs(
            RobotContainer.drivetrain.getPigeon2()
                    .getAngularVelocityZDevice()
                    .getValueAsDouble());
    if (angularVel > 720.0) {
        doRejectUpdate = true;
    }

    // Reject if the vision pose is wildly different from current odometry
    // (protects against sudden jumps / flipped solutions)
    if (!doRejectUpdate) {
        Pose2d currentPose = RobotContainer.drivetrain.getState().Pose;
        double dx = mt2.pose.getX() - currentPose.getX();
        double dy = mt2.pose.getY() - currentPose.getY();
        double distanceJump = Math.hypot(dx, dy);

        if (distanceJump > 1.5) { // meters – tune this
            doRejectUpdate = true;
        }
    }

    // Optional extra filters when you have the raw data
    if (!doRejectUpdate && mt2.avgTagDist > 4.5) {          // too far
        doRejectUpdate = true;
    }
    if (!doRejectUpdate && mt2.avgTagArea < 0.05) {         // tags too small / noisy
        doRejectUpdate = true;
    }

    // 3. Accept the measurement with dynamic stddevs
    if (!doRejectUpdate) {
        // Tighter when we see multiple tags or are close; looser otherwise
        double xyStdDev;
        if (mt2.tagCount >= 2) {
            xyStdDev = 0.4;                 // multi-tag → higher trust
        } else if (mt2.avgTagDist < 2.0) {
            xyStdDev = 0.6;                 // single close tag
        } else {
            xyStdDev = 1.0;                 // single far tag → lower trust
        }

        // We trust the gyro much more than vision for rotation
        RobotContainer.drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

        RobotContainer.drivetrain.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
    }
}

}