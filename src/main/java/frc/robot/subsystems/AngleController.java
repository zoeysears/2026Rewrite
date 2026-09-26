package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleControllerConsts;
import frc.robot.generated.TunerConstants;

public class AngleController extends SubsystemBase {
    private TalonFX hoodMotor;
    MotionMagicVoltage motionMagicVoltage;
    NeutralOut stopMode; 
    
    public AngleController(int leftMotorId) {
        hoodMotor = new TalonFX(leftMotorId, TunerConstants.kCANBus);

        motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);
        stopMode = new NeutralOut();
        initHoodConfigs();
    }

    private void initHoodConfigs() {
      TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.Feedback.SensorToMechanismRatio = AngleControllerConsts.GEAR_RATIO;
    // configs.MotorOutput.DutyCycleNeutralDeadband = 0.001;

    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 40;

    configs.MotionMagic.MotionMagicCruiseVelocity = 50; // 7,530 / 60 = 125.5 rps
    configs.MotionMagic.MotionMagicAcceleration = 20;
    configs.MotionMagic.MotionMagicJerk = 50;
    // 27 degrees start aiming for 37
    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // 320 barely under (less than 1 degree)
    configs.Slot0.kP = 320; // 25 // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0; // 4 // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.2; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.93; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second

    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hoodMotor.getConfigurator().apply(configs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public Command setHoodAngleCommand(double position) {
      return run(() -> setHoodAngle(position));
    }

    private void setHoodAngle(double rotation) {
      hoodMotor.setControl(motionMagicVoltage.withPosition(rotation/360));
    }

    public Command waitUntilAtPosition(double desiredPosition) {
      return Commands.run(() -> setHoodAngle(desiredPosition)).until(() -> atTarget(desiredPosition));
    }

    public double getAnglePosition() {
      return hoodMotor.getPosition().getValueAsDouble() / AngleControllerConsts.GEAR_RATIO;
    }

    public boolean atTarget(double desiredPosition) {
      return Math.abs(desiredPosition - getAnglePosition()) > 5;
    }

    public void stopMotor() {
      hoodMotor.stopMotor();
    }
}