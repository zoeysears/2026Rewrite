package frc.robot.subsystems;

import java.util.logging.LogManager;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeActuationConsts;
import frc.robot.generated.TunerConstants;

public class IntakeAcutation extends SubsystemBase {

    private TalonFX actuationMotor;
    private MotionMagicVoltage motionMagicVoltage;
    private NeutralOut stopMode;

    public IntakeAcutation(int motorId) {
        actuationMotor = new TalonFX(motorId, TunerConstants.kCANBus);

        motionMagicVoltage = new MotionMagicVoltage(0).withEnableFOC(true);
        stopMode = new NeutralOut();
        setMotorConfigs();

    }

    private void setMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.Feedback.SensorToMechanismRatio = IntakeActuationConsts.GearRatio;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 20;

        configs.MotionMagic.MotionMagicCruiseVelocity = 30;
        configs.MotionMagic.MotionMagicAcceleration = 50;
        configs.MotionMagic.MotionMagicJerk = 75;

        configs.CurrentLimits.StatorCurrentLimit = 120;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        configs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        configs.Slot0.kP = 30; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 1.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.1; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = actuationMotor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

    }

    public Command setPositionUntilSupply(double position) {
        return run(() -> setPosition(position)).until(() -> isStalled()).finallyDo(() -> stopMotor());
    }

    public Command setPositionAtSpeedUntilSupply(double speed) {
        return run(() -> setPositionSlowly(speed)).until(() -> isStalled());
    }

    private void setPositionSlowly(double speed) {
        DynamicMotionMagicDutyCycle request = new DynamicMotionMagicDutyCycle(getPosition(), speed, .1);
        actuationMotor.setControl(request.withPosition(0));
    }

    public Command stopMotorCommand() {
        return new InstantCommand(() -> this.stopMotor());
    }

    private void setPosition(double position) {
        actuationMotor.setControl(motionMagicVoltage.withPosition(position));
    }

    private void stopMotor() {
        //actuationMotor.setControl(stopMode);
        actuationMotor.stopMotor();
        actuationMotor.setNeutralMode(NeutralModeValue.Coast); //see if this will do both stop and set to coast
    }

    protected double getPosition() {
        return this.actuationMotor.getPosition().getValueAsDouble();
    }

    protected double getVelocity() {
        return this.actuationMotor.getVelocity().getValueAsDouble(); //rot/sec
    }

    protected double getStatorCurrent() {
        return this.actuationMotor.getStatorCurrent().getValueAsDouble();
    }

    protected boolean isStalled() {
        double maxStalledCurrentThersold = 100.0;
        boolean isStalled = (Math.abs(getStatorCurrent()) > maxStalledCurrentThersold);
        DogLog.log("isStalled", isStalled);
        DogLog.log("statorCurrent", getStatorCurrent());
        return (Math.abs(getStatorCurrent()) > maxStalledCurrentThersold);
    }

    public void setCoastMode() {
        actuationMotor.setNeutralMode(NeutralModeValue.Coast); //see if this will do both stop and set to coast
    }
}
