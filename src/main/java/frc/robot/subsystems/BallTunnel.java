package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallTunnelConsts;

public class BallTunnel extends SubsystemBase {
    private TalonFX ballTunnelMotor;
    private TalonFX diverterMotor;
    private TalonFX hopperMotor;

    private VelocityVoltage velocityVoltage;
    private NeutralOut neutralOut;

    public BallTunnel(int ballTunnelId, int diverterId, int hopperFloorId) {
        ballTunnelMotor = new TalonFX(ballTunnelId);
        diverterMotor = new TalonFX(diverterId);
        hopperMotor = new TalonFX(hopperFloorId);

        velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);
        neutralOut = new NeutralOut();

        initBallTunnelMotorConfigs();
        initHopperMotorConfigs();
        initDiverterConfigs();
    }

    private void initBallTunnelMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.Feedback.SensorToMechanismRatio = BallTunnelConsts.MainGearRatio;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 30;

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 1;// 0.0000001; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.1; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.10; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = ballTunnelMotor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

    }

    private void initHopperMotorConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.Feedback.SensorToMechanismRatio = BallTunnelConsts.DivergentGearRatio;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 30;

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 0.1; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.10; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = diverterMotor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

    }

    private void initDiverterConfigs() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.Feedback.SensorToMechanismRatio = BallTunnelConsts.DivergentGearRatio;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 30;

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 0.1; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.10; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hopperMotor.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    // run ball tunnel command
    public Command runBallTunnelCommand(double velocity, double acceleration) {
        return run(() -> runBallTunnel(velocity, acceleration));
    }

    private void runBallTunnel(double velocity, double acceleration) {
        ballTunnelMotor.setControl(velocityVoltage.withVelocity(velocity).withAcceleration(acceleration));
        diverterMotor.setControl(velocityVoltage.withAcceleration(velocity).withAcceleration(acceleration));
    }

    private void runHopper(double velocity, double acceleration) {
        hopperMotor.setControl(velocityVoltage.withAcceleration(acceleration).withVelocity(velocity));
    }

    public Command runHopperCommand(double velocity, double acceleration) {
        return run(() -> runHopper(velocity, acceleration));
    }

    public Command runHopperAndBallTunnelCommand(double velocity, double acceleration) {
        return run(() -> {
            runHopper(velocity, acceleration);
            runBallTunnel(velocity, acceleration);
        });
    }

    public Command spiltCommand(double ballTunnelVelocity, double diverterVelocity, double hopperVelocity, double acceleration) {
        return run(() -> spilt(ballTunnelVelocity, diverterVelocity, hopperVelocity, acceleration));
    }

    private void spilt(double ballTunnelVelocity, double diverterVelocity, double hopperVelocity, double acceleration) {
        ballTunnelMotor.setControl(velocityVoltage
                .withVelocity(ballTunnelVelocity * 10)
                .withAcceleration(acceleration));
        diverterMotor.setControl(velocityVoltage
                .withVelocity(-10 * diverterVelocity)
                .withAcceleration(acceleration));
        hopperMotor.setControl(velocityVoltage
                .withVelocity(hopperVelocity)
                .withAcceleration(acceleration));
    }

    public Command stopAllMotorsCommand() {
        return run(() -> stopAllMotors());
    }

    private void stopAllMotors() {
        ballTunnelMotor.stopMotor();
        diverterMotor.stopMotor();
        hopperMotor.stopMotor();
    }

    // check at speed
    public boolean atSpeed(double speed, double tolerance) {
        return Math.abs(diverterMotor.getVelocity().getValueAsDouble()-speed) < tolerance;
    }


}
