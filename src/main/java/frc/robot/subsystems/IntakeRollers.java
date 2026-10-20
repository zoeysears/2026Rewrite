package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRollersConsts;

public class IntakeRollers extends SubsystemBase {

    private TalonFX rollerMotor;
    private VelocityVoltage  velocityVoltage;


    public IntakeRollers(int motorId) {
        rollerMotor = new TalonFX(motorId);
        velocityVoltage = new VelocityVoltage(0).withEnableFOC(true);

        initalizeRollerConfig();
    }

    private void initalizeRollerConfig() {
         TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configs.Feedback.SensorToMechanismRatio = IntakeRollersConsts.GearRatio;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 30;

        /*
         * Voltage-based velocity requires a feed forward to account for the back-emf of
         * the motor
         */
        configs.Slot0.kP = 0.7; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.0; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.1; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                                 // volts / Rotation per second

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = rollerMotor.getConfigurator().apply(configs);
            if (status.isOK())

                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }
    

    public Command runRollerCommand(double speed, double acceleration) {
       return run(() -> runIntakeRoller(speed, acceleration));
    }

    private void runIntakeRoller(double speed, double acceleration) {
        rollerMotor.setControl(velocityVoltage.withVelocity(speed).withAcceleration(acceleration));
    }


    public Command stopRollerCommand() {
        return new InstantCommand(() -> stopRollers());
    }

    private void stopRollers() {
        rollerMotor.stopMotor();
    }
}
