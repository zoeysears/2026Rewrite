package frc.robot.commands;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_intakeActuation;
import static frc.robot.Subsystems.m_intakeRollers;
import static frc.robot.Subsystems.m_shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootCommand extends ParallelCommandGroup {
    public ShootCommand(DoubleSupplier shooterSpeed, double shooterAngle, double ballTunnelSpeed) {

        addCommands(
                m_angleController.setHoodAngleCommand(shooterAngle),
                m_shooter.runShooterCommand(100),
                new SequentialCommandGroup(
                        m_angleController.waitUntilAtPosition(shooterAngle),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        m_ballTunnel.runBallTunnelCommand(ballTunnelSpeed, 30),
                                        m_ballTunnel.runHopperCommand(-ballTunnelSpeed, 20),
                                        new WaitCommand(0.3),
                                        m_ballTunnel.runHopperCommand(ballTunnelSpeed, 20))),
                                Commands.waitSeconds(2),
                                m_intakeRollers.runRollerCommand(85, 100),
                                m_intakeActuation.setPositionAtSpeedUntilSupply(0.45)));
                                

    }
}