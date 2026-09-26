package frc.robot.commands;

import static frc.robot.Subsystems.m_angleController;
import static frc.robot.Subsystems.m_ballTunnel;
import static frc.robot.Subsystems.m_shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopShootingCommand extends ParallelCommandGroup{
    public StopShootingCommand() {

        addCommands(
            m_angleController.setHoodAngleCommand(2.0),
            m_shooter.stopShooterCommand(),
            m_ballTunnel.stopAllMotorsCommand()
        );
    }
}
