package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterPivot;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterRollerSubsystem;

public class AutoFeed extends Command
{
    ShooterPivotSubsystem m_shooterPivot;
    ShooterRollerSubsystem m_shooterRoller;
    IntakeRollerSubsystem m_intakeRoller;
    IntakePivotSubsystem m_intakePivot;

    public AutoFeed()
    {
        m_shooterPivot=ShooterPivotSubsystem.getInstance();
        m_shooterRoller=ShooterRollerSubsystem.getInstance();
        m_intakePivot=IntakePivotSubsystem.getInstance();
        m_intakeRoller=IntakeRollerSubsystem.getInstance();
        addRequirements(m_intakePivot);
        addRequirements(m_shooterPivot);
        addRequirements(m_intakePivot);
        addRequirements(m_shooterPivot);
    }
}
