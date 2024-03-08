package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivotSubsystem;

public class ShooterPivot extends Command
{
    ShooterPivotSubsystem m_shooterPivot;
    double speed;

    public ShooterPivot(double speed)
    {
        m_shooterPivot=ShooterPivotSubsystem.getInstance();
        this.speed=speed;
        addRequirements(m_shooterPivot);
    }

    @Override
    public void initialize()
    {
        m_shooterPivot.setPivotMotor(speed);
    }

    
}
