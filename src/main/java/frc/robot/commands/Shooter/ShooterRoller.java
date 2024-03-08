package frc.robot.commands.Shooter;

import frc.robot.subsystems.ShooterRollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRoller extends Command
{
    ShooterRollerSubsystem shooterRollerSubsystem;

    double speed;
    public ShooterRoller(double speed)
    {
        this.speed=speed;
        shooterRollerSubsystem= ShooterRollerSubsystem.getInstance();
        addRequirements(shooterRollerSubsystem);
    }
    
    @Override
    public void initialize()
    {
        shooterRollerSubsystem.setRollerMotor(speed);
    }
    
    
    
}
