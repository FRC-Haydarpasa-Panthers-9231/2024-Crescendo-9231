package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRoller extends Command 
{
    private IntakeRollerSubsystem intakeRollerSubsystem;
    private double speed;
    

    public IntakeRoller(double speed)
    {
        intakeRollerSubsystem=IntakeRollerSubsystem.getInstance();
        this.speed=speed;
        addRequirements(intakeRollerSubsystem);
        
    }

    @Override
    public void initialize()
    {
        intakeRollerSubsystem.setRollerMotor(speed);
    }

    

    @Override
    public void end(boolean interrupted)
    {
        intakeRollerSubsystem.setRollerMotor(0);
    }

    @Override 
    public boolean isFinished()
    {
        return intakeRollerSubsystem.getLimitSwitch();
    }
}