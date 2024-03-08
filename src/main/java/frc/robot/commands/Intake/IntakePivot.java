package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakePivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakePivot extends Command 
{
    private IntakePivotSubsystem intakePivotSubsystem;
    private double speed;

    public IntakePivot(double speed)
    {
        intakePivotSubsystem=IntakePivotSubsystem.getInstance();
        this.speed=speed;
        addRequirements(intakePivotSubsystem);
    }

    @Override
    public void initialize()
    {
        intakePivotSubsystem.setPivotMotor(speed);
    }
    

}