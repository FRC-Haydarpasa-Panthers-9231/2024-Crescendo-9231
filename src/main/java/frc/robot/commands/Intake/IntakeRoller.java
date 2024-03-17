package frc.robot.commands.Intake;

import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRoller extends Command 
{
    private IntakeRollerSubsystem intakeRollerSubsystem;
    private IntakePivotSubsystem intakePivotSubsystem;
    private double speed;
    

    public IntakeRoller(double speed)
    {
        intakeRollerSubsystem=IntakeRollerSubsystem.getInstance();
        intakePivotSubsystem=IntakePivotSubsystem.getInstance();
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
        //intakePivotSubsystem.pivotSet(Rotation2d.fromDegrees(10));
    }

    @Override 
    public boolean isFinished()
    {
        return intakeRollerSubsystem.getLimitSwitch();
    }
}