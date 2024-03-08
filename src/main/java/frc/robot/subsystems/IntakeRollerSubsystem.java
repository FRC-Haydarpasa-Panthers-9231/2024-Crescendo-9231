package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeRollerSubsystem extends SubsystemBase 
{    
    CANSparkMax rollerMotor;


    private DigitalInput limitSwitch=new DigitalInput(1);


    static IntakeRollerSubsystem instance;


    public IntakeRollerSubsystem() 
    {
        rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
        



        motorConfig();
    }

    public void motorConfig()
    {
        rollerMotor.restoreFactoryDefaults();

        rollerMotor.setIdleMode(IdleMode.kBrake);

    }

    public void setRollerMotor(double forward) {
        
        rollerMotor.set(forward);
    }

    

    public boolean getLimitSwitch()
    {
        return limitSwitch.get();
    }


    @Override
    public void periodic()
    {
        
        SmartDashboard.putBoolean("LimitW", getLimitSwitch());
    }

    

    public static IntakeRollerSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new IntakeRollerSubsystem();
        }
        return instance;
    }
}
