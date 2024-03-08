package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstant;

public class ShooterRollerSubsystem extends SubsystemBase
{
        static ShooterRollerSubsystem instance;

    CANSparkMax RollerMotor1 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax RollerMotor2 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR2_PORT, MotorType.kBrushless);

    public ShooterRollerSubsystem()
    {
        RollerMotor1.restoreFactoryDefaults();        
        RollerMotor2.restoreFactoryDefaults();
        RollerMotor1.setIdleMode(IdleMode.kBrake);
        RollerMotor2.setIdleMode(IdleMode.kBrake);
        
        RollerMotor2.setInverted(true);
    }

    public void setRollerMotor(double forward) 
    {
        
        RollerMotor1.setVoltage(forward);
        RollerMotor2.setVoltage(forward);
    }

      public static ShooterRollerSubsystem getInstance()
    {
        if (instance == null) 
        {
            instance = new ShooterRollerSubsystem();
        }
        return instance;
    }

}
