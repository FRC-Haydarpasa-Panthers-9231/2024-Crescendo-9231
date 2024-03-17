package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstant;

public class ShooterRollerSubsystem extends SubsystemBase
{
        static ShooterRollerSubsystem instance;

    CANSparkMax RollerMotor1 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax RollerMotor2 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR2_PORT, MotorType.kBrushless);

    private RelativeEncoder FlyWheelEncoder1;
    private RelativeEncoder FlyWheelEncoder2;

    public ShooterRollerSubsystem()
    {
        RollerMotor1.restoreFactoryDefaults();        
        RollerMotor2.restoreFactoryDefaults();
        RollerMotor1.setIdleMode(IdleMode.kBrake);
        RollerMotor2.setIdleMode(IdleMode.kBrake);
        
        RollerMotor2.setInverted(true);
        FlyWheelEncoder1 = RollerMotor1.getEncoder();
        FlyWheelEncoder2 = RollerMotor2.getEncoder();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("FlyWheel1V", FlyWheelEncoder1.getVelocity());
        SmartDashboard.putNumber("FlyWheel2V", FlyWheelEncoder2.getVelocity());
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
