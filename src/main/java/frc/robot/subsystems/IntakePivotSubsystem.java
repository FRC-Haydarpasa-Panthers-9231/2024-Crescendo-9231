package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakePivotSubsystem extends SubsystemBase 
{    
    private static double kS = 0.00;
    private static double kG = 0.2;
    private static double kV = 0.0; 

    CANSparkMax pivotMotor;
    
    SparkPIDController pivotController;

    private RelativeEncoder boreEncoder;

    

    private ArmFeedforward pivotFeedForward;

    static IntakePivotSubsystem instance;

    static double setPoint;

    public IntakePivotSubsystem() 
    {
        pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
        
        pivotController= pivotMotor.getPIDController();

        pivotFeedForward = new ArmFeedforward(kS, kG, kV);

        boreEncoder = pivotMotor.getAlternateEncoder(Type.kQuadrature,8192);

        SmartDashboard.putNumber("intakeDegree",getAbsoluteEncoder());
        motorConfig();
    }

    public void motorConfig()
    {
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setIdleMode(IdleMode.kBrake);


        pivotController.setP(Constants.IntakeConstants.INTAKE_P);
        pivotController.setI(Constants.IntakeConstants.INTAKE_I);
        pivotController.setD(Constants.IntakeConstants.INTAKE_D);
        pivotController.setIZone(0);
        pivotController.setIMaxAccum(0,0);
        

        pivotController.setFeedbackDevice(boreEncoder);

        boreEncoder.setPositionConversionFactor(180);
        pivotMotor.setInverted(true);
        //pivotSet(Rotation2d.fromDegrees(0));
    }

    public void pivotSet(Rotation2d angle)
    {
        pivotController.setReference(
         angle.getDegrees(),
         CANSparkMax.ControlType.kPosition,
         0,
         pivotFeedForward.calculate(angle.getRadians(),0.00),
         ArbFFUnits.kVoltage);
         
        setPoint=angle.getDegrees();
    }

 

    public void setPivotMotor(double forward) {
        
        pivotMotor.set(forward);
    }


    public void resetAbsoluteEncoder()
    {
    
        SmartDashboard.putBoolean("resetlendi", true);
    }

    public double getAbsoluteEncoder()
    {
        return boreEncoder.getPosition();
    }

    @Override
    public void periodic()
    {
        
        SmartDashboard.putNumber("Intake Bore", getAbsoluteEncoder());
        SmartDashboard.putNumber("Setpoint", setPoint);
    }

    

    public static IntakePivotSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new IntakePivotSubsystem();
        }
        return instance;
    }
}
