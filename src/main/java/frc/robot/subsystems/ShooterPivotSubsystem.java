package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandcolor.digout.DigoutMode.Disabled;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstant;
import frc.robot.LimelightHelpers;
import frc.robot.PantherUtils;
public class ShooterPivotSubsystem extends SubsystemBase {
    
    private PIDController shooterPID;
    CANSparkMax PivotMotor1 = new CANSparkMax(ShooterConstant.PIVOT_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax PivotMotor2 = new CANSparkMax(ShooterConstant.PIVOT_MOTOR2_PORT, MotorType.kBrushless);
    
    
    boolean isLimelightActive = true;
    static ShooterPivotSubsystem instance;

    static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9);
    public double degreeAim;
    public ShooterPivotSubsystem() {

        PivotMotor1.restoreFactoryDefaults();
        PivotMotor2.restoreFactoryDefaults();
        
        
        PivotMotor1.setIdleMode(IdleMode.kBrake);        
        PivotMotor2.setIdleMode(IdleMode.kBrake);
       
        PivotMotor1.setInverted(true);

        absoluteEncoder.reset();

        shooterPID=new PIDController(3,0,0);
        PIDinitialize(0.965); //0.965 amp 0.96 shooter
        SmartDashboard.putNumber("Shooter SetPoint",getAbsoluteDegree());
        
    }
    private void PIDinitialize(double degree)
    {
        degreeAim = degree;
        shooterPID.reset();
        shooterPID.setSetpoint(degree);
        shooterPID.setTolerance(0.0001);
        
    }
    public void changeDegreeAim(double degree) {
        PIDinitialize(degree);
    }

    public void setPivotMotor(double degree) {
        
        PivotMotor1.set(degree);
        PivotMotor2.set(degree);
    }

    //get degree
    public double getAbsoluteDegree()
    {
        double bizim_encoder=(absoluteEncoder.getAbsolutePosition()<0.5f)? absoluteEncoder.getAbsolutePosition()+1:absoluteEncoder.getAbsolutePosition() ;
        return bizim_encoder;
    }

    public void fixedPos()
    {
        isLimelightActive=false;
        PIDinitialize(0.965);
    }

    public void limelightPos()
    {
        isLimelightActive=true;
        PIDinitialize(DISTANCE_TO_ANGLE_MAP.get(LimelightHelpers.getTY("limelight")));
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Shooter Bore",getAbsoluteDegree());
        
        //double newDegree = SmartDashboard.getNumber("Shooter SetPoint", 1.05);
        //newDegree=PantherUtils.clamp(newDegree,0.95,1.05);
        //if(newDegree != degreeAim)
        //{
            //changeDegreeAim(newDegree);
        //}
        //changeDegreeAim(newDegree);
        setPivotMotor(shooterPID.calculate(getAbsoluteDegree()));
        

        //changeDegreeAim(newDegree);
        //SmartDashboard.putNumber("Limelight_Pose", 0.97-((LimelightHelpers.getTY("limelight")/400)));
        if((LimelightHelpers.getFiducialID("limelight")==4 ||LimelightHelpers.getFiducialID("limelight")==7 || LimelightHelpers.getFiducialID("limelight")==2) && isLimelightActive)
        {
            if(isLimelightActive)
            {
                PIDinitialize(DISTANCE_TO_ANGLE_MAP.get(LimelightHelpers.getTY("limelight")));
            }
            //SmartDashboard.putNumber("LIMLIT", LimelightHelpers.getTY("limelight"));
            //changeDegreeAim(1-((LimelightHelpers.getTY("limelight")/400)));        
        }
    }

    public static ShooterPivotSubsystem getInstance()
    {
        if (instance == null) 
        {
            instance = new ShooterPivotSubsystem();
        }
        return instance;
    }

    
    public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_TO_ANGLE_MAP.put(17.3, 0.965);
        DISTANCE_TO_ANGLE_MAP.put(16.89, 0.965);
        DISTANCE_TO_ANGLE_MAP.put(13.5,0.97);
        DISTANCE_TO_ANGLE_MAP.put(11.30,0.976);
        DISTANCE_TO_ANGLE_MAP.put(9.14,0.98); 
        DISTANCE_TO_ANGLE_MAP.put(7.3,0.984);
        DISTANCE_TO_ANGLE_MAP.put(5.37,0.988);
        DISTANCE_TO_ANGLE_MAP.put(3.34,0.99);
        DISTANCE_TO_ANGLE_MAP.put(1.95,0.995); 
        DISTANCE_TO_ANGLE_MAP.put(0.53,0.997); 
        DISTANCE_TO_ANGLE_MAP.put(-0.84,0.999); 
        DISTANCE_TO_ANGLE_MAP.put(-2.28,1.0); 
        DISTANCE_TO_ANGLE_MAP.put(-3.38,1.005);
        DISTANCE_TO_ANGLE_MAP.put(-4.59,1.005);
        DISTANCE_TO_ANGLE_MAP.put(-5.52,1.008);
        DISTANCE_TO_ANGLE_MAP.put(-6.51,1.01);
        DISTANCE_TO_ANGLE_MAP.put(-7.53,1.013);  
        //DISTANCE_TO_ANGLE_MAP.put(3.0, ArmConstants.kOffset - 0.058);
        //DISTANCE_TO_ANGLE_MAP.put(4.1, ArmConstants.kOffset - 0.038);
        //DISTANCE_TO_ANGLE_MAP.put(4.9, ArmConstants.kOffset - 0.035);
        //DISTANCE_TO_ANGLE_MAP.put(5.5, ArmConstants.kOffset - 0.028);
      }
  
}
