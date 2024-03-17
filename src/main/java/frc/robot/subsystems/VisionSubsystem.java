package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase 
{
    static VisionSubsystem instance;

    SwerveSubsystem m_drive;
    
    public VisionSubsystem()
    {
        m_drive=SwerveSubsystem.getInstance();
    }



    @Override 
    public void periodic()
    {
        double error=LimelightHelpers.getTY("limelight");
        SmartDashboard.putNumber("LimelightTx", error);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(limelightMeasurement.tagCount >= 2)
        {
            m_drive.addVisionReading(limelightMeasurement.pose,limelightMeasurement.timestampSeconds);
        }
    }

    //hedefin ekrandaki x konumunu vericek
    public double getXPos() {
        return 0;
    }

    public static VisionSubsystem getInstance()
    {
        if(instance == null)
        {
            instance= new VisionSubsystem();
        }
        return instance;
    }
}
