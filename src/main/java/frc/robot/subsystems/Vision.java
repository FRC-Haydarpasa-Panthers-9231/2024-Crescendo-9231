package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase 
{
    static Vision instance;

    SwerveSubsystem m_drive;
    
    public Vision()
    {
        m_drive=SwerveSubsystem.getInstance();
    }



    @Override 
    public void periodic()
    {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(LimelightHelpers.getTV("limelight"))
        {
            m_drive.addVisionReading(limelightMeasurement.pose,limelightMeasurement.timestampSeconds);
        }
    }

    //hedefin ekrandaki x konumunu vericek
    public double getXPos() {
        return 0;
    }

    public Vision getInstance()
    {
        if(instance == null)
        {
            instance= new Vision();
        }
        return instance;
    }
}
