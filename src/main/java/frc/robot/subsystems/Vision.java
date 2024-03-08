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


    public double estimateDistance(){

        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight");
        double limelightMountAngleDegrees = 46.0; 
        double limelightLensHeightInches = 5.611811023622; 
        double goalHeightInches = 80.708661417323; 
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);



        return distanceFromLimelightToGoalInches;
    }

    @Override 
    public void periodic()
    {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        SmartDashboard.getNumber("distance", estimateDistance());
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
