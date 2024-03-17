
package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  SwerveDrive swerveDrive;
  private static SwerveSubsystem INSTANCE = null;


  
  public SwerveSubsystem(File directory) 
  {
    SwerveDriveTelemetry.verbosity=TelemetryVerbosity.LOW;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MaxModuleSpeed);
    } 
    catch (Exception e)
    {
      throw new RuntimeException(e);
    }  
    INSTANCE=this;
    setupPathPlanner();

  }
  


  public void setupPathPlanner()
  {
  AutoBuilder.configureHolonomic(
              this::getPose,
              this::resetOdometry,
              this::getRobotOrientedVelocity,
              this::setChassisSpeed,
              new HolonomicPathFollowerConfig(
                  new PIDConstants(Constants.PIDConstants.kSwerveAutoPIDP, Constants.PIDConstants.kSwerveAutoPIDI, Constants.PIDConstants.kSwerveAutoPIDD),
                  new PIDConstants(
                      swerveDrive.swerveController.config.headingPIDF.p,
                      swerveDrive.swerveController.config.headingPIDF.i,
                      swerveDrive.swerveController.config.headingPIDF.d),
                  Constants.MaxModuleSpeed,
                  swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                  new ReplanningConfig()
              ),
              this::shouldPathFlip,
              this
          );  
  }

  public Rotation2d getHeading()
  {
    return swerveDrive.getYaw();
  }


//MAVİ TARAFTA ÇALIŞIYOR
  public Command driveCommand2(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX,BooleanSupplier fieldOrianted)
  {
    return run(() -> {
      // Make the robot move
      DriverStation.Alliance color;
	    color = DriverStation.getAlliance().get();
      int isBlueAlliance= ((DriverStation.Alliance.Red==color)&& fieldOrianted.getAsBoolean()==true )? -1:1;
      //SmartDashboard.putNumber("Joystick", angularRotationX.getAsDouble());
      swerveDrive.drive(new Translation2d(-translationX.getAsDouble() * swerveDrive.getMaximumVelocity()*isBlueAlliance,
                                          -translationY.getAsDouble() * swerveDrive.getMaximumVelocity()*isBlueAlliance),
                        -angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        fieldOrianted.getAsBoolean(),
                        false);
    });
  }


  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }


  @Override
  public void periodic() 
  {
    
  }

  @Override
  public void simulationPeriodic() {

  }

  public static SwerveSubsystem getInstance() 
  {
    if (INSTANCE == null)
    {
      INSTANCE = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return INSTANCE;
  }

  public SwerveController getSwerveController()
  {
        return swerveDrive.swerveController;
  }


  public void resetOdometry(Pose2d pose)
  {
      swerveDrive.resetOdometry(pose);
  }

  public Pose2d getPose()
  {
      return swerveDrive.getPose();
  }

  public void setChassisSpeed(ChassisSpeeds velocity)
  {
      swerveDrive.setChassisSpeeds(velocity);
  }

  public ChassisSpeeds getRobotOrientedVelocity()
  {
      return swerveDrive.getRobotVelocity();
  }
  
  public boolean shouldPathFlip()
  {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void addVisionReading(Pose2d robotPose2d,double time)
  {
    swerveDrive.addVisionMeasurement(robotPose2d,time);
  }

}
