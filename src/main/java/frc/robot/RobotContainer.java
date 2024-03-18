package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterRoller;
import frc.robot.commands.sequence.IntakeTakeSequence;
import frc.robot.commands.sequence.IntakeFeedShotSequence;

import frc.robot.commands.sequence.ShootSequence;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.DriverControlsSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterRollerSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  XboxController driverXbox = new XboxController(0);

  private final SwerveSubsystem driveSubsystem ;
  private final ShooterRollerSubsystem m_shooterRoller;
  private final ShooterPivotSubsystem shooterPivotSubsystem;


  private final IntakeRollerSubsystem m_intakeRoller;
  private final IntakePivotSubsystem m_intakePivot;

  private final VisionSubsystem m_vision;

  private final SendableChooser<Command> autoChooser;
  private  final DriverControlsSubsystem driverControlsSubsystem;

  private UsbCamera cam;
  private UsbCamera cam2;
  
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() 
  {

    

    driveSubsystem= new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
    shooterPivotSubsystem= ShooterPivotSubsystem.getInstance();
    m_shooterRoller=ShooterRollerSubsystem.getInstance();

    m_intakeRoller=IntakeRollerSubsystem.getInstance();
    m_intakePivot=IntakePivotSubsystem.getInstance();

    driverControlsSubsystem = DriverControlsSubsystem.getInstance();
    
    m_vision = VisionSubsystem.getInstance();
    
    driverControlsSubsystem.registerTriggers();

    //intake roller

    NamedCommands.registerCommand("intakeRollerIn", new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER));
    NamedCommands.registerCommand("intakeRollerOut", new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)));
    NamedCommands.registerCommand("intakeRollerStop", new InstantCommand(()->m_intakeRoller.setRollerMotor(0)));

    //intake position
    NamedCommands.registerCommand("intakeFeed", new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(2))));
    NamedCommands.registerCommand("intakeFeedTop", new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(10))));
    NamedCommands.registerCommand("intakeTop", new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(90))));
    NamedCommands.registerCommand("intakeBottom", new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(210))));

    // shooter roller

    NamedCommands.registerCommand("shooterRoller", new ShooterRoller(Constants.ShooterConstant.SPEAKER_POWER));
    NamedCommands.registerCommand("shooterRollerStop", new InstantCommand(()->m_shooterRoller.setRollerMotor(0)));

// sequence
    NamedCommands.registerCommand("shootSequence", new ShootSequence());
    NamedCommands.registerCommand("intakeTakeSequence", new IntakeTakeSequence());
    NamedCommands.registerCommand("intakeFeedShotSequence", new IntakeFeedShotSequence());


    autoChooser = AutoBuilder.buildAutoChooser("Enes");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    //  Command taxi= driveSubsystem.driveCommand2(
    //  ()->0.2,
    //  ()->0,
    //  ()->0,
    //   true);

    //   return taxi.withTimeout(2);
   return autoChooser.getSelected();
   //return null;
  }
}
