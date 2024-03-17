package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterRoller;
import frc.robot.commands.sequence.IntakeTakeSequence;


public class DriverControlsSubsystem extends SubsystemBase{
    
    static DriverControlsSubsystem instance;
    private ClimberSubsystem m_climber;
    private IntakePivotSubsystem m_intakePivot;
    private IntakeRollerSubsystem m_intakeRoller;
    private SwerveSubsystem swerveSubsystem;
    private XboxController driverController;
    private PS4Controller operatorController;
    private XboxController practiceController;
    private double speedRate;

    public DriverControlsSubsystem(){
        // kollar
        driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
        operatorController = new PS4Controller(Constants.OperatorConstants.kOperatorControllerPort);
        practiceController = new XboxController(Constants.OperatorConstants.kPracticeControllerPort);

    
        m_climber=ClimberSubsystem.getInstance();
        m_intakeRoller=IntakeRollerSubsystem.getInstance();
        m_intakePivot=IntakePivotSubsystem.getInstance();
        swerveSubsystem=SwerveSubsystem.getInstance();
        speedRate =1;
    }

    public XboxController getDriverController(){
        return driverController;
    }

    public PS4Controller getOperatorController()
    {
        return operatorController;
    }
     public XboxController getPractiveController()
     {
        return practiceController;
     }


    /*Intake roller */
    public boolean IntakeRollerIn(){
        return driverController.getRightTriggerAxis()>0;
    }
    
    public boolean IntakeRollerOut(){
        return driverController.getLeftTriggerAxis()>0;
    }

    public boolean IntakeRolllerZero()
    {
        return (driverController.getPOV() == 0)|| driverController.getRightStickButton();
    }

    public boolean ResetGyro()
    {
        return practiceController.getXButton();
    }

    /*Shooter roller */
    public boolean SpeakerShootRoller(){
        return driverController.getRightBumper();
    }
     public boolean AmpShoot()
    {
        return practiceController.getAButton();
    }

    /*Climber */
    public boolean Climber1Positive(){
        return operatorController.getR2Button();
    }
    public boolean Climber1Negative(){
        return operatorController.getR1Button();
    }


    public boolean ShooterFixedPos(){
        return operatorController.getTriangleButton();
    }

    public boolean Climber2Positive()
    {
        return operatorController.getL2Button();
    }
    
    public boolean Climber2Negative()
    {
        return operatorController.getL1Button();
    }

   
  
    public boolean IntakeTakeCommand(){
        return driverController.getLeftBumper();
    }

      public boolean IntakeAmpPose(){
        return driverController.getPOV()==0;
    }

    public boolean IntakeGroundPose()
    {
        return driverController.getPOV()==90;
    }

    public boolean IntakeFeedPose()
    {
        return driverController.getPOV()==270;
    }

    public boolean IntakeStartPose()
    {
        return driverController.getPOV()==180;
    }

    public boolean slowMod()
    {
        return practiceController.getYButton();
    }



    public void registerTriggers()
    {

        Command driveFieldOrientedDirectAngle = swerveSubsystem.driveCommand2(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFTY_DEADBAND)*speedRate,
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFTX_DEADBAND)*speedRate,
        () ->  MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHTX_DEADBAND),
        ()->!driverController.getYButton());

        swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    // Intake
    new Trigger(this::IntakeRollerOut).onTrue(new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)))
                                        .onFalse(new InstantCommand(()->m_intakeRoller.setRollerMotor(0)));
    new Trigger(this::IntakeRollerIn).onTrue(new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER))
                                        .onFalse(new IntakeRoller(0));
    
    new Trigger(this::ResetGyro).onTrue(new InstantCommand(()->swerveSubsystem.zeroGyro()));



    // Shooter Power
    new Trigger(this::SpeakerShootRoller).onTrue(new ShooterRoller(Constants.ShooterConstant.SPEAKER_POWER))
                                        .onFalse(new ShooterRoller(0));
    new Trigger(this::AmpShoot).onTrue((new ShooterRoller(Constants.ShooterConstant.AMP_POWER)))
                                .onFalse((new ShooterRoller(0)));

    // Climber
    new Trigger(this::Climber1Positive).onTrue(new InstantCommand(()->m_climber.climber1Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber1Motor(0)));
    new Trigger(this::Climber1Negative).onTrue(new InstantCommand(()->m_climber.climber1Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber1Motor(0)));
    new Trigger(this::Climber2Positive).onTrue(new InstantCommand(()->m_climber.climber2Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber2Motor(0)));
    new Trigger(this::Climber2Negative).onTrue(new InstantCommand(()->m_climber.climber2Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber2Motor(0)));

    // intake pose
    new Trigger(this::IntakeAmpPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(90))));
                                        
    new Trigger(this::IntakeGroundPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(210))));
                                        
     new Trigger(this::IntakeFeedPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(10))));

    new Trigger(this::IntakeStartPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(0))));
                                        

    // Command 
    new Trigger(this::slowMod).onTrue(new InstantCommand(()->speedRate=0.2))
                                .onFalse(new InstantCommand(()->speedRate=1));
    new Trigger(this::IntakeTakeCommand).onTrue(new IntakeTakeSequence());


    
    }
    public static DriverControlsSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new DriverControlsSubsystem();
        }
        return instance;
    }
    
    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
    }


}