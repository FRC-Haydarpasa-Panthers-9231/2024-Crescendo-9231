package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.Intake.IntakePivot;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterPivot;
import frc.robot.commands.Shooter.ShooterRoller;


public class DriverControlsSubsystem extends SubsystemBase{
    
    static DriverControlsSubsystem instance;
    private ClimberSubsystem m_climber;
    private IntakePivotSubsystem m_intakePivot;
    private IntakeRollerSubsystem m_intakeRoller;
    private ShooterPivotSubsystem m_shooterPivot;
    private ShooterRollerSubsystem m_shooterRoller;
    private SwerveSubsystem swerveSubsystem;
    private XboxController driverController;
    private PS4Controller operatorController;

    public DriverControlsSubsystem(){
        driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
        operatorController = new PS4Controller(Constants.OperatorConstants.kOperatorControllerPort);
        m_climber=ClimberSubsystem.getInstance();
        m_intakeRoller=IntakeRollerSubsystem.getInstance();
        m_intakePivot=IntakePivotSubsystem.getInstance();
        m_shooterPivot=ShooterPivotSubsystem.getInstance();
        m_shooterRoller=ShooterRollerSubsystem.getInstance();
        swerveSubsystem=SwerveSubsystem.getInstance();
    }

    public XboxController getDriverController(){
        return driverController;
    }

    public PS4Controller getOperatorController()
    {
        return operatorController;
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

    //Intake pivot 
    public boolean IntakePivotPositive(){
        return driverController.getYButton();
    }
    
    public boolean IntakePivotNegative(){
        return driverController.getXButton();
    }

    public boolean ResetGyro()
    {
        return driverController.getLeftBumper();
    }

    /*Shooter pivot *//* 
     public boolean ShooterPivotPositive(){
        return driverController.getRightBumper();
    }
     public boolean ShooterPivotNegative(){
        return driverController.getLeftBumper();
    }*/

    /*Shooter roller */
    public boolean ShooterRoller(){
        return operatorController.getCrossButton();
    }

    /*Climber */
    public boolean Climber1Positive(){
        return operatorController.getR2Button();
    }
    public boolean Climber1Negative(){
        return operatorController.getR1Button();
    }

    public boolean ShooterLimelightPos(){
        return operatorController.getCircleButton();
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

    public boolean IntakeAmpPose(){
        return operatorController.getPOV()==0;
    }

   // public boolean shooterDownPID()
    //{
    //    return driverController.getPOV()==180;
    //}

    public boolean IntakeGroundPose()
    {
        return operatorController.getPOV()==90;
    }

    public boolean IntakeFeedPose()
    {
        return operatorController.getPOV()==270;
    }

    public boolean IntakeStartPose()
    {
        return operatorController.getPOV()==180;
    }

    public void setRumble(double speed){
        driverController.setRumble(RumbleType.kBothRumble, speed);
    }

    public boolean isFieldOrianted(){
       return !driverController.getRightBumper();
    }

    public void registerTriggers()
    {

        Command driveFieldOrientedDirectAngle = swerveSubsystem.driveCommand2(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFTY_DEADBAND),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFTX_DEADBAND),
        () ->  MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHTX_DEADBAND),
        !driverController.getRightBumper());

        swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    // Intake
    new Trigger(this::IntakeRollerOut).onTrue(new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)))
                                        .onFalse(new InstantCommand(()->m_intakeRoller.setRollerMotor(0)));
    new Trigger(this::IntakeRollerIn).onTrue(new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER))
                                        .onFalse(new IntakeRoller(0));
    
    //for angular aiming
    // new Trigger(this::IntakeRollerOut).onTrue(new InstantCommand(()->swerveSubsystem.driveCommand2(Constants.IntakeConstants.AMP_SHOOT_POWER)))
    //                                     .onFalse(new InstantCommand(()->swerveSubsystem.driveCommand2());

    new Trigger(this::ResetGyro).onTrue(new InstantCommand(()->swerveSubsystem.zeroGyro()));

    //limlight disable
    new Trigger(this::ShooterLimelightPos).onTrue(new InstantCommand(()->m_shooterPivot.changeLimelightActivation(true)));   
     new Trigger(this::ShooterFixedPos).onTrue(new InstantCommand(()->m_shooterPivot.changeLimelightActivation(false)));


    new Trigger(this::IntakePivotPositive).onTrue(new InstantCommand(()->m_intakePivot.setPivotMotor(Constants.IntakeConstants.PIVOT_POWER)))
                                            .onFalse(new InstantCommand(()->m_intakePivot.setPivotMotor(0)));

    new Trigger(this::IntakePivotNegative).onTrue(new InstantCommand(()->m_intakePivot.setPivotMotor(-Constants.IntakeConstants.PIVOT_POWER)))
                                            .onFalse(new InstantCommand(()->m_intakePivot.setPivotMotor(0)));

    // Shooter
    //new Trigger(this::ShooterPivotPositive).onTrue(new ShooterPivot(Constants.ShooterConstant.PIVOT_POWER))
    //                                        .onFalse(new ShooterPivot(0));
    //new Trigger(this::ShooterPivotNegative).onTrue(new ShooterPivot(-Constants.ShooterConstant.PIVOT_POWER))
    //                                        .onFalse(new ShooterPivot(0));

    // Shooter
    new Trigger(this::ShooterRoller).onTrue(new ShooterRoller(Constants.ShooterConstant.ROLLER_POWER))
                                        .onFalse(new ShooterRoller(0));

    // Climber
    new Trigger(this::Climber1Positive).onTrue(new InstantCommand(()->m_climber.climber1Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber1Motor(0)));
    new Trigger(this::Climber1Negative).onTrue(new InstantCommand(()->m_climber.climber1Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber1Motor(0)));
    new Trigger(this::Climber2Positive).onTrue(new InstantCommand(()->m_climber.climber2Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber2Motor(0)));
    new Trigger(this::Climber2Negative).onTrue(new InstantCommand(()->m_climber.climber2Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber2Motor(0)));

    //PID
    
    //new Trigger(this::shooterTopPID).onTrue(new ShooterPIDControl(0.95));
    //new Trigger(this::shooterDownPID).onTrue(new ShooterPIDControl(1.05));
    new Trigger(this::IntakeAmpPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(90))));
                                        
    new Trigger(this::IntakeGroundPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(210))));
                                        
     new Trigger(this::IntakeFeedPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(10))));

    new Trigger(this::IntakeStartPose).onTrue(new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(0))));
                                        
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
        SmartDashboard.putNumber("TIMER", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("tus", driverController.getRightBumper());
        if(DriverStation.getMatchTime() < 20){
            //setRumble(1);
        }
        else if(DriverStation.getMatchTime() < 10){
            //setRumble(1);
        }
    }

    @Override
    public void simulationPeriodic() {
    }


}
