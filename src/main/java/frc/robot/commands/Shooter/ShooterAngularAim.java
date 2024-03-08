// package frc.robot.commands.Shooter;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriverControlsSubsystem;
// import frc.robot.subsystems.IntakeRollerSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.Vision;

// public class ShooterAngularAim extends Command  {
//     private DriverControlsSubsystem driverControlsSubsystem;
//     private SwerveSubsystem swerveSubsystem;
//     private Vision vision;
//     private double speed;
//     private PIDController shooterPID;
//     private double degreeAim;

//     public ShooterAngularAim(double speed)
//     {
//         driverControlsSubsystem=DriverControlsSubsystem.getInstance();
//         swerveSubsystem = SwerveSubsystem.getInstance();
//         vision =
//         this.speed=speed;
        
        
//         addRequirements(driverControlsSubsystem, swerveSubsystem, vision);
        
//     }

//     private void PIDinitialize(double degree)
//     {
//         degreeAim = degree;
//         shooterPID.reset();
//         shooterPID.setSetpoint(degree);
//         shooterPID.setTolerance(0.01);
        
//     }

//     @Override
//     public void initialize()
//     {
//         shooterPID=new PIDController(3,0,0);
//         PIDinitialize(1.05);
//     }

//     @Override
//     public void execute() {
//         swerveSubsystem.driveCommand2(0,0, Vision().instance, true);
//     }
//     @Override
//     public void end(boolean interrupted)
//     {
//         shooterPID.reset();
//     }

//     @Override 
//     public boolean isFinished()
//     {
//         //TODO tuş atama
//         return driverControlsSubsystem.Climber1Negative();
//     }
// }
