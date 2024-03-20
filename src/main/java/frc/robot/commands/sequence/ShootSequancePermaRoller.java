
package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import frc.robot.commands.Shooter.ShooterRoller;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterRollerSubsystem;



public class ShootSequancePermaRoller extends SequentialCommandGroup {
    private IntakeRollerSubsystem m_intakeRoller;
    private ShooterRollerSubsystem m_shooterRoller;
  public ShootSequancePermaRoller() {
        m_intakeRoller=IntakeRollerSubsystem.getInstance();


    addCommands(
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new SequentialCommandGroup( 
              new WaitCommand(1), //1 saniye bekle
              new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)) // intake roll out ver ve atış yap
                                      )
                                  ).withTimeout(2.5), //2.5
          new WaitCommand(0.1), // 0.9 saniye bekle
          new ParallelCommandGroup(
            new InstantCommand(()->m_intakeRoller.setRollerMotor(0)) //intake roller motorunu durdur
      )));
  }
}
