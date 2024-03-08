
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



public class ShootSequence extends SequentialCommandGroup {
    private IntakeRollerSubsystem m_intakeRoller;
    private ShooterRollerSubsystem m_shooterRoller;
  public ShootSequence() {
        m_intakeRoller=IntakeRollerSubsystem.getInstance();
        m_shooterRoller=ShooterRollerSubsystem.getInstance();


    addCommands(
        new SequentialCommandGroup(
        new ParallelRaceGroup(
        new ShooterRoller(Constants.ShooterConstant.ROLLER_POWER), //shooter çalıştır
        new SequentialCommandGroup( 
            new WaitCommand(1), //1 saniye bekle
            new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)) // intake roll out ver ve atış yap
        ),
        new WaitCommand(2) // 2 saniye içinde komutlar bitmemişse hepsini bitir
      ),
      new WaitCommand(0.9), // 0.9 saniye bekle
      new InstantCommand(()->m_shooterRoller.setRollerMotor(0)), //shooter roller motorunu durdur
      new InstantCommand(()->m_intakeRoller.setRollerMotor(0)) //intake roller motorunu durdur
        )
      );
  }
}
