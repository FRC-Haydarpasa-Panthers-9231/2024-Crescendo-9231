package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.subsystems.IntakePivotSubsystem;

public class AutoIntakeSequence extends SequentialCommandGroup {
    private IntakePivotSubsystem m_intakePivot;
  public AutoIntakeSequence() {
    m_intakePivot=IntakePivotSubsystem.getInstance();

    addCommands(
  new SequentialCommandGroup(
       new ParallelCommandGroup(
        new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(204))), //intake bottom pozisyonuna getir
        new SequentialCommandGroup(
          new WaitCommand(0.5), // yarım saniye bekle
            new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER) // içeri nota al(limit switchden dolayı içeri alınca bitmesi lazım.)
        )
        ).withTimeout(4),
        new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(0))) // feedTop pozisyonuna getir
        )
);

   
  }
}
