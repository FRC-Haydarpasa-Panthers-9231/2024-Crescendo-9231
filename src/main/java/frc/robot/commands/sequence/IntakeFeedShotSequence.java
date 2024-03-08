
package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakePivotSubsystem;

public class IntakeFeedShotSequence extends SequentialCommandGroup {
    private IntakePivotSubsystem m_intakePivot;
  public IntakeFeedShotSequence() {
    m_intakePivot=IntakePivotSubsystem.getInstance();

    addCommands(

       new SequentialCommandGroup(
        new InstantCommand(()->m_intakePivot.pivotSet(Rotation2d.fromDegrees(10))), // feedTop pozisyonuna getir
        new ShootSequence()// shoot çalıştr
        )

    );

   
  }
}
