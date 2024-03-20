
package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

public class AutoOuttakeSequence extends SequentialCommandGroup {
    private IntakeRollerSubsystem m_intakeRoller;
  public AutoOuttakeSequence() {
    m_intakeRoller=IntakeRollerSubsystem.getInstance();

    addCommands(

       new SequentialCommandGroup(
        new InstantCommand(()->m_intakeRoller.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)), // outtake ver
        new WaitCommand(0.5), //0.5 saniye de feedle
        new InstantCommand(()->m_intakeRoller.setRollerMotor(0))
        )

    );

   
  }
}
