// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakePositions;
import frc.robot.subsystems.IntakeNote;
import frc.robot.subsystems.IntakeWraist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomaticNoteIntake extends SequentialCommandGroup {
  /** Creates a new AutomaticNoteIntake. */
  public AutomaticNoteIntake(IntakeWraist m_IntakeWraist, IntakeNote m_IntakeNote) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
      new PositionIntakeWraist(m_IntakeWraist, IntakePositions.FloorPickup).withTimeout(0.1),
      new InputNote(m_IntakeNote),
      new PositionIntakeWraist(m_IntakeWraist, IntakePositions.HomePosition).withTimeout(0.2)
    );
  }
}
