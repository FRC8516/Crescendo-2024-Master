// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeNote;

public class OutTakeNote extends Command {
   private final IntakeNote m_InputNote;

  /** Creates a new OutTakeNote. */
  public OutTakeNote(IntakeNote mIntakeNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_InputNote = mIntakeNote;
    addRequirements(m_InputNote);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_InputNote.ShootNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
