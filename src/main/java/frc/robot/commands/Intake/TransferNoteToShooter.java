// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeNote;
import frc.robot.subsystems.ShootNote;

public class TransferNoteToShooter extends Command {
  private final IntakeNote mIntakeNote;
  private final ShootNote mShootNote;

  /** Creates a new TransferNoteToShooter. */
  public TransferNoteToShooter(IntakeNote m_NoteTransfer, ShootNote m_ShootNote) {
    mIntakeNote = m_NoteTransfer;
    mShootNote = m_ShootNote;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeNote, mShootNote);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShootNote.TransferNoteToShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntakeNote.IntakeTransferForAmp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShootNote.StopShooterMotion();
    mIntakeNote.StopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
