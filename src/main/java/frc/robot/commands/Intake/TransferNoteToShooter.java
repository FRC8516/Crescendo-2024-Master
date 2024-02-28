// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeNote;
import frc.robot.subsystems.ShootNote;

public class TransferNoteToShooter extends Command {
  private final IntakeNote mIntakeNote;
  private final ShootNote mShootNote;
  private Timer m_timer;
  boolean m_isdone;

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
    mIntakeNote.IntakeTransferForAmp();
    m_isdone = false;
    //Start timer as second kill out this command
    m_timer = new Timer();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* This just cycles the transfer note command to timeout */
    if (m_timer.get() > Constants.ManipulatorConstants.kTransferTime) {
      m_isdone = true;
    }
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
    return m_isdone;
  }
}
