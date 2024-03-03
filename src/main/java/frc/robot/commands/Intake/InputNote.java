// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeNote;

public class InputNote extends Command {
  private final IntakeNote m_InputNote;
  private Timer m_timer;
  double m_TimeOut= 3.0;
  boolean m_isdone;

  /** Creates a new IntakeNote. */
  public InputNote(IntakeNote inputNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_InputNote = inputNote;
    addRequirements(m_InputNote);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // SmartDashboard.getNumber("Intake Time Out", m_TimeOut);
    m_InputNote.NoteIntake();
    m_isdone = false;
    //Start timer as second kill out this command
    m_timer = new Timer();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* This checks subsystem to see if the sensor sees the note */
    if (m_InputNote.getSensorIntake() == true) {
      m_InputNote.StopMotion();
      m_isdone = true;
    }
    /* This ensures this commands ends if sensor not found */
    if (m_timer.get() > m_TimeOut) {
      m_isdone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_InputNote.StopMotion();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isdone;
  }
}
