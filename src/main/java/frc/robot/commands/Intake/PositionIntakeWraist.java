// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeWraist;

public class PositionIntakeWraist extends Command {
  private final IntakeWraist m_IntakeWraist;
  private final String m_NewPosition;
  private Timer m_timer;
  boolean m_isdone;

  /** Creates a new PositionIntakeWraist. */
  public PositionIntakeWraist(IntakeWraist m_positionIntakeWraist, String whichPosition) {
    // Set local variables
    m_IntakeWraist = m_positionIntakeWraist;
    m_NewPosition = whichPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeWraist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeWraist.MoveIntakeToPosition(m_NewPosition);
    m_isdone = false;
    //Start timer as second kill out this command
    m_timer = new Timer();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* This ensures this commands ends if sensor not found */
    if (m_timer.get() > Constants.ManipulatorConstants.kIntakeWraistMotor) {
      m_isdone = true;
    }
    /* This checks to see if arm is position */
    if (m_IntakeWraist.isIntakeWraistInPosition() == true) {
      m_isdone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isdone;
  }
}
