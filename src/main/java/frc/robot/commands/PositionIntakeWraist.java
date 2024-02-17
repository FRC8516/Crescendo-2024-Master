// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWraist;

public class PositionIntakeWraist extends Command {
  private final IntakeWraist m_IntakeWraist;
  private final String m_NewPosition;

  /** Creates a new PositionIntakeWraist. */
  public PositionIntakeWraist(IntakeWraist m_positionIntakeWraist, String whichPosition) {
    m_IntakeWraist = m_positionIntakeWraist;
    m_NewPosition = whichPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeWraist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeWraist.MoveIntakeToPosition(m_NewPosition);
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
