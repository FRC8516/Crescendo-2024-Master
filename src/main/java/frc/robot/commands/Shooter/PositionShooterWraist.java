// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterWraist;

public class PositionShooterWraist extends Command {
  private final ShooterWraist m_ShooterWraist;
  private final String m_NewPosition;
 
  /** Creates a new PositionShooterWraist. */
  public PositionShooterWraist(ShooterWraist mShooterWraist, String whichPosition) {
    m_ShooterWraist = mShooterWraist;
    m_NewPosition = whichPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterWraist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterWraist.MoveShooterToPosition(m_NewPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ShooterWraist.isShooterWraistInPosition();
  }
}
