// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterWraist;

public class PositionShooterWraist extends Command {
  private final ShooterWraist m_ShooterWraist;
  private final String m_NewPosition;
  private Timer m_timer;
  boolean m_isdone;

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
    m_isdone = false;
    //Start timer as second kill out this command
    m_timer = new Timer();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     /* This ensures this commands ends if sensor not found */
    if (m_timer.get() > Constants.ManipulatorConstants.kShooterWraistTime) {
      m_isdone = true;
    }
    /* This checks to see if arm is position */
    if (m_ShooterWraist.isShooterWraistInPosition() == true) {
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
