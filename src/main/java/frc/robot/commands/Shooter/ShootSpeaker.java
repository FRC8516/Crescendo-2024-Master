// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeNote;
import frc.robot.subsystems.LauncherSubsystem;

public class ShootSpeaker extends Command {
  private final LauncherSubsystem mLauncherSubsystem;
  private final IntakeNote mIntakeNote;
  Timer mDelayTimer;

  /** Creates a new ShootSpeaker. */
  public ShootSpeaker(LauncherSubsystem m_LauncherSubsystem, IntakeNote m_IntakeNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    mLauncherSubsystem = m_LauncherSubsystem;
    mIntakeNote = m_IntakeNote;
    mDelayTimer.delay(0.1);
    addRequirements(mLauncherSubsystem,mIntakeNote);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLauncherSubsystem.ScoreSpeaker();
    mDelayTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mLauncherSubsystem.ScoreSpeaker();
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
