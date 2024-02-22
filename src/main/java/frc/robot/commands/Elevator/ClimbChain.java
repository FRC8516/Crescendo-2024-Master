// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ClimbChain extends Command {
   private final Elevator m_Elevator;
   private DoubleSupplier m_speed;

  /** Creates a new ClimbChain. */
  public ClimbChain(DoubleSupplier speed, Elevator mElevator) {
    m_Elevator = mElevator;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Math.copySign(m_speed.getAsDouble() * m_speed.getAsDouble(), m_speed.getAsDouble());
    m_Elevator.GoClimbChain(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.GoClimbChain(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
