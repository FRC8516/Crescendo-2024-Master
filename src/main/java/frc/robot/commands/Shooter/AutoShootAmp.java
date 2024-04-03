// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterPositions;
import frc.robot.commands.Intake.TransferNoteToShooter;
import frc.robot.subsystems.IntakeNote;
import frc.robot.subsystems.ShootNote;
import frc.robot.subsystems.ShooterWraist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootAmp extends SequentialCommandGroup {
  /** Creates a new AutoShootAmp. */
  public AutoShootAmp(IntakeNote mTransferNote, ShooterWraist mWraistAmp, ShootNote mShootAmp) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TransferNoteToShooter(mTransferNote, mShootAmp),
                new PositionShooterWraist(mWraistAmp, ShooterPositions.AmpScoringPosition).withTimeout(1),
                new ShootAmp(mShootAmp).withTimeout(1.5),
                new PositionShooterWraist(mWraistAmp, ShooterPositions.TransferPosition)
                );
  }
}
