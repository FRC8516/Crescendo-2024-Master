// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootNote extends SubsystemBase {
  //Vortex Motors, Flex Max- OutTake to shooter
  private CANSparkFlex m_OutTopMotor;
  private CANSparkFlex m_OutBottomMotor;
  //Vortex Motors, Flex Max -Intake to 
  private CANSparkFlex m_IntakeTopMotor;
  private CANSparkFlex m_IntakeBottomMotor;

  /** Creates a new ShootNote. */
  public ShootNote() {
    // create four new FLEX SPARK MAXs and configure them
    /*  OutTake Top Motor */
    m_OutTopMotor =
        new CANSparkFlex(Constants.Launcher.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_OutTopMotor.setInverted(true);
    m_OutTopMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_OutTopMotor.setIdleMode(IdleMode.kCoast);
    //burn config to flash
    m_OutTopMotor.burnFlash();
    /* OutTake Bottom Motor */
    m_OutBottomMotor =
        new CANSparkFlex(Constants.Launcher.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_OutBottomMotor.setInverted(false);
    m_OutBottomMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_OutBottomMotor.setIdleMode(IdleMode.kCoast);
     //burn config to flash
    m_OutBottomMotor.burnFlash();
    /*  InTake Top Motor */
    m_IntakeTopMotor =
        new CANSparkFlex(Constants.Launcher.kInTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_IntakeTopMotor.setInverted(true);
    m_IntakeTopMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_IntakeTopMotor.setIdleMode(IdleMode.kCoast);
    //burn config to flash
    m_IntakeTopMotor.burnFlash();
    /*  InTake Bottom Motor */
    m_IntakeBottomMotor =
        new CANSparkFlex(Constants.Launcher.kInBotCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_IntakeBottomMotor.setInverted(false);
    m_IntakeBottomMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_IntakeBottomMotor.setIdleMode(IdleMode.kCoast);
    //burn config to flash
    m_OutTopMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Transfer Note from Intake to Shooter
  public void TransferNoteToShooter () {
    m_IntakeTopMotor.set(Constants.Launcher.kTopAmpPower);
    m_IntakeBottomMotor.set(Constants.Launcher.kBotAmpPower);
  }

  // Scoring into the Note to Amp
  public void ScoreAmp () {
    // (Top roller is slower then Bottom roller)
    m_OutTopMotor.set(Constants.Launcher.kTopAmpPower);
    m_OutBottomMotor.set(Constants.Launcher.kBotAmpPower);
    m_IntakeTopMotor.set(Constants.Launcher.kTopAmpPower);
    m_IntakeBottomMotor.set(Constants.Launcher.kBotAmpPower);
  }

  // Shooting Note to Speaker
  public void ScoreSpeaker () {
    // set the launcher motor powers based on constant file 
    // (Top roller is slower then Bottom roller)
    m_OutTopMotor.set(Constants.Launcher.kTopSpeakerPower);
    m_OutBottomMotor.set(Constants.Launcher.kBottomSpeakerPower);
    m_IntakeTopMotor.set(Constants.Launcher.kTopSpeakerPower);
    m_IntakeBottomMotor.set(Constants.Launcher.kBottomSpeakerPower);
  }

  //Stops all motors when commands are completed
  public void StopShooterMotion() {
    m_OutTopMotor.set(0.0);
    m_OutBottomMotor.set(0.0);
    m_IntakeTopMotor.set(0.0);
    m_IntakeBottomMotor.set(0.0);
  }

}
