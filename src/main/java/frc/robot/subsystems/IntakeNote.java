// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeNote extends SubsystemBase {
  /* Hardware */
  private final TalonFX m_IntakeMotor = new TalonFX(ManipulatorConstants.kIntakeMotor, "rio");
  
  /* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  //* Rotation Per Second */
  double dIntakeNoteSpeed = -50; // Go for plus/minus 10 rotations per second
  double dOutputNoteSpeed = 50; // Go for plus/minus 10 rotations per second
  double dTransferSpeed = 20;

  //* Read Digital Input */
  private DigitalInput m_sensorInput;
 
  /** Creates a new IntakeNote. */
  public IntakeNote() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_IntakeMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
    //*** */ ToDO  Check direction!!! ****
    m_IntakeMotor.setInverted(false);
    //*  game piece detection   */
    m_sensorInput = new DigitalInput(0);
    SmartDashboard.putBoolean("Game Piece", false);
    SmartDashboard.putNumber("Intake Time Out", 3.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Game Piece Detected", m_sensorInput.get());    
  }

  public void NoteIntake() {
     /* Use voltage velocity */
     m_IntakeMotor.setControl(m_voltageVelocity.withVelocity(dIntakeNoteSpeed));
  } 

  //Used transfer to shooter mechanism 
  public void ShootNoteToSpeaker() {
     /* Use voltage velocity */
     m_IntakeMotor.setControl(m_voltageVelocity.withVelocity(dOutputNoteSpeed));
  }

  public void IntakeTransferForAmp() {
    /* Use voltage velocity */
     m_IntakeMotor.setControl(m_voltageVelocity.withVelocity(dTransferSpeed));
  }

  //See if the note is intaked into holder
  public boolean getSensorIntake() {
    if (m_sensorInput.get() == true) {
        return true;
    } else {
        return false;
    }
}

  //* Stop motor motion / disable controller */
  public void StopMotion() {
     m_IntakeMotor.setControl(m_voltageVelocity.withVelocity(0));
     m_IntakeMotor.stopMotor();
  }

}
