// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeWraist extends SubsystemBase {
  /* Hardware */
  private final TalonFX m_IntakeWraistMotor = new TalonFX(ManipulatorConstants.kIntakeWraistMotor, "rio");
    //Motion Magic
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    // create a Motion Magic Velocity request, voltage output
    // private final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    //backup key values not returned from perference table on shuffleboard....80:1 Gear box
	  final double PickUpPosition = 10.0;
	  final double TransferPosition = 2.0;
    //Use to get from the preference table
	  final String IntakePickup = "Intake Pickup";
	  final String IntakeTransfer = "Intake Transfer";
    //local setpoint for moving to position by magic motion
	  private double setPoint;
	  private double backUp;
    private String Key;
	  //local variable to keep track of position
	  private double currentSetPoint;
	  private double moveSetpoint;
  
    /** Creates a new IntakeWraist. */
  public IntakeWraist() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /** *********************************************************************************************
     *  Motion Magic
    /* Configure current limits   */
    MotionMagicConfigs mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
    mm.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    Slot0Configs slot0 = configs.Slot0;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0; // no output for error derivative  
    
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_IntakeWraistMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //Updates position on the dashboard
    StatusSignal<Double> dPosition = m_IntakeWraistMotor.getPosition();
    SmartDashboard.putNumber("Intake Position", dPosition.getValue());
  }

  //Call by the commands to move intake wraist to positions
  public void MoveIntakeToPosition(String sMoveTo) {
    //Determines which position to move the intake wraist.
    if (sMoveTo == IntakePositions.FloorPickup) {
      //Floor Pickup
		  	backUp = PickUpPosition;
        Key = IntakePickup;
    } else {
      //Transfer note position
		  	backUp = TransferPosition;
        Key = IntakeTransfer;
    }

    //gets the current value
	  setPoint = getPreferencesDouble(Key, backUp);
    //sets the new position to the motor controller.
	  this.MoveToPosition(setPoint);
  }

  //Move the Intake back to the home position
  public void MoveToHomePosition() {
     /* Use voltage position */
     m_IntakeWraistMotor.setControl(m_mmReq.withPosition(0.2).withSlot(0));
  }

  private void MoveToPosition(double targetPos) {
    /* Use voltage position */
     m_IntakeWraistMotor.setControl(m_mmReq.withPosition(targetPos).withSlot(0));
  }

  /**
    * Retrieve numbers from the preferences table. If the specified key is in
    * the preferences table, then the preference value is returned. Otherwise,
    * return the backup value, and also start a new entry in the preferences
    * table.
	 * @return 
    */
    private double getPreferencesDouble(String key, double backup) {
		if (!Preferences.containsKey(key)) {
		  Preferences.initDouble(key, backup);
		  Preferences.setDouble(key, backup);
		}
		return Preferences.getDouble(key, backup);
	  }
}
