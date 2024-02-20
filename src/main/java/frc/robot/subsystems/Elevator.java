// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.ShooterPositions;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Elevator extends SubsystemBase {
  	/* Hardware */
  	private final TalonFX m_ElevatorMotor = new TalonFX(ManipulatorConstants.kElevatorMotor, "rio");
	 //Motion Magic
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
	 //backup key values not returned from perference table on shuffleboard //44 revs to max height
	 final double FullyExtended = 5.0;
	 final double AmpPosition = 3.0;	
	//Use to get from the preference table
	 final String ElevatorHigh = "Elevator High";
	 final String ElevatorAmp = "Elvator Amp";
	 //local setpoint for moving to position by magic motion
	 private double setPoint;
	 private double backUp;
	 //locat variable to keep track of position
	 StatusSignal<Double> dCurrentPosition;
		
  /** Creates a new Elevator. */
  public Elevator() {
	/* Factory default hardware to prevent unexpected behavior */
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
	   status = m_ElevatorMotor.getConfigurator().apply(configs);
	   if (status.isOK()) break;
	 }
	 if(!status.isOK()) {
	   System.out.println("Could not apply configs, error code: " + status.toString());
	 }
	 //*** */ ToDO  Check direction!!! ****
	 m_ElevatorMotor.setInverted(false);
	 
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	//Updates position on the dashboard
     dCurrentPosition = m_ElevatorMotor.getPosition();
    SmartDashboard.putNumber("Elevator Position", dCurrentPosition.getValue());
  }

  public void SetElevatorToPosition(String Key) {
    //set up the grab from values at Smart Dashboard perference table
	switch (Key) {
		case ShooterPositions.ExtendedPosition:;
			//Elevator High
		  	backUp = FullyExtended;
			break;
		case ShooterPositions.AmpScoringPosition:;
		    //Elevator Mid - Amp scoring position
		    backUp = AmpPosition;
			break;
		case ShooterPositions.TransferPosition:;
			//Home position / transfer
			backUp = 0.1;
		}

	//gets the current value
	setPoint = getPreferencesDouble(Key, backUp);  

	//sets the new position to the motor controller.
	this.MoveToPosition(setPoint);
  }

  public void MoveToPosition(double targetPos) {
	 /* Use voltage position */
     m_ElevatorMotor.setControl(m_mmReq.withPosition(targetPos).withSlot(0));
  }

  //This checks Current positon to setpoint for the commands calls - isFinished flag
  public Boolean isElevatorInPosition() {
	double dError = dCurrentPosition.getValue() - setPoint;
	//Returns the check to see if the elevator is in position
	if ((dError < 0.5) || (dError > -0.5)) {
		return true;
	} else {
		return false;
	}
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

