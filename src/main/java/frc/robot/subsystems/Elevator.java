// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;

public class Elevator extends SubsystemBase {
  	/* Hardware */
  private final TalonFX m_ElevatorMotor = new TalonFX(ManipulatorConstants.kIntakeMotor, "rio");
	/*TO DO */
	 //backup key values not returned from perference table on shuffleboard //44 revs to max height
	 final double FullyExtended = 5.0;
	
	//Use to get from the preference table
	 final String ElevatorHigh = "Elevator High";
	 final String ElevatorDefault = "Elvator Home";
	 //local setpoint for moving to position by magic motion
	 private double setPoint;
	 private double backUp;
	 //locat variable to keep track of position
	 private double currentSetPoint;
	 private double moveSetpoint;

  /** Creates a new Elevator. */
  public Elevator() {
	/* Factory default hardware to prevent unexpected behavior */
	 m_ElevatorMotor.getConfigurator();

     TalonFXConfiguration configs = new TalonFXConfiguration();

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
//	SmartDashboard.putNumber("Elevator Encoder", m_ElevatorMotor.getRotorPosition(EncoderConstants.kPIDLoopIdx));
  }

  public void SetElevatorToPosition(String Key) {
    //set up the grab from values at Smart Dashboard perference table
	/*switch (Key) {
		case Elevator:;
			//Elevator Low
		  	backUp = LowScore;
			Key = ElevatorLow;
			break;
		case RobotArmPos.ScoreMid:;
		    //Elevator Mid
		    backUp = MidScore;
			Key = ElevatorMid;
			break;

		}
	   */
	//gets the current value
	setPoint = getPreferencesDouble(Key, backUp);  
	
	/* Motion Magic */
	/* 2048 ticks/rev * x Rotations in either direction */
	double targetPos = setPoint * 2048;
	//sets the new position to the motor controller.
	this.MoveToPosition(targetPos);
  }

  public void ElevatorUp(){
    moveSetpoint = currentSetPoint + 2048;
	this.MoveToPosition(moveSetpoint);
  }

  public void ElevatorDown(){
	moveSetpoint = currentSetPoint - 2048;
	this.MoveToPosition(moveSetpoint);
  }
  
  //Motor control mode motion magic to set point
  private void MoveToPosition(double targetPos) {
	//m_ElevatorMotor.set(TalonFXControlMode.MotionMagic, targetPos);
	//currentSetPoint = targetPos;
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

