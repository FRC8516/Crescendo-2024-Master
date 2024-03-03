// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Elevator extends SubsystemBase {
  	/* Hardware */
  	private final TalonFX m_ElevatorMotor = new TalonFX(ManipulatorConstants.kElevatorMotor, "rio");
		
  /** Creates a new Elevator. */
  public Elevator() {
    /* Factory default hardware to prevent unexpected behavior */
      TalonFXConfiguration configs = new TalonFXConfiguration();
    //Software limits - Reverse motion
    //configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    //Software limits - forward motion
    //configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ;
    //Set configurations  
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_ElevatorMotor.getConfigurator().apply(configs);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void GoClimbChain(double dspeed) {
	  m_ElevatorMotor.set(dspeed);
  }
 
	/**
    * Retrieve numbers from the preferences table. If the specified key is in
    * the preferences table, then the preference value is returned. Otherwise,
    * return the backup value, and also start a new entry in the preferences
    * table.
	 * @return 
    
    private double getPreferencesDouble(String key, double backup) {
		if (!Preferences.containsKey(key)) {
		  Preferences.initDouble(key, backup);
		  Preferences.setDouble(key, backup);
		}
		return Preferences.getDouble(key, backup);
	  }  */
}