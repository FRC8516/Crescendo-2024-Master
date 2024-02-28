// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
    //Motion Magic Voltage output
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    // backup key values not returned from perference table on shuffleboard....80:1 Gear box
    // This value is entered degrees
	  final double PickUpPosition = 137;
	  final double TransferPosition = 40;
    final double LoadingStationPosition = 90;
    final double HomePosition = 0.1;
    //Use to get from the preference table
	  final String IntakePickup = "Intake Pickup";
	  final String IntakeTransfer = "Intake Transfer";
    final String IntakeLoading = "Intake Loading";
    final String IntakeHome = "Intake Home";
    //local setpoint for moving to position by magic motion
	  private double setPoint;
	  private double backUp;
    private String Key;
	  //local variable to keep track of position
    StatusSignal<Double> dCurrentPosition;
    /* Keep a brake request so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();
    /* Scale for 360 degrees */
    private double scale = 360;
  
    /** Creates a new IntakeWraist. */
  public IntakeWraist() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    //Used for the homing of the mech
    configs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    configs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    configs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
    //Software limits - forward motion
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 139;  //28

    /** *********************************************************************************************
     *  Motion Magic
    /*  Configure current limits   */
    MotionMagicConfigs mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 6; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 1.5; // Target acceleration of 400 rps/s (0.25 seconds to max)
    mm.MotionMagicJerk = 50; // Target jerk of 4000 rps/s/s (0.1 seconds)

    Slot0Configs slot0 = configs.Slot0;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60;   // An error of 1 rps results in 0.11 V output
    slot0.kI = 0;    // no output for integrated error
    slot0.kD = 1;    // no output for error derivative
    // 80:1 Gearbox
    FeedbackConfigs fdb = configs.Feedback;
    fdb.SensorToMechanismRatio = 80;

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
    dCurrentPosition = m_IntakeWraistMotor.getPosition();
    SmartDashboard.putNumber("Intake Position", dCurrentPosition.getValue());
  }

  //Call by the commands to move intake wraist to positions
  public void MoveIntakeToPosition(String sMoveTo) {
    //set up the grab from values at Smart Dashboard perference table
    switch (sMoveTo) {
      case IntakePositions.FloorPickup:;
        //Floor Pickup
        backUp = PickUpPosition;
        Key = IntakePickup;
        break;
      case IntakePositions.LoadingStationPosition:;
        //Loading Station note position
        backUp = LoadingStationPosition;
        Key = IntakeLoading;
        break;
      case IntakePositions.TransferPosition:;
         //Transfer note position
        backUp = TransferPosition;
        Key = IntakeTransfer;
        break;
      case IntakePositions.HomePosition:;
        //Move to home position
        backUp = 0.1;
        Key = IntakeHome;
        break;
    }
    //gets the current value
	  setPoint = getPreferencesDouble(Key, backUp);
    //sets the new position to the motor controller.
	  this.MoveToPosition(setPoint/scale);
  }

  //Move the Intake back to the home position
  public void MoveToHomePosition() {
     /* Use voltage position */
     m_IntakeWraistMotor.setControl(m_mmReq.withPosition(0.1).withSlot(0));
  }

  private void MoveToPosition(double targetPos) {
    /* Use voltage position */
     m_IntakeWraistMotor.setControl(m_mmReq.withPosition(targetPos).withSlot(0));
  }

  //This checks Current positon to setpoint for the commands calls - isFinished flag
  public Boolean isIntakeWraistInPosition() {
    double dError = dCurrentPosition.getValue() - setPoint;
    //Returns the check to see if the elevator is in position
    if ((dError < 0.5) || (dError > -0.5)) {
      return true;
    } else {
      return false;
    }
  }

  public void StopMotion() {
    m_IntakeWraistMotor.setControl(m_brake);
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
