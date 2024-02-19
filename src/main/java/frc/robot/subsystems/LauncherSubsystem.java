package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class LauncherSubsystem extends SubsystemBase {
  //Vortex Motors - Flex Max
  private CANSparkMax m_topMotor;
  private CANSparkMax m_bottomMotor;
  //Falcon 500 Motor
  private final TalonFX m_IntakeShooter;

  private boolean m_launcherRunning;

  /**
   * Creates a new LauncherSubsystem.
   */
  public LauncherSubsystem() {
    // create two new FLEX SPARK MAXs and configure them
    m_topMotor =
        new CANSparkMax(Constants.Launcher.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_topMotor.setInverted(false);
    m_topMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_topMotor.setIdleMode(IdleMode.kBrake);

    m_topMotor.burnFlash();

    m_bottomMotor =
        new CANSparkMax(Constants.Launcher.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_bottomMotor.setInverted(false);
    m_bottomMotor.setSmartCurrentLimit(Constants.Launcher.kCurrentLimit);
    m_bottomMotor.setIdleMode(IdleMode.kBrake);

    m_bottomMotor.burnFlash();
    m_launcherRunning = false;
    
    //Intake to the shooter motor is Falcon 500
    m_IntakeShooter = new TalonFX(ManipulatorConstants.kShooterIntakeWraistMotor, "rio");
  }

  /**
   * Turns the launcher on.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
   */
  public void runLauncher() {
    m_launcherRunning = true;
  }

  /**
   * Turns the launcher off.  Can be run once and the launcher will stay running or run continuously in a {@code RunCommand}.
   */
  public void stopLauncher() {
    m_launcherRunning = false;
  }

  @Override
  public void periodic() {  // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    if (m_launcherRunning) {
      m_topMotor.set(Constants.Launcher.kTopPower);
      m_bottomMotor.set(Constants.Launcher.kBottomPower);
    } else {
      m_topMotor.set(0.0);
      m_bottomMotor.set(0.0);
    }
  }
}