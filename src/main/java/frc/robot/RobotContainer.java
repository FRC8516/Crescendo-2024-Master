// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakePositions;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterPositions;
import frc.robot.commands.Elevator.ClimbChain;
import frc.robot.commands.Intake.AutomaticNoteIntake;
import frc.robot.commands.Intake.InputNote;
import frc.robot.commands.Intake.PositionIntakeWraist;
import frc.robot.commands.Shooter.AutoShootAmp;
import frc.robot.commands.Shooter.AutoShootSpeaker;
import frc.robot.commands.Shooter.PositionShooterWraist;
import frc.robot.commands.Shooter.ShootAmp;
import frc.robot.commands.Shooter.ShootSpeaker;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeNote;
import frc.robot.subsystems.IntakeWraist;
import frc.robot.subsystems.ShootNote;
import frc.robot.subsystems.ShooterWraist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CandleControl;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Auto Selections from dashboard
  private final SendableChooser<Command> m_autoChooser;
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //* Candle Light Controller */
  public CandleControl m_CandleControl = new CandleControl();
  //* Constructors for Subsystems */
  private final IntakeNote mIntakeNote = new IntakeNote();
  private final ShootNote m_ShootNote = new ShootNote();
  private final ShooterWraist m_ShooterWraist = new ShooterWraist();
  private final Elevator m_Elevator = new Elevator();

  //* Constructors for Commands */
  //Intake Commands
  private final IntakeWraist m_IntakeWraist = new IntakeWraist();
  private final InputNote m_IntakeNote = new InputNote(mIntakeNote);
  private final PositionIntakeWraist mIntakeWraistFloor = new PositionIntakeWraist(m_IntakeWraist, IntakePositions.FloorPickup);
  private final PositionIntakeWraist mIntakeWraistHome = new PositionIntakeWraist(m_IntakeWraist, IntakePositions.HomePosition);  

  //Shooter Commands
  private final ShootAmp m_ShootAmp = new ShootAmp(m_ShootNote);
 // private final ShootSpeaker m_ShootSpeaker = new ShootSpeaker(m_ShootNote);
  private final AutoShootSpeaker m_AutoShootSpeaker = new AutoShootSpeaker(mIntakeNote, m_ShootNote);
  //private final AutoShootAmp m_AutoShootAmp = new AutoShootAmp(mIntakeNote, m_ShooterWraist, m_ShootNote);

  //Automatic intake / position
  private final AutomaticNoteIntake m_AutomaticNoteIntake = new AutomaticNoteIntake(m_IntakeWraist, mIntakeNote);
  //Shooter Wraist
  private final PositionShooterWraist m_PositionShooterWraist = new PositionShooterWraist(m_ShooterWraist, ShooterPositions.AmpScoringPosition);

  //* The driver's joystick controller */ 
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //* Operator joystick controller */
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    /* Auto selection   */
   // m_autoChooser.addOption("One Note", m_OneNoteAuto);
   // m_autoChooser.setDefaultOption("Two Note", m_TwoNoteAuto);
    NamedCommands.registerCommand("Auto Shoot Speaker", m_AutoShootSpeaker);
    NamedCommands.registerCommand("Auto Note Intake", m_AutomaticNoteIntake);
     m_autoChooser = AutoBuilder.buildAutoChooser();
     Shuffleboard.getTab("Auto").add("Auto Mode", m_autoChooser);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
     // Configure the elevator command
     m_Elevator.setDefaultCommand(
        new RunCommand(
            () -> m_Elevator.GoClimbChain(
              -MathUtil.applyDeadband(m_operatorController.getLeftY(), OIConstants.kDriveDeadband)),
              m_Elevator));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {        
    //Configure drive train
     m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.start().onTrue(Commands.runOnce(m_robotDrive::zeroHeading, m_robotDrive));

    //Configure Joysticks actuators
    //TESTING mech·an·ism #*********@@@@!!!!!!
     m_driverController.a().onTrue(m_IntakeNote);
  
     m_driverController.y().whileTrue(m_ShootAmp);
     m_driverController.rightBumper().onTrue(mIntakeWraistHome);

    //Operator Joystick triggers
    m_operatorController.leftTrigger().onTrue(m_AutomaticNoteIntake);
    m_operatorController.rightTrigger().onTrue(m_AutoShootSpeaker);
    // m_operatorController.rightBumper().onTrue(m_AutoShootAmp);
    m_operatorController.a().onTrue(m_PositionShooterWraist);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
