// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.RobotBase;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.RunIntake;
import frc.robot.commands.IdleIntake;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.commands.RunMagazine;
import frc.robot.commands.ServoDeflectorOn;
import frc.robot.commands.ServoDeflectorOff;
import frc.robot.commands.StageMagazine;
import frc.robot.commands.StopShooter;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ShootAmp;
//import frc.robot.commands.Wait;
import frc.robot.commands.ShootMagazine;
import frc.robot.commands.PukeIntake;
import frc.robot.subsystems.ServoSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.RetractClimber;
import frc.robot.subsystems.NoteSensor;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.GoalEndState;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;


//import frc.robot.commands.swervedrive.drivebase.CustomDrive3688;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final IntakeSubsystem intakeMotors = new IntakeSubsystem();

  private final ShooterSubsystem shooterMotors = new ShooterSubsystem();

  private final MagazineSubsystem magazineMotors = new MagazineSubsystem();

  private final ServoSubsystem servoMotor = new ServoSubsystem();

  private final ClimberSubsystem climberMotors = new ClimberSubsystem();

  private final NoteSensor noteSensor = new NoteSensor();

  private final SendableChooser<Command> autoChooser;
  
  
  Joystick shooterController = new Joystick(Constants.OperatorConstants.SHOOTER_USB_PORT);
  Joystick driverController = new Joystick(Constants.OperatorConstants.DRIVER_USB_PORT);

  
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //Register commands for Pathplanner/autonomous
    NamedCommands.registerCommand("ServoDeflectorOff", new ServoDeflectorOff(servoMotor));
    NamedCommands.registerCommand("StageMagazine", new StageMagazine(magazineMotors));
    NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(shooterMotors));
    NamedCommands.registerCommand("ShootMagazine", new ShootMagazine(magazineMotors));
    NamedCommands.registerCommand("StopShooter", new StopShooter(shooterMotors));
    NamedCommands.registerCommand("ServoDeflectorOn", new ServoDeflectorOn(servoMotor));
    NamedCommands.registerCommand("ShootAmp", new ShootAmp(shooterMotors));
    NamedCommands.registerCommand("RunIntake", new RunIntake(intakeMotors));
    NamedCommands.registerCommand("StopIntake", new StopIntake(intakeMotors));
    NamedCommands.registerCommand("RunMagazine", new RunMagazine(magazineMotors));

    // Creates UsbCamera and MjpegServer and connects them
    // CameraServer.startAutomaticCapture();
    UsbCamera camera0 = CameraServer.startAutomaticCapture(0);
    UsbCamera camera1 = CameraServer.startAutomaticCapture(1);
                camera0.setResolution(160, 120);
                camera1.setResolution(160, 120);
    
    // Configure the trigger bindings
    configureBindings();
  

// Added to test TeleopDrive code in robot-centric
// This proved more controllable than CustomDrive3688; also, the 180-degree slowdown was gone
    TeleopDrive teleopRobotCentric = new TeleopDrive(
        drivebase,
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-(
                                        (driverController.getRawAxis(2)) *    // Cube the input for joystick curve
                                        (driverController.getRawAxis(2)) *
                                        (driverController.getRawAxis(2)) *
                                        Constants.OperatorConstants.TURN_FACTOR  // Factor to moderate turn speed
                                      ), OperatorConstants.LEFT_Z_DEADBAND),
        () -> false); // true = field-centric

//TODO all of the joystick inputs are here, can the intake from getLeftX()

   //IdleIntake idleIntake = new IdleIntake(intakeMotors);

    

     drivebase.setDefaultCommand(teleopRobotCentric);

     //intakeMotors.setDefaultCommand(idleIntake);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
     

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // ZERO GYRO
    new JoystickButton(driverController, 7).onTrue((new InstantCommand(drivebase::zeroGyro)));
    
    // LOCK DRIVE
    new JoystickButton(driverController, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    // RUN INTAKE/MAGAZINE FOR FLOOR PICKUP
    new JoystickButton(shooterController, 1).whileTrue(new RunIntake(intakeMotors)
                                                      .alongWith(new RunMagazine(magazineMotors))
                                                                                ); 
    // SHOOT SPEAKER
    new JoystickButton(driverController, 6).onTrue(new InstantCommand(drivebase::lock, drivebase)
                                                      .andThen(new ServoDeflectorOff(servoMotor))
                                                      .andThen(new StageMagazine(magazineMotors)) // position note down -- built in timer
                                                      .andThen(new ShootSpeaker(shooterMotors)) // spin up shooter motors -- built in timer                               
                                                      .andThen(new ShootMagazine(magazineMotors)) // shoots magazine -- built in timer
                                                      .andThen(new StopShooter(shooterMotors))  // stop shooter motors
                                                                                 ); 
    // SHOOT AMP
    new JoystickButton(driverController, 4).onTrue((new ServoDeflectorOn(servoMotor))
                                                      .andThen(new StageMagazine(magazineMotors)) // position note down -- built in timer
                                                      .andThen(new ShootAmp(shooterMotors)) // spin up shooter motors -- built in timer    
                                                      .andThen(new ShootMagazine(magazineMotors)) // shoots magazine -- built in timer
                                                      .andThen(new StopShooter(shooterMotors))  // stop shooter motors
                                                      .andThen(new ServoDeflectorOff(servoMotor))
                                                                                ); 

    // PUKE INTAKE
    new JoystickButton(shooterController,2).whileTrue(new PukeIntake(intakeMotors));  //reverses intake motors

    //TODO: Servo test buttons.  Delete this
    new JoystickButton(shooterController,10).whileTrue(new ServoDeflectorOn(servoMotor)); // servo to deflect position
    new JoystickButton(shooterController,12).whileTrue(new ServoDeflectorOff(servoMotor)); // servo to retract position

    // OPERATE CLIMBER
    new JoystickButton(shooterController,6).whileTrue(new ExtendClimber(climberMotors)); // extend climber
    new JoystickButton(shooterController,4).whileTrue(new RetractClimber(climberMotors)); // retract climber

    //IDLE INTAKE
    new JoystickButton(driverController,1).whileTrue(new IdleIntake(intakeMotors));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // was crashing code referencing path only; had to use pathplanner Autos to create command to follow path
    //return drivebase.getAutonomousCommand("New Auto");

    //added per pathplanner docs to choose the selected auton routine from the dashboard
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }



}
