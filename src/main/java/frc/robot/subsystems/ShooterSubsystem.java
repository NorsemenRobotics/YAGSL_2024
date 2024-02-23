// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class ShooterSubsystem extends SubsystemBase {
private final CANSparkMax m_leftShooter_motor = new CANSparkMax(Constants.MotorConstants.LEFT_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
private final CANSparkMax m_rightShooter_motor = new CANSparkMax(Constants.MotorConstants.RIGHT_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);

private SparkPIDController m_right_pidController;
private SparkPIDController m_left_pidController;
//private RelativeEncoder m_right_encoder;
//private RelativeEncoder m_left_encoder;

CommandJoystick shooterController = new CommandJoystick(Constants.OperatorConstants.SHOOTER_USB_PORT);


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    m_leftShooter_motor.restoreFactoryDefaults();
    m_leftShooter_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_leftShooter_motor.setSmartCurrentLimit(Constants.MotorConstants.SHOOTER_MOTOR_CURRENT_LIMIT);
    m_leftShooter_motor.burnFlash();

    Timer.delay(0.2);

    m_rightShooter_motor.restoreFactoryDefaults();
    m_rightShooter_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightShooter_motor.setSmartCurrentLimit(Constants.MotorConstants.SHOOTER_MOTOR_CURRENT_LIMIT);
    m_rightShooter_motor.burnFlash();

     Timer.delay(0.2);
/**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */

    m_left_pidController = m_leftShooter_motor.getPIDController();
    m_right_pidController = m_rightShooter_motor.getPIDController();

    // Encoder object created to display position values
    //m_right_encoder = m_rightShooter_motor.getEncoder();
    //m_left_encoder = m_leftShooter_motor.getEncoder();

    m_right_pidController.setP(Constants.ShooterPIDConstants.SHOOTER_PID_kP);
    m_right_pidController.setI(Constants.ShooterPIDConstants.SHOOTER_PID_kI);
    m_right_pidController.setD(Constants.ShooterPIDConstants.SHOOTER_PID_kD);
    m_right_pidController.setIZone(Constants.ShooterPIDConstants.SHOOTER_PID_kIz);
    m_right_pidController.setFF(Constants.ShooterPIDConstants.SHOOTER_PID_kFF);
    m_right_pidController.setOutputRange(Constants.ShooterPIDConstants.SHOOTER_PID_kMinOutput, Constants.ShooterPIDConstants.SHOOTER_PID_kMaxOutput);

    m_left_pidController.setP(Constants.ShooterPIDConstants.SHOOTER_PID_kP);
    m_left_pidController.setI(Constants.ShooterPIDConstants.SHOOTER_PID_kI);
    m_left_pidController.setD(Constants.ShooterPIDConstants.SHOOTER_PID_kD);
    m_left_pidController.setIZone(Constants.ShooterPIDConstants.SHOOTER_PID_kIz);
    m_left_pidController.setFF(Constants.ShooterPIDConstants.SHOOTER_PID_kFF); // inverted?
    m_left_pidController.setOutputRange(Constants.ShooterPIDConstants.SHOOTER_PID_kMinOutput, Constants.ShooterPIDConstants.SHOOTER_PID_kMaxOutput);





    System.out.println("*** shooter motor controllers configured.");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooterMotors(double shooter_motor_speed) {
    m_leftShooter_motor.set(-shooter_motor_speed);
    m_rightShooter_motor.set(shooter_motor_speed);
  }

  public void stopShooterMotors() {
    m_leftShooter_motor.set(0);
    m_rightShooter_motor.set(0);
  }

  public void runShooterMotorsRPM(double setPointRPM) {
    m_left_pidController.setReference(-setPointRPM, CANSparkMax.ControlType.kVelocity);
    m_right_pidController.setReference(setPointRPM, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooterMotorsRPM() {
    m_left_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
    m_right_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }

  public void runShooterMotorsVoltage(double voltage) {
    m_leftShooter_motor.setVoltage(-voltage);
    m_rightShooter_motor.setVoltage(voltage);
  }

  public void stopShooterMotorsVoltage() {
    m_leftShooter_motor.setVoltage(0);
    m_rightShooter_motor.setVoltage(0);
  }
}
