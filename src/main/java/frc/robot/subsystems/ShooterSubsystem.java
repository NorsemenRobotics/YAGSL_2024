// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSubsystem extends SubsystemBase {
private final CANSparkMax m_leftShooter_motor = new CANSparkMax(Constants.MotorConstants.LEFT_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
private final CANSparkMax m_rightShooter_motor = new CANSparkMax(Constants.MotorConstants.RIGHT_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);

CommandJoystick shooterController = new CommandJoystick(Constants.OperatorConstants.SHOOTER_USB_PORT);
private double joystickMotorControlInput;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    m_leftShooter_motor.restoreFactoryDefaults();
    m_leftShooter_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_leftShooter_motor.setSmartCurrentLimit(Constants.MotorConstants.SHOOTER_MOTOR_CURRENT_LIMIT);
    m_leftShooter_motor.burnFlash();

    m_rightShooter_motor.restoreFactoryDefaults();
    m_rightShooter_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightShooter_motor.setSmartCurrentLimit(Constants.MotorConstants.SHOOTER_MOTOR_CURRENT_LIMIT);
    m_rightShooter_motor.burnFlash();

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

}
