// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class MagazineSubsystem extends SubsystemBase {

  private final CANSparkMax m_magazine_motor = new CANSparkMax(Constants.MotorConstants.MAGAZINE_MOTOR_CAN_ID, MotorType.kBrushless);

  /** Creates a new MagazineSubsystem. */
  public MagazineSubsystem() {
    m_magazine_motor.restoreFactoryDefaults();
    m_magazine_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_magazine_motor.setSmartCurrentLimit(Constants.MotorConstants.MAGAZINE_MOTOR_CURRENT_LIMIT);
    m_magazine_motor.burnFlash();

    System.out.println("*** magazine motor controllers configured.");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMagazineMotor() {
      m_magazine_motor.set(Constants.MotorConstants.MAGAZINE_INTAKE_SPEED);
    }

  public void setMagazineMotor(double setSpeed){
    m_magazine_motor.set(setSpeed);
  }

    public void stopMagazineMotor(){
      m_magazine_motor.set(0);
    }


}
