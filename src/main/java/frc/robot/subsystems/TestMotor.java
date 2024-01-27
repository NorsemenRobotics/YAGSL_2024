// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class TestMotor extends SubsystemBase {
  
    // create new m_test_motor
    private final CANSparkMax m_test_motor = new CANSparkMax(Constants.TestMotor.TEST_MOTOR_CAN_ID, MotorType.kBrushless);
    
 

// I am not sure what this TestMotor() {} does (it's empty...?), but it is present in the wpilib documentation so I duplicated it.
// Maybe it is there to placehold for a call to require this whole class in a command?
public TestMotor() {} 

  public Command initializeMotor() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return this.runOnce(
            () -> m_test_motor.restoreFactoryDefaults())
          .andThen(run(
            () -> m_test_motor.setIdleMode(CANSparkMax.IdleMode.kCoast)))
            .withName("Test Motor Configure");
       }
  
  public Command runMotor() {
    return this.run(
      () -> m_test_motor.set(0.5))
      .finallyDo(interrupted -> m_test_motor.set(0))
      .withName("Test Motor On");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
