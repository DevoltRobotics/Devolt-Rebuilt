// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANId.CAN_s1;
import frc.robot.Constants.CANId.CAN_s2;

public class TransferSubsystem extends SubsystemBase {

  private SparkMax beltMotor = new SparkMax(1, CAN_s1.TransferCan, MotorType.kBrushless);

  private TalonFX kickerMotor = new TalonFX(CAN_s2.KickerCan, CANBus.systemCore(2));
  private VelocityDutyCycle Velocity = new VelocityDutyCycle(0);

  private double desiredBeltSpeed = 0;
  private double desiredKickerSpeed = 0;



  /** Creates a new TransferSubsystem. */
  public TransferSubsystem() {}

  @Override
  public void periodic() {
    beltMotor.set(desiredBeltSpeed);
    kickerMotor.setControl(Velocity.withVelocity(desiredKickerSpeed));
  }

  public void setSpeeds(double beltSpeed, double kickerSpeed){
    desiredBeltSpeed = beltSpeed;
    desiredKickerSpeed = kickerSpeed;
  }
}
