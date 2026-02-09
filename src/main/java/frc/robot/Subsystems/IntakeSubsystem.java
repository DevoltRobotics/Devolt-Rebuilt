// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANId.CAN_s1;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX rollerMotor = new TalonFX(CAN_s1.IntakeCan, CANBus.systemCore(1));
  private SparkMax pivot = new SparkMax(1, CAN_s1.PivotCan, MotorType.kBrushless);

  private double desiredPosition;
  private double desiredRollerSpeed;

  private PIDController pivotPID = new PIDController(.01, 0, 0);
  private VelocityDutyCycle RollerDutyCycle;

  private SparkMaxConfig pivotConfig = new SparkMaxConfig();

  private AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();
  
  
  public IntakeSubsystem() {
    pivotConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    pivotConfig.absoluteEncoder
      .positionConversionFactor(1);
      

      pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
    

  }

  @Override
  public void periodic() {
    pivotPID.setSetpoint(desiredPosition);
    pivot.set(pivotPID.calculate(pivotEncoder.getPosition()));

    rollerMotor.setControl(RollerDutyCycle.withVelocity(desiredRollerSpeed));


  }

  public void setRollerSpeed(double speed){
    desiredRollerSpeed = speed;
  }

  public void setPosition(double position){
    desiredPosition = position;
  }
}
