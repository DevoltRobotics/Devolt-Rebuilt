// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX flywheel;
  private SparkMax turret;

  private VelocityVoltage Velocity = new VelocityVoltage(0);

 


  private double desiredVelocity;
  private double desiredAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);


  PIDController turretPidController = new PIDController(.01, 0, 0);
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(TalonFX flywheelMotor, SparkMax turretMotor, Pose2d positionInRobot ) {
    this.flywheel = flywheelMotor;
    this.turret = turretMotor;

    

  }

  @Override
  public void periodic() {
    flywheel.setControl(Velocity.withVelocity(desiredVelocity));
    turretPidController.setSetpoint(desiredAngle);
    turret.set(turretPidController.calculate(turret.getEncoder().getPosition()));
  }

  public void setVelocity(double velocity){
    desiredVelocity = velocity;
  }
  public void setAngle(double angle){
    desiredAngle = angle;
  }
}
