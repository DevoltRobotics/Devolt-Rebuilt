// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

  private SparkMax Lturret;
  private SparkMaxConfig TurretConfig = new SparkMaxConfig();

  PIDController turretPidController = new PIDController(.01, 0, 0);

  private double desiredAngle = 0;
  double TurretOut = 0;

  double turretOffsetEnc = 0;

  double turretRelativePos = 0;
  double normalizedAngle = 0;


  double Upperlimit = 90;
  double Lowerlimit = -270;


  public TurretSubsystem(int canId) {
    Lturret = new SparkMax(2, canId, MotorType.kBrushless);

    TurretConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    TurretConfig.encoder
        .positionConversionFactor(7.2);

    Lturret.configure(TurretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetOffsetCMD().schedule();
  }

  @Override
  public void periodic() {

    turretRelativePos = Lturret.getEncoder().getPosition() - turretOffsetEnc;

    if (desiredAngle > Upperlimit) {
      normalizedAngle = desiredAngle - 360;

    }else if (desiredAngle < Lowerlimit) {
      normalizedAngle = desiredAngle + 360;

    }else{
      normalizedAngle = desiredAngle;

    }

    turretPidController.setSetpoint(normalizedAngle);

    if (    normalizedAngle > Upperlimit || normalizedAngle < Lowerlimit ) {
      TurretOut = 0;

    } else if (turretPidController.calculate(turretRelativePos) < -0.7) {
      TurretOut = -0.7;
    }else if (turretPidController.calculate(turretRelativePos) > 0.7) {
      TurretOut = 0.7;
    }
     else {
      TurretOut = turretPidController.calculate(turretRelativePos);
    }

    Lturret.set(TurretOut);
    SmartDashboard.putNumber("LturretPos", Lturret.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret/TurretRelative", turretRelativePos);
  }

  public void setAngle(double angle) {
    desiredAngle = angle;
  }

  public void resetOffset(){
    turretOffsetEnc = Lturret.getEncoder().getPosition();
  }

  public Command resetOffsetCMD(){
    return new InstantCommand(()->{resetOffset();
    });
  }

  public Command SetTurretPosCMD(double Pos) {
    return new InstantCommand(() -> {
      setAngle(-Pos);
    }, this);
  }

}
