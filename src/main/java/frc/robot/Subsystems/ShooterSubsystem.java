// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANId.CAN_s2;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX Lflywheel =  new TalonFX(CAN_s2.LFlywheelCan, CANBus.systemCore(2));
  private SparkMax Lturret = new SparkMax(2, CAN_s2.LTurretCan, MotorType.kBrushless);

   private final TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();
  private SparkMaxConfig TurretConfig = new SparkMaxConfig();

  private VelocityVoltage Velocity = new VelocityVoltage(0);


  InterpolatingDoubleTreeMap rpsMap = new InterpolatingDoubleTreeMap();
  double flywheelRadius = 0.3; //meters

  Translation2d LtargetVector = new Translation2d();
  double Ldistance;
  double SOTFLturretLAngle;
  double SOTFLrequiredRSpeed;

  double Goal_X_Red = 0;
  double Goal_Y_Red = 0;

  double Goal_X_Blue = 0;
  double Goal_Y_Blue = 0;

  double Goal_X = 0;
  double Goal_Y = 0;

  double TurretL_X = 0;
  double TurretL_Y = 0;

  Pose2d positionInRobot = new Pose2d();
  ChassisSpeeds Speed = new ChassisSpeeds();

  Translation2d LturretOffset = new Translation2d(0.165, 0.152);
  Translation2d RturretOffset = new Translation2d(0.165, -0.152);



  private double desiredVelocity;
  private double desiredAngle;

  double TurretOut = 0;



  PIDController turretPidController = new PIDController(.01, 0, 0);
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(Pose2d positionInRobot,ChassisSpeeds Speed) {


    if ( DriverStation.getAlliance().get() == Alliance.Red ) {
      Goal_X = Goal_X_Red;
      Goal_Y = Goal_Y_Red;
    }else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      Goal_X = Goal_X_Blue;
      Goal_Y = Goal_Y_Blue;
    }else{
      Goal_X = Goal_X_Red;
      Goal_Y = Goal_Y_Red;
    }

    rpsMap.put(1.0, 20d); // 1 metro → 20 rps
    rpsMap.put(20.0, 40d);
    rpsMap.put(38.0, 80d);

    
    TurretConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    TurretConfig.encoder
      .positionConversionFactor(7.2);
      
      

      Lturret.configure(TurretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    FlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    FlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    FlywheelConfig.CurrentLimits.SupplyCurrentLimit = 35;
    FlywheelConfig.Slot0.kP = 0.1;
    FlywheelConfig.Slot0.kV = 0.11;
    FlywheelConfig.Slot0.kS = 0.2;
    Lflywheel.getConfigurator().apply(FlywheelConfig);

    this.positionInRobot = positionInRobot;
    this.Speed = Speed;
  }

  @Override
  public void periodic() {
Translation2d LturretFieldPosition = positionInRobot.getTranslation().plus( LturretOffset.rotateBy(positionInRobot.getRotation()));

 TurretL_X = LturretFieldPosition.getX();
 TurretL_Y = LturretFieldPosition.getY();

  double targetTurretL_X = Goal_X - TurretL_X;
  double targetTurretL_Y = Goal_Y - TurretL_Y;
  Translation2d LtargetPosition = new Translation2d(targetTurretL_X, targetTurretL_Y);

  // Calculate the ideal exit velocity magnitude (based on distance)
   Ldistance = LtargetPosition.getNorm();
   double LidealSpeed = getShooterSpeedForDistance(Ldistance);

   LtargetVector = LtargetPosition.div(Ldistance).times(LidealSpeed);

   double vel_x = Speed.vx;
   double vel_y = Speed.vy;

  Translation2d robotVelocity = new Translation2d(vel_x, vel_y); // Field centric velocity!
  
  Translation2d LshotVector = LtargetVector.minus(robotVelocity);

    SOTFLturretLAngle = LshotVector.getAngle().getDegrees();
    SOTFLrequiredRSpeed = LshotVector.getNorm(); // en M/S


    Lflywheel.setControl(Velocity.withVelocity(desiredVelocity));
    turretPidController.setSetpoint(desiredAngle);

    if (turretPidController.calculate(Lturret.getEncoder().getPosition()) > 0.7) {
      TurretOut = 0.7;
    }else if (turretPidController.calculate(Lturret.getEncoder().getPosition()) < -0.7) {
      TurretOut = -0.7;
    }else{
      TurretOut = turretPidController.calculate(Lturret.getEncoder().getPosition());
    }

    Lturret.set(TurretOut);
    SmartDashboard.putNumber("LturretPos", Lturret.getEncoder().getPosition());

  }

  public void setVelocity(double velocity){
    desiredVelocity = velocity;
  }
  public void setAngle(double angle){
    desiredAngle = -angle;
  }

  public Command SetTurretPosCMD(double Pos){
      return new InstantCommand(()->{ setAngle(Pos);},this);
  }
  public Command SetVelocityCMD(double Vel){
    return new InstantCommand(()->{ setVelocity(Vel);});
  }

  public Command ShootToTargetCMD(){
    return new RunCommand(()->{
      setAngle(LtargetVector.getAngle().getDegrees());
      setVelocity(rpsMap.get(Ldistance));
    });
  }

  public Command SOTFCMD(){
    return new RunCommand(()->{
      double rps = SOTFLrequiredRSpeed/(2*Math.PI*flywheelRadius);

      setAngle(SOTFLturretLAngle);
      setVelocity(rps);
    });
  }

  //velocidad de pelota en M/S
  public double getShooterSpeedForDistance(double distance){

    double rps = rpsMap.get(distance);
    double speed = rps * 2 * Math.PI * flywheelRadius; //in m/s
    return speed;
  }

}

