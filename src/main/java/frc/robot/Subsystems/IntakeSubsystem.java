// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANId.CAN_s1;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX rollerMotor = new TalonFX(CAN_s1.IntakeCan, CANBus.systemCore(1));
  private SparkMax pivot = new SparkMax(1, CAN_s1.PivotCan, MotorType.kBrushless);

  private double desiredPosition;


  private PIDController pivotPID = new PIDController(.01, 0, 0);
  private DutyCycleOut RollerDutyCycle = new DutyCycleOut(0);

   private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
  private SparkMaxConfig pivotConfig = new SparkMaxConfig();

  private AbsoluteEncoder pivotEncoder = pivot.getAbsoluteEncoder();

  private double PivotOut = 0;

  public double intakeUpPos = 0;

  public double intakeDownPos = 120;

  double tolerance = 3;
  
  
  public IntakeSubsystem() {
    pivotConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake);
    pivotConfig.absoluteEncoder
      .positionConversionFactor(1);
      

      pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 35;

    rollerMotor.getConfigurator().apply(rollerConfig);
      
    

  }

  @Override
  public void periodic() {
    pivotPID.setSetpoint(desiredPosition);

    PivotOut = pivotPID.calculate(pivotEncoder.getPosition());

    pivot.set(PivotOut);
  }

  public void setRollerSpeed(double desiredRollerSpeed){
    rollerMotor.setControl(RollerDutyCycle.withOutput(desiredRollerSpeed));

  }

  public void setPosition(double position){
    desiredPosition = position;
  }

  public class SetPivotPosCMD extends Command{

    private final IntakeSubsystem intakeSubsystem;
    double targetPos;

    boolean FinishOnpoint = false;

    public SetPivotPosCMD(IntakeSubsystem intakeSubsystem,Double targetPos,boolean FinishOnpoint){

      this.intakeSubsystem = intakeSubsystem;
      this.targetPos = targetPos;
      this.FinishOnpoint = FinishOnpoint;

    }

    @Override
    public void initialize(){
        intakeSubsystem.setPosition(targetPos);
    }

    @Override
    public boolean isFinished() {
      if(FinishOnpoint){
        return (intakeSubsystem.pivotPID.getError() < Math.abs(intakeSubsystem.tolerance));
      }else{
        return true;
      }
    }
    
  }

  public Command IntakeinCMD(IntakeSubsystem intakeSubsystem){
      return new InstantCommand(()->{intakeSubsystem.setRollerSpeed(1);});
  }

  public Command IntakeOutCMD(IntakeSubsystem intakeSubsystem){
    return new InstantCommand(()->{intakeSubsystem.setRollerSpeed(-1);});
}

public Command IntakeStopCMD(IntakeSubsystem intakeSubsystem){
  return new InstantCommand(()->{intakeSubsystem.setRollerSpeed(0);});
}

}
