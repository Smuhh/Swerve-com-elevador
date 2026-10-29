// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSUB extends SubsystemBase {

    private final SparkMax Motor1 = new SparkMax(Constants.ElevatorSUBConstants.MOTOR1_ELEVADOR,MotorType.kBrushless);

    private final RelativeEncoder encoder = Motor1.getEncoder();

    private final SparkMaxConfig configMotor1 = new SparkMaxConfig();

    private final SparkClosedLoopController closed = Motor1.getClosedLoopController(); //envia setpoints


    public ElevatorSUB() {


    encoder.setPosition(0);
    configMotor1.closedLoop.pid(0.040, 0.0, 0.0);
    configMotor1.closedLoopRampRate(0.4);
    configMotor1.smartCurrentLimit(40);
    configMotor1.closedLoop.outputRange(-0.2, 0.2);
    configMotor1.idleMode(IdleMode.kBrake);


    configMotor1.encoder.positionConversionFactor(Constants.ElevatorSUBConstants.CM_POR_ROTACAO_MOTOR);

    Motor1.configure(configMotor1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
   }
   

    public double getPosicao() {
        return encoder.getPosition();
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void subir(double position) {
        SmartDashboard.putNumber("Elevator/AlvoAtual", position);
        closed.setSetpoint(position,ControlType.kPosition);
}
    

    public double getAltura() {
        return getPosicao(); 
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("posi", getPosicao());
        SmartDashboard.putNumber("velocidade", getVelocity());
        SmartDashboard.putNumber("alturacm", getAltura());
        SmartDashboard.putNumber("Elevator/Output", Motor1.getAppliedOutput());
    }
}