// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSUB extends SubsystemBase {

    private final SparkMax Motor1 = new SparkMax(6,MotorType.kBrushless);

    private final RelativeEncoder encoder = Motor1.getEncoder();

    private final SparkClosedLoopController closed = Motor1.getClosedLoopController(); //envia setpoints
    
    double TargetHeight;

    public ElevatorSUB() {

   SparkMaxConfig configMotor1 = new SparkMaxConfig();



    configMotor1.closedLoop.pid(0.02, 0.0, 0.0);

    configMotor1.closedLoop.maxMotion.cruiseVelocity(20).  //ir ajustando 
    maxAcceleration(10).allowedProfileError(1);

    configMotor1.smartCurrentLimit(40);
    configMotor1.closedLoop.outputRange(-0.5, 1.0);
    configMotor1.idleMode(IdleMode.kBrake);
    configMotor1.closedLoopRampRate(0.5);


    configMotor1.encoder.positionConversionFactor(Constants.ElevatorSUBConstants.CM_POR_ROTACAO_MOTOR);

    Motor1.configure(configMotor1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    encoder.setPosition(Constants.ElevatorSUBConstants.positionMotor);

   }
   

    public double getPosicao() {
        return encoder.getPosition();
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void moverAte(double position) {
       double alvo = MathUtil.clamp(position, 9.5, 73);
       TargetHeight = alvo;
       closed.setSetpoint(alvo, ControlType.kPosition);
        
}

    public double getTargetHeight(){
        return TargetHeight;
    }

    public double getAltura() {
          return encoder.getPosition();
            
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("posi", getAltura());
        SmartDashboard.putNumber("Posicao boladona kdsksk()", getAltura());
        SmartDashboard.putNumber("velocidade", getVelocity());
        SmartDashboard.putNumber("Elevator/Output", Motor1.getAppliedOutput());
    }
}