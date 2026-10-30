// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevador;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorSUBConstants;

public class ElevatorSUB extends SubsystemBase {

    private final static SparkMax Motor1 = new SparkMax(Constants.ElevatorSUBConstants.MOTOR_ID, MotorType.kBrushless);

    private final static RelativeEncoder encoder = Motor1.getEncoder();

    private final SparkMaxConfig configMotor1 = new SparkMaxConfig();

    private final SparkClosedLoopController closed = Motor1.getClosedLoopController(); // envia setpoints

    private final Debouncer noAlvoDebouncer = new Debouncer(0.2, DebounceType.kRising);

    public enum ElevatorState {
        PARADO,
        MOVENDO,
        NO_ALVO
    }

    private ElevatorState estado = ElevatorState.PARADO;
    private double posicaoAlvo = 0.0;
    private static final double TOLERANCIA = 0.5;

    public ElevatorSUB() {

        encoder.setPosition(0);
        configMotor1.closedLoop.pid(0.02, 0.0, 0.0);
        configMotor1.closedLoopRampRate(0.6);
        configMotor1.smartCurrentLimit(40);
        configMotor1.closedLoop.outputRange(-0.5, 1);
        configMotor1.idleMode(IdleMode.kBrake);

        Motor1.configure(configMotor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // ==== GETTERS ====

    public static double getPosicao() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public ElevatorState getEstado() {
        return estado;
    }

    public static double getAltura() {
        return getPosicao(); // ja convertida em cm
    }

    // ==== SETTERS ====
    private void atualizarEstado() {
        boolean pertoDoAlvo = Math.abs(getPosicao() - posicaoAlvo) < TOLERANCIA;

        switch (estado) {

            case PARADO:
                break;

            case MOVENDO:
                if (noAlvoDebouncer.calculate(pertoDoAlvo)) {
                    estado = ElevatorState.NO_ALVO;
                }
                break;

            case NO_ALVO:
                if (!pertoDoAlvo) { // algo empurrou o elevador para fora
                    estado = ElevatorState.MOVENDO;
                }
                break;
        }
    }

    public void setPosition(double posicao) {

        posicaoAlvo = MathUtil.clamp(posicao,
                Constants.ElevatorSUBConstants.min_position,
                Constants.ElevatorSUBConstants.max_position);

        noAlvoDebouncer.calculate(false); // começa a contagem do zero
        estado = ElevatorState.MOVENDO;
        closed.setSetpoint(posicaoAlvo, ControlType.kPosition);
    }

    // ==== PERIODIC =====
    @Override
    public void periodic() {
        atualizarEstado();
        Logger.recordOutput("Elevator/Posicao", getPosicao());
        Logger.recordOutput("Elevator/Velocidade", getVelocity());
        Logger.recordOutput("Elevator/Altura", getAltura());
        Logger.recordOutput("Elevator/Output", Motor1.getAppliedOutput());
        Logger.recordOutput("Elevator/Estado", estado.name());

    }
}