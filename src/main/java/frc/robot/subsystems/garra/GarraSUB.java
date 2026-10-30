package frc.robot.subsystems.garra;

import java.security.Key;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GarraSUB extends SubsystemBase {

    private final SparkMax m_garra = new SparkMax(7, MotorType.kBrushless);
    private final RelativeEncoder encoder = m_garra.getEncoder();
    private final SparkClosedLoopController closedLoop = m_garra.getClosedLoopController();
    private final SparkMaxConfig config = new SparkMaxConfig();

    public enum GarraState {
        PARADA,
        COLETANDO,
        EJETANDO,
        COLETADA
    }

    private GarraState estado = GarraState.PARADA;


    private static final double CORRENTE_PECA = 30; // amperes

    private final Debouncer pecaDebouncer = new Debouncer(0.25, DebounceType.kRising);

    public GarraSUB() {
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        config.closedLoop.pid(0.0001, 0.0, 0.0);
        config.closedLoop.outputRange(-1, 1);

        config.closedLoop.feedForward.kV(0.0022);

        config.closedLoop.maxMotion
                .maxAcceleration(400);

        m_garra.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    
    public GarraState getEstado() {
       return estado;
    }

    public void setVelocity(double rpm) {
        double velocidadeAlvo = rpm;
        closedLoop.setSetpoint(velocidadeAlvo, ControlType.kMAXMotionVelocityControl);
    }

    public void coletar(double rpm) {
        pecaDebouncer.calculate(false); // zera a contagem
        estado = GarraState.COLETANDO;
        setVelocity(rpm);
    }

    public void ejetar(double rpm) {
        estado = GarraState.EJETANDO;
        setVelocity(rpm);
    }

    public void parar() {
      estado = GarraState.PARADA;
      m_garra.stopMotor();
    }

    private void atualizarEstado() {
        switch (estado) {

            case PARADA:
                break;

            case COLETANDO:
                // peça entrou: o rolo fica pesado e a corrente sobe
                if (pecaDebouncer.calculate(m_garra.getOutputCurrent() > CORRENTE_PECA)) {
                    estado = GarraState.COLETADA;
                    m_garra.stopMotor(); // neutro: o idleMode Brake segura o rolo
                }
                break;

            case EJETANDO:
                break;

            case COLETADA:
                break;
        }
    }

    @Override
    public void periodic() {
        atualizarEstado();
        Logger.recordOutput("Garra/Estado", estado.name());
        Logger.recordOutput("Garra/Corrente", m_garra.getOutputCurrent());
        Logger.recordOutput("Garra/Velocity",getVelocity());
    }

}
