package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSUB extends SubsystemBase {
    

    SparkMax Motor_Do_Gripper_1 = new SparkMax(7, MotorType.kBrushless);
    SparkClosedLoopController closed =  Motor_Do_Gripper_1.getClosedLoopController();
    RelativeEncoder encoder1 = Motor_Do_Gripper_1.getEncoder();

    public GripperSUB() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0.00014, 0, 0);
        config.closedLoop.feedForward.kV(0.0025);
        config.closedLoopRampRate(0.70);
        config.smartCurrentLimit(40);
        config.closedLoop.outputRange(-0.4, 0.4);

        Motor_Do_Gripper_1.configure(
            config, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters
        );

}

        public enum estadogripper{
         AGUARDANDO,
         COLETANDO,
         PECA_DETECTADA,
         EXPULSANDO}

        Timer timerColeta = new Timer();
        estadogripper estadoatual = estadogripper.AGUARDANDO;

        public void puxar() {
       
        closed.setSetpoint(Constants.GripperConstants.velcolt,ControlType.kVelocity);
    }

    public void iniciarColeta() {
    if (estadoatual == estadogripper.AGUARDANDO) {
        estadoatual = estadogripper.COLETANDO;
    }
    }
    public void iniciarExpulsao() {
    if (estadoatual == estadogripper.PECA_DETECTADA) {
        estadoatual = estadogripper.EXPULSANDO;
    }
}

    public void finalizarExpulsao() {
    if (estadoatual == estadogripper.EXPULSANDO) {
        estadoatual = estadogripper.AGUARDANDO;
        Motor_Do_Gripper_1.stopMotor();
    }
}

    public double getVelo(){
    return encoder1.getVelocity();} 


    public void soltar() {
        closed.setSetpoint(Constants.GripperConstants.velexp, ControlType.kVelocity);
    }

    public double getCorrente(){
    return Motor_Do_Gripper_1.getOutputCurrent();
    }

    public estadogripper getEstado() {
    return estadoatual;
    }
    public void parar(){
        Motor_Do_Gripper_1.stopMotor();
    }
   
 @Override
 public void periodic() {
     switch (estadoatual) {
        case AGUARDANDO:     
            Motor_Do_Gripper_1.stopMotor();
        break;

        case COLETANDO :
            puxar();
            double correnteatual = getCorrente();
            if(correnteatual >= 30){
                timerColeta.start();
            if(timerColeta.get() >= 0.2){
                estadoatual = estadogripper.PECA_DETECTADA;
                Motor_Do_Gripper_1.stopMotor();}
            }

            else{timerColeta.reset();
            timerColeta.stop();}

            break;

            case PECA_DETECTADA : Motor_Do_Gripper_1.stopMotor();
            timerColeta.stop();
            timerColeta.reset();

            break;

             case EXPULSANDO : soltar();
             break;
     
             default:
             break;
     }
    SmartDashboard.putString("GripperEstado", estadoatual.toString());
    SmartDashboard.putNumber("velocidadegrap", getVelo());
    SmartDashboard.putNumber("corrente", getCorrente());
 }

}

