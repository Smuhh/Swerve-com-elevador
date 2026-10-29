package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperSUB extends SubsystemBase {
    
    SparkMaxConfig config = new SparkMaxConfig();

    SparkMax Motor_Do_Gripper_1 = new SparkMax(Constants.GripperConstants.MOTOR2_GRIPPER, MotorType.kBrushless);

    public GripperSUB() {

        config.openLoopRampRate(1);
        config.smartCurrentLimit(40);
        config.closedLoop.outputRange(-0.4, 0.4);

        Motor_Do_Gripper_1.configure(
            config, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kNoPersistParameters

        );
}
    public void puxar() {
        Motor_Do_Gripper_1.set(0.4);
    }

    public void expulsar() {
        Motor_Do_Gripper_1.set(-0.4);
    }

    public void stop() {
        Motor_Do_Gripper_1.stopMotor();
    }
   
 

}

