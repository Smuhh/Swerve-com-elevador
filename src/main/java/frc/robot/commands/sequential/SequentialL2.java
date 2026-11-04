package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSUB;
import frc.robot.subsystems.GripperSUB;
import frc.robot.commands.ElevatorCMD;
import frc.robot.commands.GripperCMD;

public class SequentialL2 extends SequentialCommandGroup{

    public SequentialL2(ElevatorSUB elevatorSUB, GripperSUB gripperSUB) {
      addCommands(new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.pointL1),
                  new GripperCMD(gripperSUB, -1200),
                  new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.pointL2));
    }   
    
}
