package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSUB;
import frc.robot.subsystems.GripperSUB;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCMD;
import frc.robot.commands.GripperCMD;

public class SequentialL3 extends SequentialCommandGroup{

  public SequentialL3(ElevatorSUB elevatorSUB, GripperSUB grippersub){
    addCommands(new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.pointL1),
                new GripperCMD(grippersub, -1200).withTimeout(6),
                new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.pointL3).onlyIf(() -> gripperSUB.getEstado() == GripperSUB.estadogripper.PECA_DETECTADA));

  }
}
