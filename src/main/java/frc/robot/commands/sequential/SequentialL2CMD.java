package frc.robot.commands.sequential;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorCMD;
import frc.robot.commands.garra.ColetarCMD;
import frc.robot.subsystems.elevador.ElevatorSUB;
import frc.robot.subsystems.garra.GarraSUB;

public class SequentialL2CMD extends SequentialCommandGroup{
    public SequentialL2CMD(ElevatorSUB elevatorSUB, GarraSUB garraSUB) {

        addCommands(
            new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.point0),
            new ColetarCMD(garraSUB, Constants.GarraSUBConstants.VELOCIDADE_COLETA),
            new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.pointL2)
        );
        
    }
}
