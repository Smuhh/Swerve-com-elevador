package frc.robot.commands.sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorCMD;
import frc.robot.commands.garra.ColetarCMD;
import frc.robot.commands.garra.EjetarAutoCMD;
import frc.robot.subsystems.elevador.ElevatorSUB;
import frc.robot.subsystems.garra.GarraSUB;

public class SequentialL3CMD extends SequentialCommandGroup {

    public SequentialL3CMD(ElevatorSUB elevatorSUB, GarraSUB garraSUB) {
        addCommands(
            new ElevatorCMD(elevatorSUB, Constants.ElevatorSUBConstants.point0),
            new ColetarCMD(garraSUB,Constants.GarraSUBConstants.VELOCIDADE_COLETA),
            new ElevatorCMD(elevatorSUB,Constants.ElevatorSUBConstants.pointL3));
    }
}