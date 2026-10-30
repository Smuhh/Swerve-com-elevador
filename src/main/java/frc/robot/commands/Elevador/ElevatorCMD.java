
package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevador.ElevatorSUB;
import frc.robot.subsystems.elevador.ElevatorSUB.ElevatorState;

public class ElevatorCMD extends Command {

    private final ElevatorSUB elevador;
    private final double posicao;

    public ElevatorCMD(ElevatorSUB elevador, double posicao) {
        this.elevador = elevador;
        this.posicao = posicao;

        addRequirements(elevador);
    }

    @Override
    public void initialize() {
        elevador.setPosition(posicao);
    }

    @Override
    public boolean isFinished() {
        return elevador.getEstado() == ElevatorState.NO_ALVO;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
