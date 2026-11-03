package frc.robot.commands.garra;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.garra.GarraSUB;
import frc.robot.subsystems.garra.GarraSUB.GarraState;

public class ColetarCMD extends Command {

    private final GarraSUB garra;
    private final double rpm;

    public ColetarCMD(GarraSUB garra, double rpm) {
        this.garra = garra;
        this.rpm = rpm;

        addRequirements(garra);
    }

    @Override
    public void initialize() {
        garra.coletar(rpm);
    }

    @Override
    public boolean isFinished() {
        return garra.getEstado() == GarraState.COLETADA;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            garra.parar();
        }
    }
}