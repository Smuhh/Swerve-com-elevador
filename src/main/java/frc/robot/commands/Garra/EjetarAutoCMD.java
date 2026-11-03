package frc.robot.commands.garra;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.garra.GarraSUB;

public class EjetarAutoCMD extends Command {

    private final GarraSUB garra;
    private final double rpm;
    private final double tempo;
    private final Timer timer = new Timer();

    public EjetarAutoCMD(GarraSUB garra, double rpm, double tempo) {
        this.garra = garra;
        this.rpm = rpm;
        this.tempo = tempo;

        addRequirements(garra);
    }

    @Override
    public void initialize() {
        timer.restart();
        garra.ejetar(rpm);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(tempo);
    }

    @Override
    public void end(boolean interrupted) {
        garra.parar();
        timer.stop();
    }
}