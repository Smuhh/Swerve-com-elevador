package frc.robot.commands.Garra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.garra.GarraSUB;

public class EjetarCMD extends Command {

    private final GarraSUB garra;
    private final double rpm;

    public EjetarCMD(GarraSUB garra, double rpm) {
        this.garra = garra;
        this.rpm = rpm;

        addRequirements(garra);
    }

    @Override
    public void initialize() {
        garra.ejetar(rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        garra.parar();
    }
}