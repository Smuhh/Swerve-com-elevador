    package frc.robot.commands;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.GripperSUB;
    import frc.robot.Constants;

    public class GripperCMD extends Command {

        GripperSUB grippersub;
        double velcolt;

        public GripperCMD (GripperSUB gripperSUB,double velocidade) {
        this.grippersub = gripperSUB;
        this.velcolt = velocidade;
        addRequirements(gripperSUB);}

        @Override
        public void initialize() {
        if (velcolt == Constants.GripperConstants.velcolt) {
        grippersub.iniciarColeta();} 
        else if (velcolt == Constants.GripperConstants.velexp) {
        grippersub.iniciarExpulsao();}}

        @Override
       public boolean isFinished() {
       return velcolt == Constants.GripperConstants.velcolt && grippersub.getEstado() == GripperSUB.estadogripper.PECA_DETECTADA;}

        @Override
       public void end(boolean interrupted) {
       if (velcolt == Constants.GripperConstants.velexp) {
        grippersub.finalizarExpulsao();}}
    }
