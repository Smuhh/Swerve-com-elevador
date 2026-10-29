    package frc.robot.commands;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.GripperSUB;

    public class GripperCMD extends Command {

        GripperSUB grippersub;
        boolean puxar;

        public GripperCMD (GripperSUB gripperSUB,boolean puxar) {
        this.grippersub = gripperSUB;
        this.puxar = puxar;
        addRequirements(gripperSUB);}

        @Override
        public void execute() {
        if(puxar){
        grippersub.puxar();}

        else{grippersub.expulsar();}

        
        }
        @Override
        public void end(boolean interrupted) {
            grippersub.stop();
        }
    }
