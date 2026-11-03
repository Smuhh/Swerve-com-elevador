    package frc.robot.commands;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.GripperSUB;

    public class GripperCMD extends Command {

        GripperSUB grippersub;
        boolean puxar;
        double velcolt;

        public GripperCMD (GripperSUB gripperSUB,double velcolt) {
        this.grippersub = gripperSUB;
        this.velcolt = velcolt;
        addRequirements(gripperSUB);}

        @Override
        public void execute() {
        if(puxar){
        grippersub.puxar();}

        else{grippersub.soltar();}

        
        }
        @Override
        public void end(boolean interrupted) {
            grippersub.parar();
        }
    }
