// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ElevatorSUB;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class ElevatorCMD extends Command {

double position;
ElevatorSUB elevatorSUB;

public ElevatorCMD(ElevatorSUB elevatorSUB, double position) {
    this.elevatorSUB = elevatorSUB;
    this.position = position;
    addRequirements(elevatorSUB);
}


  @Override
  public void execute() {
      elevatorSUB.moverAte(position);
  }

  
      
  @Override
  public boolean isFinished() {
  return Math.abs(
elevatorSUB.getAltura() - position) < Constants.ElevatorSUBConstants.TOLERANCIA;
 }

}
