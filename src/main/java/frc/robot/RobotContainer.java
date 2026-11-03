// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.garra.ColetarCMD;
import frc.robot.commands.garra.EjetarCMD;
import frc.robot.commands.elevator.ElevatorCMD;
import frc.robot.commands.sequential.SequentialL3CMD;
import frc.robot.commands.sequential.SequentialL2CMD;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.elevador.ElevatorSUB;
import frc.robot.subsystems.garra.GarraSUB;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  final CommandXboxController co_piloto = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  // Elevator
  private final ElevatorSUB elevado = new ElevatorSUB();
  private final GarraSUB garra = new GarraSUB();

  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveDirectAngle = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerHeadingAxis(driverXbox::getRightX,
          driverXbox::getRightY)
      .headingWhile(true)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Converte driver para movimentação de Angulação controlada
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY() * 0.4, // Alterar a força de translação Y do robô
      () -> -driverXbox.getLeftX() * 0.4) // Alterar a força de translação X do robô
      .withControllerRotationAxis(() -> -driverXbox.getRightX() * 0.4) // Altarar a força de rotação do robô
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST")); // Registro do Command no Autônomo
    autoChooser = AutoBuilder.buildAutoChooser(); // Responsável por entregar as opções de Autos do path
    SmartDashboard.putData("Auto Chooser", autoChooser); // Responsável por escolher as opções de Autos do path
  }

  private void configureBindings() {
    // Command do Drive Angular Velocity
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Definir o comando padrão para o drive

    // Command para zerar o giroscópio
    driverXbox.rightBumper().onTrue(Commands.runOnce(drivebase::zeroGyro));

    driverXbox.x().onTrue(Comandos.ElevatorCMD(elevado, Constants.ElevatorSUBConstants.point0));
    // co_piloto.a().onTrue(Comandos.ElevatorCMD(elevado, Constants.ElevatorSUBConstants.pointL1));
    driverXbox.a().onTrue(Comandos.SequentialL3CMD(elevado, garra));
    driverXbox.b().onTrue(Comandos.SequentialL2CMD(elevado, garra));
    driverXbox.y().onTrue(Comandos.ElevatorCMD(elevado, 0));
    driverXbox.a().onTrue(new SequentialL2CMD(elevado, garra));

    driverXbox.rightTrigger().toggleOnTrue(Comandos.EjetarCMD(garra, Constants.GarraSUBConstants.VELOCIDADE_COLETA));
    driverXbox.leftTrigger().toggleOnTrue(Comandos.ColetarCMD(garra, Constants.GarraSUBConstants.VELOCIDADE_EJECAO));

  }


  //Commands
  public static class Comandos {
    public static Command ElevatorCMD(ElevatorSUB elevatorSUB, double position) {
      return new ElevatorCMD(elevatorSUB, position);
    }

    public static Command ColetarCMD(GarraSUB garra, double rpm) {
      return new ColetarCMD(garra, rpm);
    }

    public static Command EjetarCMD(GarraSUB garra, double rpm) {
      return new EjetarCMD(garra, rpm);
    }

    public static Command SequentialL3CMD(ElevatorSUB elevatorSUB, GarraSUB garraSUB) {
      return new SequentialL3CMD(elevatorSUB, garraSUB);

    }

    public static Command SequentialL2CMD(ElevatorSUB elevatorSUB, GarraSUB garraSUB) {
      return new SequentialL2CMD(elevatorSUB, garraSUB);
    }
  }



  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public Command alignSwerveWithAngle(double angleInDegrees) {
    Rotation2d rotationToAlign = Rotation2d.fromDegrees(angleInDegrees);
    SwerveInputStream alignWithAngles = driveDirectAngle.copy().withControllerHeadingAxis(
        rotationToAlign::getCos,
        rotationToAlign::getSin);
    return drivebase.driveFieldOriented(alignWithAngles);
  }
}
