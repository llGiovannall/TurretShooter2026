package frc.robot;

import frc.Commands.AlignToTarget;
import frc.robot.subsystems.TurretSubsystem;

import java.security.DrbgParameters;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final ShooterSubsystem shooterSubsystem = new  ShooterSubsystem();
 private final CommandXboxController m_driverController = new CommandXboxController(0);
   

      


  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {

    this.m_driverController.a().whileTrue(
     Commands.run(() -> turretSubsystem.rotateLeft(), turretSubsystem)
    );

    

    //this.m_driverController.a().whileTrue(Commands.run(() -> turretSubsystem.setTurn()).until(() -> turretSubsystem.atTarget()));

 this.m_driverController.b().onTrue(
  Commands.run(() -> turretSubsystem.Stop(), turretSubsystem)
 );

 //this.m_driverController.a().whileTrue(
 //new AlignToTarget(turretSubsystem));

 this.m_driverController.x().whileTrue(Commands.run(() -> shooterSubsystem.setShotAngle()));



this.m_driverController.y().whileTrue(Commands.run(() -> shooterSubsystem.angle()));
  }





  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
  }
}

