package frc.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class AlignToTarget extends Command {
private final TurretSubsystem  turret;

public AlignToTarget(TurretSubsystem turret){
    this.turret = turret;
}

public void execute(){
turret.setTurn();
}


}
