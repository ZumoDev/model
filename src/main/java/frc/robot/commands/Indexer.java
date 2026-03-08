package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class Indexer extends Command{
    private final double speed;
    private final IntakeSubsystem sub;
    
    public Indexer(IntakeSubsystem sub, double speed) {
        this.speed = speed;
        this.sub = sub;
    }

    @Override
    public void execute() {
        sub.setIndexerSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        sub.setIndexerSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
