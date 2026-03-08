package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StartIntake extends Command {
    private final IntakeSubsystem sub;
    private final double speed;

    public StartIntake(IntakeSubsystem sub, double speed) {
        this.sub = sub;
        this.speed = speed;
    }

    @Override
    public void execute() {
        sub.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        sub.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
