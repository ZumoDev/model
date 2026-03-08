package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class WristSpeed extends Command {
    private final ShooterSubsystem sub;
    private final double speed;

    //Constructi
    public WristSpeed(ShooterSubsystem sub, double speed) {
        this.sub = sub;
        this.speed = speed;
        addRequirements(sub);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        sub.setWristSpeed(speed);
        System.out.print("pos: " + sub.getWristPosition());
    }

    @Override
    public void end(boolean interrupted) {
        sub.setWristSpeed(0.02);
    }

    @Override
    public boolean isFinished() {
        return false;

        /*if (sub.getWristPosition() <= 5 && ) return false;
        else if (sub.getWristPosition() >= 0.2) return true;
        else return false;*/
    }
}
