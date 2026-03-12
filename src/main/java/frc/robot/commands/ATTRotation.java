package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ATTRotation extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    private final PIDController limelightPID;
    private final double maxAngularRate;
    private String limelightName = Constants.Limelights.shooterLimelightName;

    double tx = LimelightHelpers.getTX(limelightName);
    double rotationOutput = 0.0;

    public ATTRotation(CommandSwerveDrivetrain drivetrain,
                           SwerveRequest.FieldCentric driveRequest,
                           PIDController limelightPID,
                           double maxAngularRate,
                           String limelightName) {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.limelightPID = limelightPID;
        this.maxAngularRate = maxAngularRate;
        this.limelightName = limelightName;
        //addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV(limelightName)) {
            rotationOutput = limelightPID.calculate(tx, Constants.Limelights.AprilTagLimits.XError);
            rotationOutput = Math.max(Math.min(rotationOutput, maxAngularRate), -maxAngularRate);

            //System.out.println("tx: " + tx + " | rotOutput: " + rotationOutput);
        } 

        driveRequest
        .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationOutput)
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

        drivetrain.setControl(driveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(driveRequest.withVelocityX(0.0)
                                                  .withVelocityY(0.0)
                                                  .withRotationalRate(0.0));
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}