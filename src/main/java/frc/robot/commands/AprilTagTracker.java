/*
 * PLAN:
 * - Estimar distancia del AprilTag con Limelight "d = (h2-h1) / tan(a1+a2)"
 * - Calcular a q potencia debera disparar el Intake dependiendo de "d"
 * 
 */

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AprilTagTracker extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric driveRequest;
    private final PIDController xPID;
    private final PIDController zPID;
    private final PIDController turnPID;
    private final double maxAngularRate;
    private final double maxSpeed;
    private final String limelightName; 

    public AprilTagTracker(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRequest, PIDController xPID, PIDController zPID, PIDController turnPID, double maxAngularRate, double maxSpeed, String limelightName) {
        this.drivetrain = drivetrain;
        this.driveRequest = driveRequest;
        this.xPID = xPID;
        this.zPID = zPID;
        this.turnPID = turnPID;
        this.maxAngularRate = maxAngularRate;
        this.maxSpeed = maxSpeed;
        this.limelightName = limelightName;

        addRequirements(drivetrain); 
    }

    @Override
    public void execute() {
        //declarando variables
        double[] targetLocation = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
        double tx = targetLocation[0];
        double tz = targetLocation[2];
        double yaw = LimelightHelpers.getTX(limelightName);

        LimelightHelpers.setPipelineIndex(limelightName, 1);
        SwerveRequest.RobotCentric newRequest;
        
        if (LimelightHelpers.getTV(limelightName)) {
            double xOutput = xPID.calculate(tx, Constants.Limelights.AprilTagLimits.XError);
            double zOutput = zPID.calculate(tz, Constants.Limelights.AprilTagLimits.ZError);
            double rotationOutput = 0.0;

            xOutput = -Math.max(Math.min(xOutput, maxSpeed), -maxSpeed);
            zOutput = Math.max(Math.min(zOutput, maxSpeed), -maxSpeed);

            if (Math.abs(tz - Constants.Limelights.AprilTagLimits.ZError) < 0.1) {
                //zOutput = 0.0;
                rotationOutput = turnPID.calculate(yaw, Constants.Limelights.AprilTagLimits.yawError);
                rotationOutput = Math.max(Math.min(rotationOutput, maxAngularRate), -maxAngularRate);
            }

            System.out.println("tx: " + tx + " | xOutput:" + xOutput);
            System.out.println("tz: " + tz + " | zOutput:" + zOutput);
            System.out.println("yaw: " + yaw + " | rotOutput: " + rotationOutput);

            newRequest = driveRequest.withVelocityX(zOutput)
                                     .withVelocityY(xOutput)
                                     .withRotationalRate(rotationOutput);
        } else {
            newRequest = driveRequest.withVelocityX(0.0)
                                     .withVelocityY(0.0)
                                     .withRotationalRate(0.0);
        }

        drivetrain.setControl(newRequest);
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
