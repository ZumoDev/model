package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class Paths {
    final String rotationPidPath = "SmartDashboard/Swerve/RotationPID/";
    final String intakePidPath = "SmartDashboard/Intake/PID/";
    final String hudPidPath = "SmartDashboard/Shooter/AnglePID/";
}

public class DashboardSubsystem extends SubsystemBase{
    Paths paths = new Paths();


    public DashboardSubsystem() {
        InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
    }

    public void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public void sendPidData() {
        SmartDashboard.putNumber(paths.rotationPidPath+"kP", Constants.Limelights.PID.Rotation.kP);
        SmartDashboard.putNumber(paths.rotationPidPath+"kI", Constants.Limelights.PID.Rotation.kI);
        SmartDashboard.putNumber(paths.rotationPidPath+"kD", Constants.Limelights.PID.Rotation.kD);

        SmartDashboard.putNumber(paths.intakePidPath+"kP", Constants.Mechanisms.Intake.PID.kP);
        SmartDashboard.putNumber(paths.intakePidPath+"kI", Constants.Mechanisms.Intake.PID.kI);
        SmartDashboard.putNumber(paths.intakePidPath+"kD", Constants.Mechanisms.Intake.PID.kD);

        SmartDashboard.putNumber(paths.hudPidPath+"kP", Constants.Mechanisms.Hud.PID.kP);
        SmartDashboard.putNumber(paths.hudPidPath+"kI", Constants.Mechanisms.Hud.PID.kI);
        SmartDashboard.putNumber(paths.hudPidPath+"kD", Constants.Mechanisms.Hud.PID.kD);
    }

    public double getPidData(String mechanism, String key) {
        String path; 
        switch (mechanism) {
            case "swerve":
                path = paths.rotationPidPath;
                break;
            case "intake":
                path = paths.intakePidPath;
                break;
            case "hud":
                path = paths.hudPidPath;
                break;
            default:
                path = "";
        }

        return SmartDashboard.getNumber(path+key, 0);
    }

    public void setTreeMapData() {
        
    }
}
