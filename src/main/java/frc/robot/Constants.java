package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    
    public class IDs {
        
        public class Shooter {
            public static final int bottomShootingMotorId = 0;
            public static final int topShootingMotorId = 0;
            public static final int leftWristMotorId = 0;
            public static final int rightWristMotorId = 0;
        }

        public class Intake {
            public static final int intakeMotorId = 0;
            public static final int leftWristMotorId = 0;
            public static final int rightWristMotorId = 0;
        }
    }

    public class Limelights {

        public static final String shooterLimelightName = "limelight";
        public static final String extraLimelightName = "limelight2";

        public class AprilTagLimits {
            public static final double XError = 0.0;
            public static final double ZError = 2;
            public static final double yawError = 0.0;
            public static final double ZDeadband = 0.05; //cm
        }

        public class PID {

            public class Rotation {
                public static double kP = 0.1;
                public static double kI = 0.0;
                public static double kD = 0.0;
            }

            public class X {
                public static final double kI = 0.1;
                public static final double kP = 4.75;
                public static final double kD = 0.1;
            }

            public class Z {
                public static final double kP = 2.5;
                public static final double kI = 0.0;
                public static final double kD = 0.1;
            }
        }
    }

    public class Mechanisms {
        public static double shooterMountingAngle = 50.0; //degrees
        public static final double shooterMountingHeight = 0; //meters
        public static final double shooterWheelRadius = 0; //meters

        public class Intake {
            public class PID {
                public static double kP = 0.0;
                public static double kI = 0.0;
                public static double kD = 0.0;
            }
        }

        public class Hud {
            public class PID {
                public static double kP = 0.0;
                public static double kI = 0.0;
                public static double kD = 0.0;
            }
        }
    }

    public class FieldConstants {
        public static final Pose2d startPose = new Pose2d(
            0.0,
            0.0,
            new Rotation2d(0)
        );
    }

    public class AprilTags {
        public class RedAlliance {
            public static final int frontHubId = 10;
            public static final int frontSideHubId = 9;
            public static final int bottomHubId = 2;
            public static final int bottomSideHubId = 11;
            public static final int topHubId = 5;
            public static final int topSideHubId = 8;
        }

        public class BlueAlliance {
            public static final int frontHubId = 25;
            public static final int frontSideHubId = 26;
            public static final int bottomHubId = 21;
            public static final int bottomSideHubId = 24;
            public static final int topHubId = 18;
            public static final int topSideHubId = 27;
        }
    }

    public static final class Shooter {
        /** TODO: RPM para disparo — punto de partida sugerido 3000, ajustar por distancia al HUB */
        public static final double SHOOT_RPM = 5000.0;
        /** TODO: rotaciones del rotor para angulo de disparo — medir con Tuner X en robot */
        public static final double WRIST_ANGLE_SHOOT = 5.0;
        /** TODO: posicion de reposo — debe ser 0 si el encoder se resetea al iniciar */
        public static final double WRIST_ANGLE_HOME = 0.0;
        /** velocidad del feeder al alimentar — 0.75 es punto de partida conservador */
        public static final double FEEDER_SPEED = 0.75;
    }

    public class Vars {
        static boolean isShooterAiming = true;
    }
}
