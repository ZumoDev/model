package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Mechanisms;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX bwMotor = new TalonFX(1);
    public final TalonFX twMotor = new TalonFX(5);
    public final TalonFX rsMotor = new TalonFX(1);
    public final TalonFX lsMotor = new TalonFX(2);

    private final List<TalonFX> motors;
    private final VelocityVoltage velVol = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut volOut = new VoltageOut(0);
    private final PositionVoltage posvol = new PositionVoltage(0);

    public ShooterSubsystem() {
        motors = List.of(bwMotor, twMotor, rsMotor);

        configureMotor(bwMotor, InvertedValue.CounterClockwise_Positive);
        configureMotor(twMotor, InvertedValue.Clockwise_Positive);
        //configureMotor(rsMotor, InvertedValue.CounterClockwise_Positive);

        lsMotor.setControl(new Follower(rsMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        bwMotor.setControl(new Follower(twMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }
    //Metodo par obtener la tempertura más alta
    public double getShooterTemp(){
    double bwMotorTemp= bwMotor.getDeviceTemp().getValueAsDouble();
    double twMotorTemp=twMotor.getDeviceTemp().getValueAsDouble();
    double lsMotorTemp=lsMotor.getDeviceTemp().getValueAsDouble();
    double rsMotorTemp=rsMotor.getDeviceTemp().getValueAsDouble();
    return Math.max(Math.max(lsMotorTemp, rsMotorTemp),Math.max(bwMotorTemp, twMotorTemp));
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(invertDirection)
                    .withNeutralMode(NeutralModeValue.Coast)
            )
            .withVoltage(
                new VoltageConfigs()
                    .withPeakReverseVoltage(Volts.of(0))
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.11)  
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.12) 
                    .withKS(0.2) 
                    .withKG(1)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
            );
        
        motor.getConfigurator().apply(config);
    }

    public void stop() {
        for (final TalonFX motor : motors) motor.stopMotor();
    }

    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    public void set(double speed) {
        rsMotor.set(speed);
    }

    public void setWristSpeed(double speed) {
        twMotor.set(speed);
        //bwMotor.set(speed);
    }

    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }   

    public void setWristPosition(double position) {
        twMotor.setControl(posvol.withPosition(position));
    }

    public double getWristPosition() {
        return twMotor.getPosition().getValueAsDouble();
    }

    public double calculateRPMWithITM(String limelightName, double distance) {
        final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

        // AQUÍ DEBES LLENAR TUS PUNTOS EMPÍRICOS (Distancia en metros, RPM)
        // table.put(1.5, 2500.0);
        // table.put(2.5, 3200.0);
        // table.put(4.0, 4100.0);
        
        table.put(0.0 , 0.0); // Borra esto cuando pongas los reales

        return table.get(distance);
    }

    public double calculateShooterRPM(
        String limelightName, 
        double cameraHeight,
        double cameraMountingAngle,
        double hubHeight
        ) {
            double targetAngle = LimelightHelpers.getTY(limelightName);
            double shooterRPM = 0.0;
            
            double distanceToTarget = (1.8288 - cameraHeight) / Math.tan((cameraMountingAngle + targetAngle) * (Math.PI / 180.0));

            double velocity = Math.sqrt(
                (9.81 * (distanceToTarget * distanceToTarget))
                /
                (2 * Math.pow(Math.cos(Mechanisms.shooterMountingAngle), 2) 
                * (distanceToTarget * Math.tan(Mechanisms.shooterMountingAngle) - (hubHeight - Mechanisms.shooterMountingHeight)))
            );

            shooterRPM = (velocity / (2 * Math.PI * Mechanisms.shooterWheelRadius) * 60);
            return shooterRPM;
    }

    public boolean isVelocityWithinTolerance() {
        return motors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velVol);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velVol.getVelocityMeasure();
            // Aumenté un poco la tolerancia a 150 RPM, 100 a veces es muy estricto para FRC y hace stutter el comando de disparar
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, RPM.of(150)); 
        });
    }
}