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
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Ports;
import frc.robot.Constants.Mechanisms;

public class ShooterSubsystem extends SubsystemBase {
    // Ruedas de disparo — TalonFX en bus RIO. rsMotor es el principal; lsMotor sigue como seguidor.
    // public final requerido: StartShooter.java accede a rsMotor directamente para setRPM().
    public final TalonFX rsMotor = new TalonFX(Ports.Shooter.RIGHT_ID, Ports.RIO_BUS);
    public final TalonFX lsMotor = new TalonFX(Ports.Shooter.LEFT_ID,  Ports.RIO_BUS);

    // HOOD DISABLED - motor inicializado en CAN pero sin comandos
    public final TalonFX twMotor = new TalonFX(Ports.Hood.LEFT_ID,  Ports.RIO_BUS);
    // HOOD DISABLED - motor inicializado en CAN pero sin comandos
    public final TalonFX bwMotor = new TalonFX(Ports.Hood.RIGHT_ID, Ports.RIO_BUS);

    // VelocityVoltage: control automático de velocidad para las ruedas de disparo.
    private final VelocityVoltage   velVol     = new VelocityVoltage(0).withSlot(0);
    // VoltageOut: control directo por voltaje, sin retroalimentación.
    private final VoltageOut        volOut     = new VoltageOut(0);

    // Tabla de interpolación distancia→RPM. Puede editarse desde SmartDashboard (ShooterTable/).
    private final InterpolatingDoubleTreeMap itm = new InterpolatingDoubleTreeMap();

    // Último setpoint de RPM enviado al shooter (publicado en SmartDashboard como TargetRPM).
    private double targetRPM = 0;

    // Solo el motor principal del shooter; el seguidor no recibe comandos directos.
    // isVelocityWithinTolerance() solo verifica el motor principal.
    private final List<TalonFX> shooterMotors;

    // Todos los motores para stop() — detiene ruedas y hood.
    private final List<TalonFX> allMotors;

    public ShooterSubsystem() {
        shooterMotors = List.of(rsMotor);
        allMotors     = List.of(rsMotor, lsMotor, twMotor, bwMotor);

        // Ruedas de disparo: configurar solo el motor principal; el seguidor hereda el comportamiento.
        configureMotor(rsMotor, InvertedValue.Clockwise_Positive);
        lsMotor.setControl(new Follower(rsMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // Hood: configurar motor principal; el seguidor gira en sentido opuesto por simetría.
        //configureHoodMotor(twMotor, InvertedValue.Clockwise_Positive);
        //bwMotor.setControl(new Follower(twMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // Arrancar en estado neutro para evitar comandos de velocidad residuales del arranque
        rsMotor.stopMotor();
        lsMotor.stopMotor();
    }
    //Metodo par obtener la tempertura más alta
    /*public double getShooterTemp(){
    double bwMotorTemp=bwMotor.getDeviceTemp().getValueAsDouble();
    double twMotorTemp=twMotor.getDeviceTemp().getValueAsDouble();
    double lsMotorTemp=lsMotor.getDeviceTemp().getValueAsDouble();
    double rsMotorTemp=rsMotor.getDeviceTemp().getValueAsDouble();
    return Math.max(Math.max(lsMotorTemp, rsMotorTemp),Math.max(bwMotorTemp, twMotorTemp));
    }*/

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
                    .withKP(0.0)  
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKV(0.0) 
                    .withKS(0.0) 
                    .withKG(0)
                    //.withGravityType(GravityTypeValue.Arm_Cosine)
            );
        
        motor.getConfigurator().apply(config);
    }

    /** Detiene todos los motores (ruedas + hood). */
    public void stop() {
        for (final TalonFX motor : allMotors) motor.stopMotor();
    }

    /** Control directo por voltaje. Útil para pruebas manuales. */
    public void setPercentOutput(double percentOutput, TalonFX motor) {
        motor.setControl(volOut.withOutput(Volts.of(percentOutput * 12)));
    }

    /** Gira las ruedas a un porcentaje de voltaje directo. */
    public void set(double speed) {
        rsMotor.set(speed);
    }

    /** HOOD DISABLED - no envía comandos al motor. */
    public void setWristSpeed(double speed) {
        // HOOD DISABLED - motor inicializado en CAN pero sin comandos
    }

    /**
     * Control automático de velocidad para las ruedas de disparo.
     * Se pasa el motor explícitamente para controlar solo uno si es necesario.
     */
    public void setRPM(double rpm, TalonFX motor) {
        motor.setControl(velVol.withVelocity(RPM.of(rpm)));
    }

    /** HOOD DISABLED - no envía comandos al motor. */
    public void setWristPosition(double position) {
        // HOOD DISABLED - motor inicializado en CAN pero sin comandos
    }

    /** Posición actual del rotor de twMotor en rotaciones. */
    public double getWristPosition() {
        return twMotor.getPosition().getValueAsDouble();
    }

    //mejor voy a usar valores de -1.0 a 1.0, en lugar de rpm, porque rpm no funciona
    public double calculateRPMWithITM(String limelightName, double distance) {
        final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

        // table.put(1.5, 2500.0);
        // table.put(2.5, 3200.0);
        // table.put(4.0, 4100.0);
        
        table.put(0.0 , 0.0); 

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
        return shooterMotors.stream().allMatch(motor -> {
            final boolean isInVelocityMode = motor.getAppliedControl().equals(velVol);
            final AngularVelocity currentVelocity = motor.getVelocity().getValue();
            final AngularVelocity targetVelocity = velVol.getVelocityMeasure();
            return isInVelocityMode && currentVelocity.isNear(targetVelocity, RPM.of(100));
        });
    }

    public double getShooterTemp() {
        return allMotors.stream()
            .mapToDouble(m -> m.getDeviceTemp().getValueAsDouble())
            .max()
            .orElse(0.0);
    }
}