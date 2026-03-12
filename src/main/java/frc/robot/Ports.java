package frc.robot;

// TODO: verificar todos los IDs con electrónica antes de la primera prueba

/**
 * IDs de todos los dispositivos de hardware del robot.
 *
 * Dos buses CAN:
 *   - "canivore": bus rápido dedicado. Se usa para el swerve y el giroscopio
 *     porque necesitan leer posición 250 veces por segundo.
 *   - "rio": bus normal del RoboRIO. Suficiente para mecanismos como intake e indexer.
 */
public final class Ports {

    private Ports() {}

    // =========================================================================
    // CANivore — bus CAN FD dedicado, nombre de red: "canivore"
    // =========================================================================

    /** Nombre del bus CANivore tal como aparece en Phoenix Tuner X. */
    public static final String CANIVORE_BUS = "canivore";

    public static final class Swerve {

        // --- Módulo Frontal Izquierdo (FL) ---
        //Aztech
        /** TalonFX que controla la tracción FL; recibe VelocityVoltage/VoltageOut. */
        public static final int FL_DRIVE_ID = 1;

        /** TalonFX que controla el giro FL; usa el CANcoder para control automático. */
        public static final int FL_STEER_ID = 2;

        /**
         * El CANcoder persiste la posición absoluta al encender → no se necesita
         * rutina de homing.
         */
        public static final int FL_CANCODER_ID = 3;

        // --- Módulo Frontal Derecho (FR) ---

        /** TalonFX tracción FR. */
        public static final int FR_DRIVE_ID = 4;

        /** TalonFX giro FR. */
        public static final int FR_STEER_ID = 5;

        /** CANcoder absoluto FR. */
        public static final int FR_CANCODER_ID = 6;

        // --- Módulo Trasero Izquierdo (BL) ---

        /** TalonFX tracción BL. */
        public static final int BL_DRIVE_ID = 7;

        /** TalonFX giro BL. */
        public static final int BL_STEER_ID = 8;

        /** CANcoder absoluto BL. */
        public static final int BL_CANCODER_ID = 9;

        // --- Módulo Trasero Derecho (BR) ---

        /** TalonFX tracción BR. */
        public static final int BR_DRIVE_ID = 10;

        /** TalonFX giro BR. */
        public static final int BR_STEER_ID = 11;

        /** CANcoder absoluto BR. */
        public static final int BR_CANCODER_ID = 12;
    }

    /**
     * Giroscopio de 9 ejes (Pigeon2). Vive en el CANivore para que Phoenix6
     * pueda leer el ángulo del robot directamente en la odometría del swerve.
     */
    public static final int PIGEON2_ID = 13;

    // =========================================================================
    // RoboRIO CAN — bus CAN nativo, nombre de red: "rio"
    // =========================================================================

    /** Nombre del bus RoboRIO; en Phoenix6 se pasa como string vacío o "rio". */
    public static final String RIO_BUS = "rio";

    public static final class Shooter {

        /**
         * TalonFX de la rueda disparadora izquierda.
         * Se configura desde el código o desde Tuner X.
         */
        public static final int LEFT_ID = 20;

        /** TalonFX rueda disparadora derecha. Generalmente configurado como Follower. */
        public static final int RIGHT_ID = 21;
    }

    public static final class Hood {

        /**
         * TalonFX (Kraken X44 mini) lado izquierdo del hood (ajuste de ángulo de disparo).
         * Se configura igual que cualquier TalonFX estándar.
         */
        public static final int LEFT_ID = 22;

        /** TalonFX (Kraken mini) lado derecho del hood. */
        public static final int RIGHT_ID = 23;
    }

    /**
     * TalonFX del indexer: lleva la nota desde la intake hasta las ruedas de disparo.
     * Está en el bus rio porque no necesita lecturas de posición rápidas.
     */
    public static final int INDEXER_ID = 24;

    public static final class Intake {

        /**
         * SPARK MAX izquierdo (motor NEO de REV).
         * Se configura con SparkMaxConfig, no con Phoenix Tuner X.
         * Para diagnóstico usar REV Hardware Client.
         */
        public static final int LEFT_SPARK_ID = 18;

        /** SPARK MAX derecho de la intake. */
        public static final int RIGHT_SPARK_ID = 15;

        public static final int ROLLER_ID = 28;

    }
}
