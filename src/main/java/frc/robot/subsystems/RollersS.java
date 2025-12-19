package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class RollersS extends SubsystemBase {

    public static class rollerConstants {

        public static final Distance OFFSET_X = Inches.of(5.6);
        public static final Distance OFFSET_Y = Inches.of(0);
        public static final Distance OFFSET_Z = Inches.of(4);

        public static final int ROLLER_CAN_ID = 41;

        public static final AngularVelocity OUTTAKE_VELOCITY = RPM.of(70);
        public static final AngularVelocity INTAKE_VELOCITY = RPM.of(-70);
        public static final Voltage INTAKE_VOLTAGE = Volts.of(-6);
        public static final Voltage OUTTAKE_VOLTAGE = Volts.of(6);

        public static final Current STATOR_LIMIT = Amps.of(50);
    }

    private SmartMotorControllerConfig rollerMotorConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            // PID
            .withClosedLoopController(30, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            .withSimClosedLoopController(30, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
            // Feedforward
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withSimFeedforward(new SimpleMotorFeedforward(0, 0.3, 0.1))
            // telemetry/simulation
            .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
            // gear ratio between motor and final shaft
            .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(4, 1)))
            // motor properties
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(rollerConstants.STATOR_LIMIT)
            .withClosedLoopRampRate(Seconds.of(0.25));

    // TalonFX motor controller object
    private TalonFX rollerMotor = new TalonFX(rollerConstants.ROLLER_CAN_ID);

    // create SmartMotorController with the TalonFX
    private SmartMotorController talonFXSmartMotorController = new TalonFXWrapper(
            rollerMotor, DCMotor.getFalcon500(1), rollerMotorConfig);

    // config object for the rollers, using a ShooterConfig object but not
    // specifying all the values that don't matter for a basic roller
    private FlyWheelConfig rollerConfig = new FlyWheelConfig(talonFXSmartMotorController)
            .withDiameter(Inches.of(2))
            .withMass(Pounds.of(0.6))
            .withTelemetry("IntakeRollerMech", TelemetryVerbosity.HIGH);

    // using a Shooter object for our roller, since the mechanisms function
    // similarly
    private FlyWheel roller = new FlyWheel(rollerConfig);

    public RollersS() {
        setDefaultCommand(setVoltage(Volts.of(0.0)));
    }

    public Command setVoltage(Voltage volts) {
        return run(() -> rollerMotor.setVoltage(volts.in(Volts)));
    }

    public Command intakeRollersStart() {
        return setVoltage(rollerConstants.INTAKE_VOLTAGE)
                .withTimeout(0.15);

    }

    public Command intakeRollersUntilStop() {
        return setVoltage(rollerConstants.INTAKE_VOLTAGE)
                .until(() -> rollerMotor.getStatorCurrent().getValueAsDouble() > rollerConstants.STATOR_LIMIT
                        .in(Amps));
    }

    public Command outTakeRollers() {
        return setVoltage(rollerConstants.OUTTAKE_VOLTAGE);
    }

    public Command coralIntake() {
        return Commands.sequence(intakeRollersStart(), intakeRollersUntilStop());
    }

    @Override
    public void periodic() {
        roller.updateTelemetry();

    }

    @Override
    public void simulationPeriodic() {
        roller.simIterate();
    }

}