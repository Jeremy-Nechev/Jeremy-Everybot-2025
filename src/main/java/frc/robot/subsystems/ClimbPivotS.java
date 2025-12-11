
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.TunerConstants;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ClimbPivotS extends SubsystemBase {
  public class climbConstants {
    public static final Distance OFFSET_X = Inches.of(-4);
    public static final Distance OFFSET_Y = Inches.of(0);
    public static final Distance OFFSET_Z = Inches.of(2);

    public static final Angle CW_LIMIT = Degrees.of(80);
    public static final Angle CCW_LIMIT = Degrees.of(10);

    public static final Angle STOW = Degrees.of(80);
    public static final Angle PIVOT_DOWN = Degrees.of(10);

    public static final double KP = 18;
    public static final double KI = 0;
    public static final double KD = 0.2;
    public static final double KS = -0.1;
    public static final double KG = 1.2;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double VELOCITY = 458;
    public static final double ACCELERATION = 688;
    public static final int MOTOR_ID = 50;
    public static final double STATOR_CURRENT_LIMIT = 120;
    public static final double MOI = 0.0855457256;
  }

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(climbConstants.KP, climbConstants.KI, climbConstants.KD,
          DegreesPerSecond.of(climbConstants.VELOCITY), DegreesPerSecondPerSecond.of(climbConstants.ACCELERATION))
      // can be seperate for sim:
      .withSimClosedLoopController(climbConstants.KP, climbConstants.KI, climbConstants.KD,
          DegreesPerSecond.of(climbConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(climbConstants.ACCELERATION))
      // Feedforward Constants
      .withFeedforward(
          new ArmFeedforward(climbConstants.KS, climbConstants.KG, climbConstants.KV, climbConstants.KA))
      .withSimFeedforward(
          new ArmFeedforward(climbConstants.KS, climbConstants.KG, climbConstants.KV, climbConstants.KA))
      // Telemetry name and verbosity level
      .withTelemetry("Climb", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which
      // corresponds to the gearbox attached to your motor.
      .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(100, 1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(climbConstants.STATOR_CURRENT_LIMIT));

  // Vendor motor controller object
  private TalonFX climbPivotMotor = new TalonFX(climbConstants.MOTOR_ID, TunerConstants.kCANBus2);

  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController climbSMC = new TalonFXWrapper(climbPivotMotor, DCMotor.getKrakenX60(1), smcConfig);

  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withRelativePosition(
          new Translation3d(climbConstants.OFFSET_X, climbConstants.OFFSET_Y, climbConstants.OFFSET_Z));

  private ArmConfig climbCfg = new ArmConfig(climbSMC)
      // Soft limit is applied to the SmartMotorControllers PID

      .withHardLimit(climbConstants.CW_LIMIT, climbConstants.CCW_LIMIT)
      // Starting position is where your arm starts
      .withStartingPosition(climbConstants.STOW)

      // Length and mass of your arm for sim.
      .withLength(Feet.of((12 / 12)))

      .withMOI(climbConstants.MOI)

      // Telemetry name and verbosity for the arm.
      .withTelemetry("Climb", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(robotToMechanism);

  // Arm Mechanism
  private Arm climb = new Arm(climbCfg);

  /**
   * Set the angle of the arm.
   * 
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {

    return climb.setAngle(angle);
  }

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return climb.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climb.updateTelemetry();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    climb.simIterate();
  }

  public Angle getAngle() {
    return climb.getAngle();
  }
}