package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.therekrab.autopilot.APTarget;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import frc.robot.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.units.measure.Distance;

import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakePivotS.intakeConstants;
import frc.robot.subsystems.RollersS;
import frc.robot.subsystems.RollersS.rollerConstants;

public class StateMachine {
    // TODO: add logging/simulation for states

    public final IntakePivotS yIntakePivot = new IntakePivotS();
    public final RollersS rollers = new RollersS();
    public final Autos Autos = new Autos(null, yIntakePivot, null, null, null);

    public enum RobotState {
        // Todo: add all states as in button mapping doc
        CORAL_PRE_SCORE,
        ALGAE_PRE_SCORE,
        DEFAULT

    }

    public RobotState currentState = RobotState.DEFAULT;

    public Command setState(RobotState newState) {
        return new InstantCommand(() -> currentState = newState);
    }

    
    // Commands below:
    public Command Score() {
        if (currentState == RobotState.CORAL_PRE_SCORE) {
            return L1Score();
        }
        if (currentState == RobotState.ALGAE_PRE_SCORE) {
            return ScoreAlgae();
        } else {
            return Commands.none();
            // Do nothing
        }
    }

    public Command AlgaeIntake() {
        return Commands.parallel(yIntakePivot.setAngle(intakeConstants.ALGAE_INTAKE),
                rollers.setVoltage(rollerConstants.INTAKE_VOLTAGE));
    }

    public Command AlgaeStow() {
        return Commands.parallel(yIntakePivot.setAngle(intakeConstants.STOW),
                rollers.setVoltage(rollerConstants.INTAKE_VOLTAGE),
                setState(RobotState.ALGAE_PRE_SCORE));
    }

    public Command AutoallignCoral() {
        return Commands.sequence(
                Autos.createAutoAlignCommand(new Pose2d(ChoreoVariables.getPose("Lolipop1").getX(),
                        ChoreoVariables.getPose("Lolipop1").getY(), ChoreoVariables.getPose("Lolipop1").getRotation())),
                setState(RobotState.CORAL_PRE_SCORE));

    }

    public Command AutoallignProcessor() {
        return Commands.sequence(
                Autos.createAutoAlignCommand(new Pose2d(ChoreoVariables.getPose("Processor").getX(),
                        ChoreoVariables.getPose("Processor").getY(),
                        ChoreoVariables.getPose("Processor").getRotation())),
                setState(RobotState.ALGAE_PRE_SCORE));

    }


    // Functions below:

    public Command ScoreAlgae() {
        return Commands.parallel(yIntakePivot.setAngle(intakeConstants.STOW),
                rollers.setVoltage(rollerConstants.OUTTAKE_VOLTAGE)).withTimeout(0.3)
                .andThen(yIntakePivot.setAngle(intakeConstants.ALGAE_POST_SCORE)
                        .withTimeout(0.5)
                        .andThen(yIntakePivot.setAngle(intakeConstants.STOW))
                        .andThen(setState(RobotState.DEFAULT)));
    }

    public Command L1Score() {
        return Commands.parallel(yIntakePivot.setAngle(intakeConstants.STOW),
                rollers.setVoltage(rollerConstants.OUTTAKE_VOLTAGE));

    }

}

// Commands below:
// TODO: add handoff sequence
