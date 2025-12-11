package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivotS;
import frc.robot.subsystems.IntakePivotS.intakeConstants;
import frc.robot.subsystems.RollersS;
import frc.robot.subsystems.RollersS.rollerConstants;

public class StateMachine {
    // TODO: add logging/simulation for states

    private final IntakePivotS intakePivot;
    private final RollersS rollers;
    private final CommandSwerveDrivetrain drivetrain;
    private Autos autos;

    public enum RobotState {
        // Todo: add all states as in button mapping doc
        CORAL_PRE_SCORE,
        ALGAE_PRE_SCORE,
        DEFAULT
    }

    public RobotState currentState = RobotState.DEFAULT;

    // Constructor that accepts dependencies
    public StateMachine(IntakePivotS intakePivot, RollersS rollers, CommandSwerveDrivetrain drivetrain) {
        this.intakePivot = intakePivot;
        this.rollers = rollers;
        this.drivetrain = drivetrain;
        this.autos = null; // Will be set later
    }

    // Setter for autos (to be called after RobotContainer creates it)
    public void setAutos(Autos autos) {
        this.autos = autos;
    }

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
        return Commands.parallel(intakePivot.setAngle(intakeConstants.ALGAE_INTAKE),
                rollers.setVoltage(rollerConstants.INTAKE_VOLTAGE));
    }

    public Command AlgaeStow() {
        return Commands.parallel(intakePivot.setAngle(intakeConstants.STOW),
                rollers.setVoltage(rollerConstants.INTAKE_VOLTAGE),
                setState(RobotState.ALGAE_PRE_SCORE));
    }

    public Command AutoallignCoral() {
        if (autos == null) {
            return Commands.none();
        }
        
        return Commands.sequence(
                autos.createAutoAlignCommand(new Pose2d(ChoreoVariables.getPose("Lolipop1").getX(),
                        ChoreoVariables.getPose("Lolipop1").getY(), ChoreoVariables.getPose("Lolipop1").getRotation())),
                setState(RobotState.CORAL_PRE_SCORE));
    }

    public Command AutoallignProcessor() {
        if (autos == null) {
            return Commands.none();
        }
        
        return Commands.sequence(
                autos.createAutoAlignCommand(new Pose2d(ChoreoVariables.getPose("Processor").getX(),
                        ChoreoVariables.getPose("Processor").getY(),
                        ChoreoVariables.getPose("Processor").getRotation())),
                setState(RobotState.ALGAE_PRE_SCORE));
    }

    // Functions below:
    public Command ScoreAlgae() {
        return Commands.parallel(intakePivot.setAngle(intakeConstants.ALGAE_INTAKE),
                rollers.setVoltage(rollerConstants.OUTTAKE_VOLTAGE)).withTimeout(0.3)
                .andThen(intakePivot.setAngle(intakeConstants.ALGAE_POST_SCORE)
                        .withTimeout(0.5)
                        .andThen(intakePivot.setAngle(intakeConstants.STOW))
                        .andThen(setState(RobotState.DEFAULT)));
                
    }

    public Command L1Score() {
        return Commands.parallel(intakePivot.setAngle(intakeConstants.STOW),
                rollers.setVoltage(rollerConstants.OUTTAKE_VOLTAGE));
    }
}