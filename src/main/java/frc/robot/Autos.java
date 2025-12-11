package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.therekrab.autopilot.APTarget;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakePivotS;

public class Autos {
    private final AutoFactory m_factory;
    protected final CommandSwerveDrivetrain m_drivebase;
    protected final IntakePivotS m_intakepiv;
    private final StateMachine m_stateMachine;
    private final double SCORE_WAIT = 0.875;

    public Autos(CommandSwerveDrivetrain drivebase, IntakePivotS intakepiv,
            AutoFactory factory, RobotContainer container, StateMachine stateMachine) {
        m_drivebase = drivebase;
        m_intakepiv = intakepiv;
        m_factory = factory;
        m_stateMachine = stateMachine;

        container.m_chooser.addRoutine(simpleAutoName, this::simpleAuto);
        container.m_chooser.addRoutine(backsideL1Name, this::backsideL1);
    }

    // Example auto
    String simpleAutoName = "Simple Auto";
    public AutoRoutine simpleAuto() {
        final AutoRoutine routine = m_factory.newRoutine(simpleAutoName);
        final AutoTrajectory traj = routine.trajectory("1");
        
        routine.active().onTrue(
                traj.resetOdometry()
                        .andThen(traj.cmd())
                        .andThen(createAutoAlignCommand(new Pose2d(
                                ChoreoVariables.getPose("Lolipop1").getX(), 
                                ChoreoVariables.getPose("Lolipop1").getY(), 
                                ChoreoVariables.getPose("Lolipop1").getRotation()))));
        return routine;
    }

    // Example auto
    String backsideL1Name = "Backside L1";
    public AutoRoutine backsideL1() {
        final AutoRoutine routine = m_factory.newRoutine(backsideL1Name);
        var firstScore = routine.trajectory("BacksideL1(1)");
        var postScoreIntake = routine.trajectory("BacksideL1(2)");
        
        routine.active().onTrue(
                firstScore.resetOdometry()
                        .andThen(firstScore.cmd()
                                .andThen(waitSeconds(SCORE_WAIT))
                                .andThen(postScoreIntake.cmd())));

        // Use the StateMachine passed in via constructor
        firstScore.doneFor(0).onTrue(m_stateMachine.AlgaeIntake());
        
        return routine;
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the targetPose. 
     * 
     * @param targetPose The desired ending Pose2d
     * @return The Command to navigate to the given Pose2d.
     */
    public Command createAutoAlignCommand(Pose2d targetPose) {
        return new AutoAlign(new APTarget(targetPose), m_drivebase);
    }

    /**
     * Creates a new Command using the Autopilot AutoAlign to navigate to the targetPose. Takes a desired entry angle
     * when approaching the targetPose.
     * 
     * @param targetPose The desired ending Pose2d
     * @param entryAngle The desired angle to approach the targetPose with.
     * @return The Command to navigate to the given Pose2d.
     */
    public Command createAutoAlignCommand(Pose2d targetPose, Rotation2d entryAngle) {
        return new AutoAlign(new APTarget(targetPose).withEntryAngle(entryAngle), m_drivebase);
    }
}