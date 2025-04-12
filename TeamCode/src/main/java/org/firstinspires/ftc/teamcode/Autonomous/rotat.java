package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandGroup.AutoSpecimensCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSampleCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.IntakeSpecimenCommand;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHere;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.driveToScoreFirstSpecimen;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHereToIntake;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHereToPark;
import org.firstinspires.ftc.teamcode.CommandGroup.roadRunnewr.runFromHereToScore;
import org.firstinspires.ftc.teamcode.CommandGroup.touchSensors;
import org.firstinspires.ftc.teamcode.Libraries.MMLib.MMOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Libraries.RoadRunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.MMSystems;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeEndUnitRotator;
import org.firstinspires.ftc.teamcode.SubSystems.LinearIntake;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringArm;
import org.firstinspires.ftc.teamcode.SubSystems.ScoringEndUnitElbow;
import org.firstinspires.ftc.teamcode.utils.OpModeType;
import org.firstinspires.ftc.teamcode.utils.ParallelCommandGroupNoCheck;

@Config
@Autonomous
public class rotat extends MMOpMode {
    static MMRobot robotInstance;
    static final double halfOpenClaw = 0.6;
    static final double rotator = 0;
    final double intakeArmPose = 0.58;

    //parking position
    public static final double tangentsToIntakeSpecimen = 310;
    public static final Pose2d intakePose = new Pose2d(40, -71, Math.toRadians(90));
    public static final Pose2d scorePose = new Pose2d(2, -28, Math.toRadians(90));


    public rotat() {
        super(OpModeType.NonCompetition.EXPERIMENTING);
    }


    @Override
    public void onInit() {
        robotInstance = MMRobot.getInstance();
        robotInstance.mmSystems.initRobotSystems(this);
        MMRobot.getInstance().mmSystems.vision.trackRed();
        MMRobot.getInstance().mmSystems.vision.switchToDetector();
        MMRobot.getInstance().mmSystems.vision.setAutonumus();

        Pose2d currentPose = new Pose2d(5.5, -61.23, Math.toRadians(270));
        robotInstance.mmSystems.initDriveTrain(currentPose);
        PinpointDrive drive = MMRobot.getInstance().mmSystems.driveTrain;

        MMRobot.getInstance().mmSystems.scoringClawEndUnit.closeScoringClaw();// pre load
        MMRobot.getInstance().mmSystems.linearIntake.setPosition(0);

        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(5, -61), Math.toRadians(225));
        new SequentialCommandGroup(
                new ActionCommand(driveToScorePreloadSample.build())
        ).schedule();

    }

    @Override
    public void run() {
        super.run();
        MMSystems.AutoPose = MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR();

        MMRobot.getInstance().mmSystems.expansionHub.pullBulkData();
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor back", touchSensors.getStateScoring());
        FtcDashboard.getInstance().getTelemetry().addData("touch sensor front", touchSensors.getStateIntake());
        FtcDashboard.getInstance().getTelemetry().addData("ang of the big robot for ori", getAng());
        FtcDashboard.getInstance().getTelemetry().addData("a", getAng() >= Math.toRadians(200));

        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void reset() {
        super.reset();

    }

    private static Command setupForPushing() {
        return new ParallelCommandGroup(
                //MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.CLOSED_POSE),
                robotInstance.mmSystems.intakeArm.setPosition(0.5),
                robotInstance.mmSystems.intakeEndUnitRotator.setPosition(rotator),
                robotInstance.mmSystems.intakEndUnit.setPose(halfOpenClaw)
        );
    }

    public static Command ThrowSample() {
        return new ParallelCommandGroup(
                MMRobot.getInstance().mmSystems.scoringEndUnitElbow.setPosition(ScoringEndUnitElbow.ScoringElbowState.PREPARE_SAMPLE_TRANSFER),
                MMRobot.getInstance().mmSystems.scoringArm.setPosition(ScoringArm.ScoringArmState.SAMPLE_TRANSFER_POSE),//be prepared for transfer
                MMRobot.getInstance().mmSystems.scoringClawEndUnit.openScoringClaw(),
                MMRobot.getInstance().mmSystems.linearIntake.setPosition(LinearIntake.LinearIntakeState.MAX_OPENING),
                MMRobot.getInstance().mmSystems.intakeArm.setPosition(IntakeArm.IntakeArmState.PREPARE_SAMPLE_INTAKE),
                MMRobot.getInstance().mmSystems.intakeEndUnitRotator.setPosition(IntakeEndUnitRotator.IntakeRotatorState.DEFAULT_POSE)
        );
    }

    public static double getAng() {
        double ang = Math.toDegrees(MMRobot.getInstance().mmSystems.driveTrain.pinpoint.getPositionRR().heading.toDouble());
        ang = ang < 0 ? ang + 360 : ang;
        return ang;
    }
}
