package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class AutoSample7 {
    public static void main(String[] args) {

        final Pose2d scorePose = new Pose2d(-59, -50, Math.toRadians(240));
        final Pose2d intakePose = new Pose2d(-26, -12, Math.toRadians(180));


        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)//+(150/25.4)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(-41+3.77, -65, Math.toRadians(180)))

//                        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                                .lineToLinearHeading(new Pose2d(-58.7, -52,Math.toRadians(247.5)))

//        TrajectoryActionBuilder driveToIntakeFirst = driveToScorePreloadSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-59.4, -48, Math.toRadians(245)))

//        TrajectoryActionBuilder driveToScoreFirst = driveToIntakeFirst.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-59.2, -51.8, Math.toRadians(257)))

//        TrajectoryActionBuilder driveToIntakeSecondSample = driveToScoreFirst.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-59.9, -50, Math.toRadians(268)))

//        TrajectoryActionBuilder driveToScoreSecondSample = driveToIntakeSecondSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-61.4, -51, Math.toRadians(250)))

//        TrajectoryActionBuilder driveToIntakeThird = driveToScoreSecondSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-57.7, -49, Math.toRadians(300)))

//        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-61.2, -51.7, Math.toRadians(245)))

//        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose,Math.toRadians(25))

//        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(scorePose,Math.toRadians(240))

//        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose,Math.toRadians(15))

//        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(scorePose,Math.toRadians(240))

//        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose,Math.toRadians(15))

//        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(scorePose,Math.toRadians(240))

//        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
                .setTangent(Math.toRadians(62))
                .splineToSplineHeading(intakePose,Math.toRadians(15))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

