package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class AutoSample7 {
    public static void main(String[] args) {

        final Pose2d scorePose = new Pose2d(-58, -49, Math.toRadians(230));
        final Pose2d intakePose = new Pose2d(-24, -8, Math.toRadians(180));


        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)//+(150/25.4)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(-41, -61.23, Math.toRadians(270)))

//                        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                        .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(new Pose2d(-65, -49.7, Math.toRadians(253)),Math.toRadians(180))

//        TrajectoryActionBuilder driveToIntakeFirstSample = driveToScorePreloadSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-58.7, -48, Math.toRadians(245)))

//        TrajectoryActionBuilder driveToScoreFirstSample = driveToIntakeFirstSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-59.1, -52, Math.toRadians(247.7)))

//        TrajectoryActionBuilder driveToIntakeThird = driveToScoreFirstSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-58.7, -48.2, Math.toRadians(307)))

//        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-64.9, -47.5, Math.toRadians(259.33)))

//        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                .lineToLinearHeading(intakePose)

//        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
                .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
                .lineToLinearHeading(intakePose)

//        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
                .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
                .lineToLinearHeading(intakePose)

//        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
                .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
                .lineToLinearHeading(intakePose)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

