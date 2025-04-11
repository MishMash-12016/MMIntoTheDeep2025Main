package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class AutoSample7 {
    public static void main(String[] args) {

        final Pose2d scorePose = new Pose2d(-58, -52, Math.toRadians(230.5412));
        final Pose2d intakePose = new Pose2d(-26, -12, Math.toRadians(180));


        MeepMeep meepMeep = new MeepMeep(600);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(-41+3.77, -65, Math.toRadians(180)))

//                        TrajectoryActionBuilder driveToScorePreloadSample = drive.actionBuilder(currentPose)
                                .lineToLinearHeading(new Pose2d(-62, -54, Math.toRadians(244)))

//        TrajectoryActionBuilder driveToIntakeFirst = driveToScorePreloadSample.endTrajectory().fresh()
                .setTangent(Math.toRadians(244+180))
                .splineToConstantHeading(new Vector2d(-59, -46.2), Math.toRadians(244-180))

//        TrajectoryActionBuilder driveToScoreFirst = driveToIntakeFirst.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-62, -52.5, Math.toRadians(263)))

//        TrajectoryActionBuilder driveToIntakeSecondSample = driveToScoreFirst.endTrajectory().fresh()
                        .lineToLinearHeading(new Pose2d(-59.9, -48.5, Math.toRadians(268)))
//                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.45, MecanumDrive.PARAMS.maxProfileAccel * 0.6));

//        TrajectoryActionBuilder driveToScoreSecondSample = driveToIntakeSecondSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-61.6, -51.2, Math.toRadians(250)))
//                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

//        TrajectoryActionBuilder driveToIntakeThird = driveToScoreSecondSample.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-57.7, -46.7, Math.toRadians(298)))
//                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));

//        TrajectoryActionBuilder driveToScoreThird = driveToIntakeThird.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(-61.2, -51.4, Math.toRadians(245)))
//                        null, new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 0.7, MecanumDrive.PARAMS.maxProfileAccel));


                        //        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
                        .setTangent(Math.toRadians(70))
                        .splineToConstantHeading(new Vector2d(-39,-18), Math.toRadians(50))
                        .splineToSplineHeading(intakePose,Math.toRadians(0))
//
//                        .setTangent(Math.toRadians(200))
//                        .splineToSplineHeading(scorePose,Math.toRadians(240))

//                        ////        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()

//
//
//                        .lineToSplineHeading(new Pose2d(-37,-22, Math.toRadians(235.0079)))
//                        .splineToSplineHeading(intakePose,Math.toRadians(20))


//        TrajectoryActionBuilder driveToIntakeForth = driveToScoreThird.endTrajectory().fresh()
//                .setTangent(Math.toRadians(62))
//                .splineToSplineHeading(intakePose, Math.toRadians(15))
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//
////        TrajectoryActionBuilder driveToScoreForth = driveToIntakeForth.endTrajectory().fresh()
//                .setTangent(Math.toRadians(180))
//                .splineTo(new Vector2d(scorePose.component1(),scorePose.component2()), scorePose.component3())
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//
////        TrajectoryActionBuilder driveToIntakeFifth = driveToScoreForth.endTrajectory().fresh()
//                .setTangent(Math.toRadians(62))
//                        .splineTo(new Vector2d(intakePose.component1(),intakePose.component2()), 0)
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//
////        TrajectoryActionBuilder driveToScoreFifth = driveToIntakeFifth.endTrajectory().fresh()
//                .setTangent(Math.toRadians(180))
//                        .splineTo(new Vector2d(scorePose.component1(),scorePose.component2()), scorePose.component3())
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//
////        TrajectoryActionBuilder driveToIntakeSixth = driveToScoreFifth.endTrajectory().fresh()
//                .setTangent(Math.toRadians(62))
//                        .splineTo(new Vector2d(intakePose.component1(),intakePose.component2()), Math.toRadians(0))
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//
////        TrajectoryActionBuilder driveToScoreSixth = driveToIntakeSixth.endTrajectory().fresh()
//                .setTangent(Math.toRadians(180))
//                        .splineTo(new Vector2d(scorePose.component1(),scorePose.component2()), scorePose.component3())
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//
////        TrajectoryActionBuilder driveToPark = driveToScoreSixth.endTrajectory().fresh()
//                .setTangent(Math.toRadians(62))
//                        .splineTo(new Vector2d(intakePose.component1(),intakePose.component2()), Math.toRadians(0))
////                        , new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.1),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel * 1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

