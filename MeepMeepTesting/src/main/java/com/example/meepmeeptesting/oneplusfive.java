package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class oneplusfive {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        final double tangentsToScoreSpecimen = 135;
        final double tangentsToIntakeSpecimen = 310;
        final Pose2d intakePose = new Pose2d(40, -65.5, Math.toRadians(90));
        final Pose2d scorePose = new Pose2d(2, -24, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,16.2)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(5.5, -60, Math.toRadians(90.00)))



//      TrajectoryActionBuilder driveToScorePreload
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90))

//        TrajectoryActionBuilder driveToEject
                        .setTangent(Math.toRadians(260))
                        .splineToSplineHeading(new Pose2d(25, -45, Math.toRadians(140+180)), Math.toRadians(-10))


//        TrajectoryActionBuilder driveToPush1
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(28, -37, Math.toRadians(225)), Math.toRadians(50))
//                );
//        TrajectoryActionBuilder turnRobot
                .lineToLinearHeading(new Pose2d(28, -45, Math.toRadians(140)))
//
//        TrajectoryActionBuilder driveToPush2
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(38, -37, Math.toRadians(225)), Math.toRadians(50))
//        TrajectoryActionBuilder turnRobot2
                .lineToLinearHeading(new Pose2d(38, -45, Math.toRadians(140)))

//        TrajectoryActionBuilder driveToPush3
                .setTangent(Math.toRadians(60))
                .splineToSplineHeading(new Pose2d(46.5, -34, Math.toRadians(210)), Math.toRadians(50))
//        TrajectoryActionBuilder turnRobot3
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(46.5, -51, Math.toRadians(90)), Math.toRadians(270))

//        TrajectoryActionBuilder driveToIntakeFirstSpecimen
//                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46.5, -66.5, Math.toRadians(90)), Math.toRadians(270))
//        TrajectoryActionBuilder driveToScoreFirstSpecimen
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(scorePose, Math.toRadians(120))
//                .splineTo(new Vector2d(scorePose.component1(), scorePose.component2()),scorePose.component3())

//        TrajectoryActionBuilder driveToIntakeSecondSpecimen
                .setTangent(Math.toRadians(310))
                .splineToLinearHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
//        TrajectoryActionBuilder driveToScoreSecondSpecimen
                        .setTangent(Math.toRadians(140))
                        .splineToLinearHeading(scorePose, Math.toRadians(110))

////        TrajectoryActionBuilder driveToIntakeThirdSpecime
//                .setTangent(Math.toRadians(310))
//                .splineToLinearHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
////        TrajectoryActionBuilder driveToScoreThirdSpecimen
//                        .setTangent(Math.toRadians(150))
//                        .splineToLinearHeading(scorePose, Math.toRadians(100))
//
////        TrajectoryActionBuilder driveToIntakeForthSpecimen
//                .setTangent(Math.toRadians(310))
//                .splineToLinearHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
////        TrajectoryActionBuilder driveToScoreForthSpecimen
//                        .setTangent(Math.toRadians(150))
//                        .splineToLinearHeading(scorePose, Math.toRadians(100))
//
////        TrajectoryActionBuilder driveToIntakeFifthSpecimen
//                .setTangent(Math.toRadians(310))
//                .splineToLinearHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
////        TrajectoryActionBuilder driveToScoreFifthSpecimen
//                        .setTangent(Math.toRadians(150))
//                        .splineToLinearHeading(scorePose, Math.toRadians(100))
//
////        TrajectoryActionBuilder driveToPark = driveToScoreFifthSpecimen.endTrajectory().fresh()
//                .setTangent(Math.toRadians(310))
//                .splineToLinearHeading(new Pose2d(28, -50, Math.toRadians(140)), Math.toRadians(140+180))
////                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
////                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


