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
        final double tangentsToIntakeSpecimen = 270;
        final Pose2d intakePose = new Pose2d(40, -65.5, Math.toRadians(90));
        final Pose2d scorePose = new Pose2d(8, -31.8, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,14.5)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90.00)))


//                        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90))
//                                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.4));

//
//        there isn't driveToEject its all conspiracy
//        TrajectoryActionBuilder driveToEject = drive.actionBuilder(new Pose2d(5.5, -29, Math.toRadians(270)))
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(0))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//
//
//        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(28, -37, Math.toRadians(225)), Math.toRadians(50))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.2)
//                );
//        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(28, -45, Math.toRadians(140)))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*2),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*1.5, MecanumDrive.PARAMS.maxProfileAccel*1.6));
//
//        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(38, -37, Math.toRadians(225)), Math.toRadians(50))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.2)
//                );
//        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .lineToLinearHeading(new Pose2d(38, -45, Math.toRadians(140)))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*2),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*1.5, MecanumDrive.PARAMS.maxProfileAccel*1.6));
//
//        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(46.5, -34, Math.toRadians(210)), Math.toRadians(50))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(46.5, -51, Math.toRadians(90)), Math.toRadians(270))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel*1.4),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*1.2, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//
//        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                        .splineToLinearHeading(new Pose2d(46.5, -68, Math.toRadians(90)), Math.toRadians(270))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));

//
//        TrajectoryActionBuilder resat = driveToIntakeFirstSpecimen.endTrajectory().fresh()
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(5.5, -61.23, Math.toRadians(270)), Math.toRadians(270));


//        TrajectoryActionBuilder driveToScoreFirstSpecimen = driveToIntakeFirstSpecimen.endTrajectory().fresh()
                        .setTangent(Math.toRadians(150))
                        .splineToConstantHeading(new Vector2d(scorePose.component1(), scorePose.component2()), scorePose.component3())
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
//
//        TrajectoryActionBuilder driveToIntakeSecondSpecimen = driveToScoreFirstSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                        .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//        TrajectoryActionBuilder driveToScoreSecondSpecimen = driveToIntakeSecondSpecimen.endTrajectory().fresh()
                        .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToIntakeThirdSpecimen = driveToScoreSecondSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                        .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//        TrajectoryActionBuilder driveToScoreThirdSpecimen = driveToIntakeThirdSpecimen.endTrajectory().fresh()
                        .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToIntakeForthSpecimen = driveToScoreThirdSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                        .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//        TrajectoryActionBuilder driveToScoreForthSpecimen = driveToIntakeForthSpecimen.endTrajectory().fresh()
                        .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToIntakeFifthSpecimen = driveToScoreForthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                        .splineToSplineHeading(new Pose2d(40, -66.5, Math.toRadians(90)), Math.toRadians(tangentsToIntakeSpecimen))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.2),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));
//        TrajectoryActionBuilder driveToScoreFifthSpecimen = driveToIntakeFifthSpecimen.endTrajectory().fresh()
                        .lineToLinearHeading(scorePose)

//        TrajectoryActionBuilder driveToPark = driveToScoreFifthSpecimen.endTrajectory().fresh()
                .setTangent(Math.toRadians(310))
                .splineToSplineHeading(new Pose2d(28, -50, Math.toRadians(140)), Math.toRadians(140+180))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel*0.8, MecanumDrive.PARAMS.maxProfileAccel * 1.2));

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


