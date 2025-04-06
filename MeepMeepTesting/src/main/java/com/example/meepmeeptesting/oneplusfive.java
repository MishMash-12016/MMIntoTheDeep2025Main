package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class oneplusfive {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

         double maxWheelVel = 70;
        double maxProfileAccel = 70;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(11.02,40)//14.5
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 70, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(5.5, -62.73, Math.toRadians(90.00)))

//                        TrajectoryActionBuilder driveToScorePreload = drive.actionBuilder(currentPose)
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(5.5, -28), Math.toRadians(90))
//                                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.4));


        //there isn't driveToEject its all conspiracy
//        TrajectoryActionBuilder driveToEject = drive.actionBuilder(new Pose2d(5.5, -29, Math.toRadians(270)))
                .setTangent(Math.toRadians(260))
                .splineToLinearHeading(new Pose2d(25, -45, Math.toRadians(140)), Math.toRadians(0))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));


//        TrajectoryActionBuilder driveToPush1 = driveToEject.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(26, -34, Math.toRadians(215)), Math.toRadians(50))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//        TrajectoryActionBuilder turnRobot = driveToPush1.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(33, -51, Math.toRadians(150)), Math.toRadians(250))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//
//        TrajectoryActionBuilder driveToPush2 = turnRobot.endTrajectory().fresh()
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(35.5, -34, Math.toRadians(215)), Math.toRadians(50))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//        TrajectoryActionBuilder turnRobot2 = driveToPush2.endTrajectory().fresh()
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(43, -51, Math.toRadians(150)), Math.toRadians(250))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//
//        TrajectoryActionBuilder driveToPush3 = turnRobot2.endTrajectory().fresh()
                .setTangent(Math.toRadians(70))
                .splineToLinearHeading(new Pose2d(50, -34, Math.toRadians(220)), Math.toRadians(50))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//        TrajectoryActionBuilder turnRobot3 = driveToPush3.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(50.5, -51, Math.toRadians(90)), Math.toRadians(270))
//                        new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * 1.5),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel * 1.5));
//
//        TrajectoryActionBuilder driveToIntakeFirstSpecimen = turnRobot3.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(50.5, -68, Math.toRadians(90)), Math.toRadians(270))
//                        new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel),
//                        new ProfileAccelConstraint(MecanumDrive.PARAMS.minProfileAccel, MecanumDrive.PARAMS.maxProfileAccel));
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)

                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


