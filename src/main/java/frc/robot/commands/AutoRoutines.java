// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
import static frc.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;
import static frc.robot.generated.ChoreoTraj.StartCycle;
import static frc.robot.generated.ChoreoTraj.FullCycle;
import static frc.robot.generated.ChoreoTraj.EndCycle;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.Position;
import frc.robot.LimelightHelpers;

public final class AutoRoutines {
    private final Swerve swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Limelight limelight;

    private final SubsystemCommands subsystemCommands;

    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public AutoRoutines(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Limelight limelight
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.limelight = limelight;

        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood 
        );

        this.autoFactory = swerve.createAutoFactory();
        this.autoChooser = new AutoChooser();
    }

    public void configure() {
        autoChooser.addRoutine("Outpost and Depot", this::outpostAndDepotRoutine);
        autoChooser.addRoutine("2 Cycle End Middle", this::cycle2EndMiddle);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private AutoRoutine outpostAndDepotRoutine() {
        final AutoRoutine routine = autoFactory.newRoutine("Outpost and Depot");
        final AutoTrajectory startToOutpost = OutpostAndDepotTrajectory$0.asAutoTraj(routine);
        final AutoTrajectory outpostToDepot = OutpostAndDepotTrajectory$1.asAutoTraj(routine);
        final AutoTrajectory depotToShootingPose = OutpostAndDepotTrajectory$2.asAutoTraj(routine);
        final AutoTrajectory shootingPoseToTower = OutpostAndDepotTrajectory$3.asAutoTraj(routine);

        routine.active().onTrue(
            Commands.sequence(
                startToOutpost.resetOdometry(),
                intake.runOnce(() -> intake.set(Intake.Position.INTAKE)),
                startToOutpost.cmd()
            )
        );

        startToOutpost.doneDelayed(1).onTrue(outpostToDepot.cmd());

        outpostToDepot.atTimeBeforeEnd(1).onTrue(intake.intakeCommand());
        outpostToDepot.doneDelayed(0.1).onTrue(depotToShootingPose.cmd());

        depotToShootingPose.active().whileTrue(limelight.idle());
        depotToShootingPose.atTime(0.5).onTrue(
            Commands.parallel(
                shooter.spinUpCommand(2600),
                hood.positionCommand(0.32)
            )
        );
        depotToShootingPose.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                shootingPoseToTower.cmd()
            )
        );

        shootingPoseToTower.active().whileTrue(limelight.idle());
        return routine;
    }

    private AutoRoutine cycle2EndMiddle() {
        final AutoRoutine routine = autoFactory.newRoutine("2 Cycle End Middle");
        final AutoTrajectory startCycle = StartCycle.asAutoTraj(routine);
        final AutoTrajectory fullCycle = FullCycle.asAutoTraj(routine);
        final AutoTrajectory endCycle = EndCycle.asAutoTraj(routine);

        //autoFactory.bind("stopIntake", intake.stopIntake());
        //autoFactory.bind("intake", intake.setIntake());
        // autoFactory.bind("shoot", subsystemCommands.aimAndShoot());
        // autoFactory.bind("stopShooter", shooter.stopCommand());
        // autoFactory.bind("startShooter", Commands.parallel(
        //     //Commands.runOnce(() -> LimelightHelpers.setPipelineIndex("",0)),
        //     shooter.spinUpCommand(2600)));

        routine.active().onTrue(
            Commands.sequence(
                startCycle.resetOdometry(),
                intake.runOnce(() -> intake.set(Position.INTAKE)),
                Commands.parallel(
                    hood.positionCommand(0.32),
                    startCycle.spawnCmd())
            )
        );

        startCycle.atTime("intake").onTrue(intake.intakeCommand());
        startCycle.atTime("stopIntake").onTrue(intake.stopIntake());

        startCycle.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                fullCycle.spawnCmd()
            )
        );

        fullCycle.done().onTrue(
            Commands.sequence(
                subsystemCommands.aimAndShoot()
                    .withTimeout(5),
                endCycle.spawnCmd()
            )
        );

        endCycle.active().whileTrue(limelight.idle());       
        return routine;
    }
}
