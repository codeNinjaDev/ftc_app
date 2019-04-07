package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/***
 * A subsytem controls a single mechanism of the robot, such as driving, liting an arm/intake, or a climber.
 *
 * Each subsytem needs at least four methods: init(), update(), reset(), and stop()
 * ***/
public interface Subsystem {
    /*** Configures hardware or any thing that needs to be set up before subsystem runs ***/
    void init();

    /*** Resets all sensors and hardware ***/
    void reset();

    /*** Runs actual movement code in main loop ***/
    void update(Telemetry telemetry);

    /*** Stops all motors ***/
    void stop();
}
