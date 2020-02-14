/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */

public class AutonomousPath {

    private String name;
    private String filename;

    AutonomousPath(String name, String filename) {

        this.name = name;
        this.filename = name;
    }

    public String getName() {
        return this.name;
    }

    public String getFilename() {
        return this.filename;
    }
}


