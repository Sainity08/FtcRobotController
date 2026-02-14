package org.firstinspires.ftc.teamcode.Nero.Mecanisms;



import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class file {
    public List<Double> routine;


    public void init() {
        routine = new ArrayList<>();
    }


    public void FileWrite(double robotX, double robotY, double robotHeading){
        routine.add(robotX);
        routine.add(robotY);
        routine.add(robotHeading);

        String routineString = routine.toString();
        routineString = routineString.substring(1,routineString.length()-1);

        File file = AppUtil.getInstance().getSettingsFile("RobotPos.txt");
        ReadWriteFile.writeFile(file,routineString);
    }

    public void FileRead(){

        File Read = AppUtil.getInstance().getSettingsFile("RobotPos.txt");
        String[] types = ReadWriteFile.readFile(Read).trim().split(", ");

        for (String type : types){
            if(!type.isEmpty()){
                routine.add(Double.parseDouble(type));
            }
        }

    }


}