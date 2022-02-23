package frc.robot.subsystems.Charts;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Grapher {
    private PrintWriter writer;

    public Grapher() {
        try {
            String time = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date());
            this.writer = new PrintWriter("src/main/java/frc/robot/subsystems/Charts/Graphs/" + time + ".csv");
        } catch (FileNotFoundException e){
            e.printStackTrace();
        }
    }

    public void write(Object value){
        writer.write(value + ", ");
    }

    public void flush() {
        writer.flush();
    }


}
