// package com.uni.frc.subsystems;

// import com.uni.frc.Ports;
// import com.uni.frc.subsystems.Requests.Request;

// import edu.wpi.first.wpilibj.motorcontrol.Spark;
 
// public class Lights extends Subsystem{
//     Spark lights;
//     Lights(){
//         lights = new Spark(Ports.Lights);
//     }
//     Color currentColor = Color.GREEN;
//     public enum Color{
//         GREEN(),
//         RED(),
//         YELLOW(),
//         RAINBOW();
//         double output = 0;
//         Color(double output){
//             this.output = output;
//         }
//     }
//     /*
//      * INTAKING,
//      * SHOOTING,
//      * CLIMB,
//      * AMP,
//      * SOURCE,
//      * OFF,
//      * IDLE,
//      * AUTO
//      * 
//      */
//     public void update(){
//         lights.set(currentColor.output);
//     }
//     public void setColor(Color color){
//         currentColor = color;
//     }
//     public Request setColorRequest(Color color){
//         return new Request() {
//             @Override
//             public void act() {
//                 setColor(color);
//             }
//         };
//     }
//     @Override
//     public void outputTelemetry() {
        
//     }
//     @Override
//     public void stop() {
        
//     }

// }
